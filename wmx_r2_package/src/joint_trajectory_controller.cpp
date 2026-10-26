// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "joint_trajectory_controller.hpp"

#include <chrono>
#include <functional>
#include <sstream>
#include <thread>

using std::placeholders::_1;
using std::placeholders::_2;

using wmx3Api::AdvancedMotion;
using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;

namespace
{
struct ScopeExit
{
  std::function<void()> fn;
  ~ScopeExit() {fn();}
};

std::string errorText(int err)
{
  char errString[256] = {};
  AdvancedMotion::ErrorToString(err, errString, sizeof(errString));
  return errString;
}
}  // namespace

JointTrajectoryControllerApi::JointTrajectoryControllerApi(const rclcpp::Logger & logger)
: logger_(logger)
{
}

JointTrajectoryControllerApi::~JointTrajectoryControllerApi()
{
  releaseDevice();
}

int JointTrajectoryControllerApi::attachDevice(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (isDeviceAttached_) {
    message = "Already attached to the WMX3 device";
    return ErrorCode::None;
  }

  int err = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout_);
  if (err != ErrorCode::None) {
    if (err == ErrorCode::StartProcessLockError) {
      message = "Failed to attach to device (lock busy). Is the engine communicating?";
    } else {
      message = "Failed to attach to device. Error=" + std::to_string(err) +
        " (" + errorText(err) + ")";
    }
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = wmx3Lib_.SetDeviceName(deviceName_);
  if (err != ErrorCode::None) {
    message = "Failed to name the device '" + std::string(deviceName_) + "'. Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    wmx3Lib_.CloseDevice();
    return err;
  }

  cm_ = CoreMotion(&wmx3Lib_);
  am_ = AdvancedMotion(&wmx3Lib_);

  err = createSplineBuffer(message);
  if (err != ErrorCode::None) {
    wmx3Lib_.CloseDevice();
    return err;
  }

  isDeviceAttached_ = true;

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void JointTrajectoryControllerApi::releaseDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  freeSplineBuffer();

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorText(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "Device closed");
    isDeviceAttached_ = false;
  }
}

int JointTrajectoryControllerApi::createSplineBuffer(std::string & message)
{
  const int err = am_.advMotion->CreateSplineBuffer(kSplineChannel, kMaxTrajectoryPoints);
  if (err != ErrorCode::None) {
    message = "Failed to create the spline buffer on channel " +
      std::to_string(kSplineChannel) + ". Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  splinePoints_.resize(kMaxTrajectoryPoints);
  splineTimesMs_.resize(kMaxTrajectoryPoints);
  return ErrorCode::None;
}

void JointTrajectoryControllerApi::freeSplineBuffer()
{
  const int err = am_.advMotion->FreeSplineBuffer(kSplineChannel);
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(
      logger_, "Failed to free the spline buffer. Error=%d (%s)", err, errorText(err).c_str());
  }
}

int JointTrajectoryControllerApi::setAxes(
  const std::vector<int64_t> & axes, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (axes.empty()) {
    message = "No axes configured: 'joint_axes' is empty.";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return ErrorCode::ArgumentOutOfRange;
  }

  if (axes.size() > static_cast<size_t>(wmx3Api::constants::maxSplineDimensions)) {
    message = "Too many axes: " + std::to_string(axes.size()) + " configured, the SDK allows " +
      std::to_string(wmx3Api::constants::maxSplineDimensions) + " spline dimensions.";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return ErrorCode::ArgumentOutOfRange;
  }

  for (const int64_t axis : axes) {
    if (axis < 0 || axis >= wmx3Api::constants::maxAxes) {
      message = "Invalid axis " + std::to_string(axis) + ": must be in [0, " +
        std::to_string(wmx3Api::constants::maxAxes) + ").";
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      return ErrorCode::ArgumentOutOfRange;
    }
  }

  axisCount_ = axes.size();
  axisSel_.axisCount = static_cast<int>(axisCount_);
  splineCommand_.dimensionCount = static_cast<unsigned int>(axisCount_);
  for (size_t i = 0; i < axisCount_; ++i) {
    axisSel_.axis[i] = static_cast<int>(axes[i]);
    splineCommand_.axis[i] = static_cast<int>(axes[i]);
  }

  message = "Driving " + std::to_string(axisCount_) + " axes";
  return ErrorCode::None;
}

int JointTrajectoryControllerApi::startCSplinePos(
  const std::vector<std::vector<double>> & positions,
  const std::vector<double> & timesMs,
  std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!isDeviceAttached_) {
    message = "Cannot start the trajectory. Device is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  if (axisCount_ == 0) {
    message = "Cannot start the trajectory. No axes are selected.";
    return ErrorCode::ArgumentOutOfRange;
  }

  if (positions.size() != timesMs.size()) {
    message = "Point count mismatch: " + std::to_string(positions.size()) +
      " position sets against " + std::to_string(timesMs.size()) + " time stamps.";
    return ErrorCode::ArgumentOutOfRange;
  }

  if (positions.empty() || positions.size() > kMaxTrajectoryPoints) {
    message = "Invalid trajectory point count " + std::to_string(positions.size()) +
      ": must be in [1, " + std::to_string(kMaxTrajectoryPoints) + "].";
    return ErrorCode::ArgumentOutOfRange;
  }

  for (size_t i = 0; i < positions.size(); ++i) {
    if (positions[i].size() != axisCount_) {
      message = "Trajectory point " + std::to_string(i) + " carries " +
        std::to_string(positions[i].size()) + " positions, expected " +
        std::to_string(axisCount_) + ".";
      return ErrorCode::ArgumentOutOfRange;
    }
    for (size_t j = 0; j < axisCount_; ++j) {
      splinePoints_[i].pos[j] = positions[i][j];
    }
    splineTimesMs_[i] = timesMs[i];
  }

  const int err = am_.advMotion->StartCSplinePos(
    kSplineChannel, &splineCommand_, static_cast<unsigned int>(positions.size()),
    splinePoints_.data(), splineTimesMs_.data());
  if (err != ErrorCode::None) {
    message = "StartCSplinePos failed. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Trajectory of " + std::to_string(positions.size()) + " points started";
  return ErrorCode::None;
}

int JointTrajectoryControllerApi::getInPos(bool & inPos, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  inPos = false;

  if (!isDeviceAttached_) {
    message = "Cannot read the axis status. Device is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  CoreMotionStatus status;
  const int err = cm_.GetStatus(&status);
  if (err != ErrorCode::None) {
    message = "GetStatus failed. Error=" + std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  for (int i = 0; i < axisSel_.axisCount; ++i) {
    if (!status.axesStatus[axisSel_.axis[i]].inPos) {
      return ErrorCode::None;
    }
  }

  inPos = true;
  return ErrorCode::None;
}

int JointTrajectoryControllerApi::stopAxes(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  int err = cm_.motion->Stop(&axisSel_);
  if (err != ErrorCode::None) {
    message = "Stop failed. Error=" + std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = cm_.motion->Wait(&axisSel_);
  if (err != ErrorCode::None) {
    message = "Wait failed. Error=" + std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Axes stopped";
  return ErrorCode::None;
}

JointTrajectoryController::JointTrajectoryController()
: LifecycleNode("joint_trajectory_controller")
{
  setRosParameter();

  api_ = std::make_unique<JointTrajectoryControllerApi>(this->get_logger());

  RCLCPP_INFO(
    this->get_logger(), "joint_trajectory_controller is unconfigured, waiting for configure...");
}

JointTrajectoryController::~JointTrajectoryController()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller stopped");
}

bool JointTrajectoryController::isNodeActive() const
{
  return isNodeActive_.load();
}

std::string JointTrajectoryController::notActiveMessage()
{
  return "joint_trajectory_controller is not active (state: " +
         this->get_current_state().label() + ").";
}

void JointTrajectoryController::setRosParameter()
{
  jointAxes_ = this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  jointTrajectoryAction_ = this->declare_parameter<std::string>(
    "joint_trajectory_action", "/joint_trajectory_action/no_param");

  std::string jointAxesText;
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    if (i > 0) {jointAxesText += ", ";}
    jointAxesText += std::to_string(jointAxes_[i]);
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", jointAxesText.c_str());
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_action: %s", jointTrajectoryAction_.c_str());
  RCLCPP_INFO(this->get_logger(), "===========================");
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring joint_trajectory_controller...");

  std::string message;
  if (api_->attachDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  if (api_->setAxes(jointAxes_, message) != ErrorCode::None) {
    api_->releaseDevice();
    return CallbackReturn::FAILURE;
  }

  servoNodeResetPub_ = this->create_publisher<control_msgs::msg::JointJog>(
    "/servo_node/delta_joint_cmds", 10);

  execActivePub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local());

  actionServer_ = rclcpp_action::create_server<FollowJointTrajectory>(
    this,
    jointTrajectoryAction_,
    std::bind(&JointTrajectoryController::handleGoal, this, _1, _2),
    std::bind(&JointTrajectoryController::handleCancel, this, _1),
    std::bind(&JointTrajectoryController::handleAccepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is configured");
  return CallbackReturn::SUCCESS;
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  isNodeActive_ = true;

  publishExecActive(false);

  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is active");
  return CallbackReturn::SUCCESS;
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  isNodeActive_ = false;

  publishExecActive(false);

  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is inactive");
  return CallbackReturn::SUCCESS;
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  isNodeActive_ = false;

  actionServer_.reset();
  execActivePub_.reset();
  servoNodeResetPub_.reset();

  api_->releaseDevice();

  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is cleaned up");
  return CallbackReturn::SUCCESS;
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

rclcpp_action::GoalResponse JointTrajectoryController::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  (void)uuid;
  (void)goal;

  if (!isNodeActive()) {
    RCLCPP_WARN(this->get_logger(), "Goal rejected: %s", notActiveMessage().c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryController::handleCancel(
  std::shared_ptr<GoalHandleFJT> goalHandle)
{
  (void)goalHandle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryController::handleAccepted(std::shared_ptr<GoalHandleFJT> goalHandle)
{
  std::thread{std::bind(&JointTrajectoryController::executeGoal, this, _1), goalHandle}.detach();
}

void JointTrajectoryController::publishExecActive(bool active)
{
  if (!execActivePub_) {
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = active;
  execActivePub_->publish(msg);
}

void JointTrajectoryController::resetServo(const std::vector<std::string> & jointNames)
{
  if (!servoNodeResetPub_) {
    return;
  }

  control_msgs::msg::JointJog jog;
  jog.header.stamp = this->get_clock()->now();
  jog.joint_names = jointNames;
  jog.velocities.assign(jointNames.size(), 0.0);
  servoNodeResetPub_->publish(jog);
}

bool JointTrajectoryController::buildSplineInput(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  std::vector<std::vector<double>> & positions,
  std::vector<double> & timesMs,
  std::string & message)
{
  const size_t axisCount = jointAxes_.size();
  size_t pointCount = trajectory.points.size();

  if (pointCount == 0) {
    message = "Trajectory carries no points.";
    return false;
  }

  if (pointCount > JointTrajectoryControllerApi::kMaxTrajectoryPoints) {
    message = "Too many trajectory point size! current points:" + std::to_string(pointCount) +
      " / max traj points:" +
      std::to_string(JointTrajectoryControllerApi::kMaxTrajectoryPoints);
    return false;
  }

  if (pointCount > 1) {
    const double lastGap =
      rclcpp::Duration(trajectory.points[pointCount - 1].time_from_start).seconds() -
      rclcpp::Duration(trajectory.points[pointCount - 2].time_from_start).seconds();
    if (lastGap < 1e-3) {
      pointCount -= 1;
    }
  }

  positions.assign(pointCount, std::vector<double>(axisCount, 0.0));
  timesMs.assign(pointCount, 0.0);

  for (size_t i = 0; i < pointCount; ++i) {
    const auto & pt = trajectory.points[i];

    if (pt.positions.size() < axisCount) {
      message = "Trajectory point " + std::to_string(i) + " carries " +
        std::to_string(pt.positions.size()) + " positions, expected at least " +
        std::to_string(axisCount) + ".";
      return false;
    }

    timesMs[i] = rclcpp::Duration(pt.time_from_start).seconds() * 1000.0;
    for (size_t j = 0; j < axisCount; ++j) {
      positions[i][j] = pt.positions[j];
    }
  }

  timesMs[0] = 0.0;

  return true;
}

void JointTrajectoryController::executeGoal(std::shared_ptr<GoalHandleFJT> goalHandle)
{
  const auto goal = goalHandle->get_goal();
  const auto & trajectory = goal->trajectory;

  publishExecActive(true);
  const auto servoJointNames = trajectory.joint_names;
  ScopeExit onExit{[this, servoJointNames]() {
      publishExecActive(false);
      resetServo(servoJointNames);
    }};

  RCLCPP_INFO(
    this->get_logger(), "Received a new trajectory goal! Point number: [%zu]",
    trajectory.points.size());

  auto result = std::make_shared<FollowJointTrajectory::Result>();

  std::vector<std::vector<double>> positions;
  std::vector<double> timesMs;
  std::string message;

  if (!buildSplineInput(trajectory, positions, timesMs, message)) {
    RCLCPP_WARN(this->get_logger(), "%s Aborting current goal.", message.c_str());
    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
    result->error_string = message;
    goalHandle->abort(result);
    return;
  }

  logTrajectory(trajectory);

  if (positions.empty()) {
    RCLCPP_INFO(this->get_logger(), "Point count is zero. It is already in the targeted position");
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    goalHandle->succeed(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Command Start!!!");
  int err = api_->startCSplinePos(positions, timesMs, message);
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    result->error_code = err;
    result->error_string = message;
    goalHandle->abort(result);
    return;
  }

  while (true) {
    if (goalHandle->is_canceling()) {
      api_->stopAxes(message);
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      goalHandle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled, axes stopped");
      return;
    }

    bool inPos = false;
    err = api_->getInPos(inPos, message);
    if (err != ErrorCode::None) {
      RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
      result->error_code = err;
      result->error_string = message;
      goalHandle->abort(result);
      return;
    }
    if (inPos) {break;}

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
  goalHandle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully");
}

void JointTrajectoryController::logTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  std::ostringstream jn;
  for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    if (i) {jn << ", ";}
    jn << trajectory.joint_names[i];
  }
  RCLCPP_INFO(this->get_logger(), "Joint Names: [%s]", jn.str().c_str());

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & pt = trajectory.points[i];
    std::ostringstream pos, vel, acc;
    for (size_t k = 0; k < pt.positions.size(); ++k) {
      if (k) {pos << ", ";}
      pos << pt.positions[k];
    }
    for (size_t k = 0; k < pt.velocities.size(); ++k) {
      if (k) {vel << ", ";}
      vel << pt.velocities[k];
    }
    for (size_t k = 0; k < pt.accelerations.size(); ++k) {
      if (k) {acc << ", ";}
      acc << pt.accelerations[k];
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Point %zu: Positions: [%s], Velocities: [%s], "
      "Accelerations: [%s], TimeFromStart: %d s %u ns",
      i, pos.str().c_str(), vel.str().c_str(), acc.str().c_str(),
      pt.time_from_start.sec, pt.time_from_start.nanosec);

    if (i != 0) {
      const rclcpp::Duration durationCur(trajectory.points[i].time_from_start);
      const rclcpp::Duration durationPre(trajectory.points[i - 1].time_from_start);
      RCLCPP_INFO(
        this->get_logger(), "Time interval: %f",
        (durationCur - durationPre).seconds());
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTrajectoryController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
