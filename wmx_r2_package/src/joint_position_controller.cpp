// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "joint_position_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

using wmx3Api::ApiBuffer;
using wmx3Api::ApiBufferCondition;
using wmx3Api::ApiBufferConditionNodeType;
using wmx3Api::ApiBufferConditionType;
using wmx3Api::ApiBufferMultiCondition;
using wmx3Api::ApiBufferOptions;
using wmx3Api::ApiBufferStatus;
using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::Motion;
using wmx3Api::ProfileType;

namespace
{
std::string errorToString(int err)
{
  char errString[256] = {};
  CoreMotion::ErrorToString(err, errString, sizeof(errString));
  return errString;
}

std::string failure(const char * what, int err)
{
  return std::string(what) + " failed. Error=" + std::to_string(err) + " (" +
         errorToString(err) + ")";
}

// Smallest per-axis velocity handed to the engine. An axis that does not move in this
// command still needs a positive limit; it does not constrain the composite speed.
constexpr double kVelocityFloor = 1e-6;

// Servo velocity below this counts as standing still.
constexpr double kStillVelocity = 1e-4;
}  // namespace

JointPositionControllerApi::JointPositionControllerApi(const rclcpp::Logger & logger)
: logger_(logger), cm_(&wmx3Lib_), cmRec_(&wmx3Rec_), ab_(&wmx3Lib_), abRec_(&wmx3Rec_)
{
}

JointPositionControllerApi::~JointPositionControllerApi()
{
  closeDevice();
}

int JointPositionControllerApi::createDevice(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  int err = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout_);
  if (err != ErrorCode::None) {
    if (err == ErrorCode::StartProcessLockError) {
      message = "Failed to attach to device (lock busy). Is the engine communicating?";
    } else {
      message = "Failed to attach to device. Error=" + std::to_string(err) +
        " (" + errorToString(err) + ")";
    }
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = wmx3Lib_.SetDeviceName(deviceName_);
  if (err != ErrorCode::None) {
    message = "Failed to name the device '" + std::string(deviceName_) + "'. Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    wmx3Lib_.CloseDevice();
    return err;
  }

  cm_ = CoreMotion(&wmx3Lib_);
  ab_ = ApiBuffer(&wmx3Lib_);

  err = wmx3Rec_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout_);
  if (err == ErrorCode::None) {
    err = wmx3Rec_.SetDeviceName("joint_position_controller_rec");
  }
  if (err != ErrorCode::None) {
    message = failure("CreateDevice (recording device)", err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    wmx3Rec_.CloseDevice();
    wmx3Lib_.CloseDevice();
    return err;
  }
  recAttached_ = true;
  cmRec_ = CoreMotion(&wmx3Rec_);
  abRec_ = ApiBuffer(&wmx3Rec_);

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void JointPositionControllerApi::closeDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  closeChannelLocked();
  if (recAttached_) {
    wmx3Rec_.CloseDevice();
    recAttached_ = false;
  }

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorToString(err).c_str());
    return;
  }

  RCLCPP_INFO(logger_, "Device closed");
}

int JointPositionControllerApi::setAxisSelection(
  const std::vector<int64_t> & axes, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (axes.empty()) {
    message = "No axes configured: 'joint_axes' is empty.";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return ErrorCode::ArgumentOutOfRange;
  }

  if (axes.size() > static_cast<size_t>(wmx3Api::constants::maxAxes)) {
    message = "Too many axes: " + std::to_string(axes.size()) + " configured, the SDK allows " +
      std::to_string(wmx3Api::constants::maxAxes) + ".";
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

  axisSel_.axisCount = static_cast<int>(axes.size());
  for (size_t i = 0; i < axes.size(); ++i) {
    axisSel_.axis[i] = static_cast<int>(axes[i]);
  }

  message = "Driving " + std::to_string(axes.size()) + " axes";
  return ErrorCode::None;
}

int JointPositionControllerApi::getStatus(
  const std::vector<int> & axes, std::vector<double> & posCmd, std::vector<double> & velCmd,
  std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  CoreMotionStatus status;
  const int err = cm_.GetStatus(&status);
  if (err != ErrorCode::None) {
    message = failure("GetStatus", err);
    return err;
  }

  posCmd.clear();
  velCmd.clear();
  for (const int axis : axes) {
    const auto & s = status.axesStatus[axis];
    if (!s.servoOn || s.ampAlarm) {
      message = "Axis " + std::to_string(axis) + " not ready (servoOn=" +
        std::to_string(s.servoOn) + ", ampAlarm=" + std::to_string(s.ampAlarm) + ")";
      return ErrorCode::ArgumentOutOfRange;
    }
    posCmd.push_back(s.posCmd);
    velCmd.push_back(s.velocityCmd);
  }

  return ErrorCode::None;
}

void JointPositionControllerApi::fill(
  Motion::LinearIntplCommand & intpl, const std::vector<int> & axes,
  const std::vector<double> & targets, const std::vector<double> & maxVelocity,
  const std::vector<double> & maxAcc)
{
  intpl.axisCount = static_cast<unsigned int>(axes.size());
  for (size_t i = 0; i < axes.size(); ++i) {
    intpl.axis[i] = axes[i];
    intpl.target[i] = targets[i];
    intpl.maxVelocity[i] = maxVelocity[i];
    intpl.maxAcc[i] = maxAcc[i];
    intpl.maxDec[i] = maxAcc[i];
  }
  intpl.profile.type = ProfileType::Trapezoidal;
  intpl.profile.velocity = 0.0;
  intpl.profile.acc = 0.0;
  intpl.profile.dec = 0.0;
}

// One API buffer channel carries the stream. The engine executes it in its own
// real-time cycle, so a recorded override fires the moment its Wait releases.
int JointPositionControllerApi::openChannel(unsigned int channel, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  channel_ = channel;
  int err = ab_.CreateApiBuffer(channel_, 1, wmx3Api::SizeUnit::Megabyte);
  if (err != ErrorCode::None) {
    // A previous run left the channel allocated: reclaim it.
    ab_.Halt(channel_);
    ab_.Clear(channel_);
    ab_.FreeApiBuffer(channel_);
    err = ab_.CreateApiBuffer(channel_, 1, wmx3Api::SizeUnit::Megabyte);
  }
  if (err != ErrorCode::None) {
    message = failure("CreateApiBuffer", err);
    return err;
  }

  // A refused block must not halt the stream; the next point replaces it.
  ApiBufferOptions options;
  options.stopOnError = false;
  options.autoRewind = false;
  options.stopOnLastBlock = false;
  err = ab_.SetOptions(channel_, &options);
  if (err == ErrorCode::None) {
    err = ab_.Execute(channel_);
  }
  if (err != ErrorCode::None) {
    message = failure("ApiBuffer setup", err);
    ab_.FreeApiBuffer(channel_);
    return err;
  }

  channelOpen_ = true;
  message = "API buffer channel " + std::to_string(channel_) + " executing";
  return ErrorCode::None;
}

void JointPositionControllerApi::closeChannelLocked()
{
  if (!channelOpen_) {
    return;
  }
  ab_.Halt(channel_);
  ab_.Clear(channel_);
  ab_.FreeApiBuffer(channel_);
  channelOpen_ = false;
}

// Records "wait until the running motion accepts an override (or the axes are
// idle), then StartLinearIntplPos". The engine evaluates the wait every cycle, so
// the override lands on the first cycle it can, instead of being refused.
int JointPositionControllerApi::recordOverride(
  const std::vector<int> & axes,
  const std::vector<double> & targets,
  const std::vector<double> & maxVelocity,
  const std::vector<double> & maxAcc,
  std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!channelOpen_ || axes.empty()) {
    message = "API buffer channel not open";
    return ErrorCode::ArgumentOutOfRange;
  }

  ApiBufferCondition idle;
  idle.bufferConditionType = ApiBufferConditionType::AxisIdle;
  idle.arg.axisIdle.axisCount = static_cast<unsigned int>(axes.size());
  for (size_t i = 0; i < axes.size(); ++i) {
    idle.arg.axisIdle.axis[i] = axes[i];
  }
  ApiBufferCondition ready;
  ready.bufferConditionType = ApiBufferConditionType::MotionStartedOverrideReady;
  ready.arg.motionStartedOverrideReady.axis = axes.front();

  ApiBufferMultiCondition when;
  const int root = when.AddGateNode(ApiBufferConditionNodeType::Or);
  when.AddLeafNode(&idle, root);
  when.AddLeafNode(&ready, root);

  Motion::LinearIntplCommand intpl;
  fill(intpl, axes, targets, maxVelocity, maxAcc);

  int err = abRec_.StartRecordBufferChannel(channel_);
  if (err != ErrorCode::None) {
    message = failure("StartRecordBufferChannel", err);
    return err;
  }
  int recErr = abRec_.Wait(&when);
  if (recErr == ErrorCode::None) {
    recErr = cmRec_.motion->StartLinearIntplPos(&intpl);
  }
  err = abRec_.EndRecordBufferChannel();
  if (recErr != ErrorCode::None) {
    message = failure("Recording override", recErr);
    return recErr;
  }
  if (err != ErrorCode::None) {
    message = failure("EndRecordBufferChannel", err);
  }
  return err;
}

int JointPositionControllerApi::channelStatus(
  int64_t & remaining, int64_t & errors, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  ApiBufferStatus status;
  const int err = ab_.GetStatus(channel_, &status);
  if (err != ErrorCode::None) {
    message = failure("ApiBuffer::GetStatus", err);
    return err;
  }
  remaining = status.remainingBlockCount;
  errors = status.errorCount;
  if (status.state == wmx3Api::ApiBufferState::Stop ||
    status.state == wmx3Api::ApiBufferState::Idle)
  {
    ab_.Execute(channel_);
  }
  return ErrorCode::None;
}

int JointPositionControllerApi::stop(unsigned int waitTimeout, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (channelOpen_) {
    ab_.Halt(channel_);
    ab_.Clear(channel_);
  }

  int err = cm_.motion->Stop(&axisSel_);
  if (err != ErrorCode::None) {
    message = failure("Stop", err);
    return err;
  }

  err = cm_.motion->Wait(&axisSel_, waitTimeout);
  if (err != ErrorCode::None) {
    message = failure("Wait", err);
    return err;
  }

  if (channelOpen_) {
    ab_.Execute(channel_);
  }

  message = "Axes stopped";
  return ErrorCode::None;
}

JointPositionController::JointPositionController()
: LifecycleNode("joint_position_controller")
{
  setRosParameter();

  api_ = std::make_unique<JointPositionControllerApi>(this->get_logger());

  RCLCPP_INFO(
    this->get_logger(), "joint_position_controller is unconfigured, waiting for configure...");
}

JointPositionController::~JointPositionController()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "joint_position_controller stopped");
}

std::vector<double> JointPositionController::perJoint(
  const std::string & name, std::vector<double> values)
{
  if (values.size() == 1) {
    values.assign(jointAxes_.size(), values.front());
  }
  const bool valid = values.size() == jointAxes_.size() &&
    std::all_of(values.begin(), values.end(), [](double v) {return std::isfinite(v) && v > 0.0;});
  if (!valid) {
    RCLCPP_WARN(
      this->get_logger(), "%s needs one positive value or one per joint (%zu), using 2.0",
      name.c_str(), jointAxes_.size());
    values.assign(jointAxes_.size(), 2.0);
  }
  return values;
}

void JointPositionController::setRosParameter()
{
  jointAxes_ = this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  jointNames_ = this->declare_parameter<std::vector<std::string>>(
    "joint_name", {"j1", "j2", "j3", "j4", "j5", "j6"});
  jointTrajectoryTopic_ = this->declare_parameter<std::string>(
    "joint_trajectory_topic", "/joint_trajectory_topic/no_param");
  publishPeriod_ = this->declare_parameter<double>("publish_period", 0.025);
  lookaheadTime_ = this->declare_parameter<double>("lookahead_time", 0.05);
  minOverrideInterval_ = this->declare_parameter<double>("min_override_interval", 0.0);
  bufferChannel_ = this->declare_parameter<int>("api_buffer_channel", 0);
  streamTimeout_ = this->declare_parameter<double>("stream_timeout", 0.1);
  positionTolerance_ = this->declare_parameter<double>("position_tolerance", 1e-5);
  maxVelocity_ = perJoint(
    "max_velocity", this->declare_parameter<std::vector<double>>(
      "max_velocity", std::vector<double>{2.0}));
  maxAcceleration_ = perJoint(
    "max_acceleration", this->declare_parameter<std::vector<double>>(
      "max_acceleration", std::vector<double>{2.0}));

  if (!std::isfinite(publishPeriod_) || publishPeriod_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "publish_period %f invalid, using 0.025", publishPeriod_);
    publishPeriod_ = 0.025;
  }
  acceptInterval_ = publishPeriod_;
  if (!std::isfinite(lookaheadTime_) || lookaheadTime_ < publishPeriod_) {
    RCLCPP_WARN(
      this->get_logger(), "lookahead_time %f must be >= publish_period, using %f",
      lookaheadTime_, 2.0 * publishPeriod_);
    lookaheadTime_ = 2.0 * publishPeriod_;
  }
  if (!std::isfinite(streamTimeout_) || streamTimeout_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "stream_timeout %f invalid, using 0.1", streamTimeout_);
    streamTimeout_ = 0.1;
  }
  if (!std::isfinite(positionTolerance_) || positionTolerance_ < 0.0) {
    RCLCPP_WARN(
      this->get_logger(), "position_tolerance %f invalid, using 1e-5", positionTolerance_);
    positionTolerance_ = 1e-5;
  }

  axes_.clear();
  for (const int64_t axis : jointAxes_) {
    axes_.push_back(static_cast<int>(axis));
  }
  for (size_t i = 0; i < jointNames_.size() && i < jointAxes_.size(); ++i) {
    indexByName_[jointNames_[i]] = i;
  }

  auto join = [](const auto & values) {
      std::string text;
      for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {text += ", ";}
        text += std::to_string(values[i]);
      }
      return text;
    };

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", join(jointAxes_).c_str());
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_topic: %s", jointTrajectoryTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "publish_period: %f", publishPeriod_);
  RCLCPP_INFO(this->get_logger(), "lookahead_time: %f", lookaheadTime_);
  RCLCPP_INFO(this->get_logger(), "min_override_interval: %f", minOverrideInterval_);
  RCLCPP_INFO(this->get_logger(), "api_buffer_channel: %d", bufferChannel_);
  RCLCPP_INFO(this->get_logger(), "stream_timeout: %f", streamTimeout_);
  RCLCPP_INFO(this->get_logger(), "position_tolerance: %f", positionTolerance_);
  RCLCPP_INFO(this->get_logger(), "max_velocity: [%s]", join(maxVelocity_).c_str());
  RCLCPP_INFO(this->get_logger(), "max_acceleration: [%s]", join(maxAcceleration_).c_str());
  RCLCPP_INFO(this->get_logger(), "===========================");

  paramCallback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if (p.get_name() == "min_override_interval") {
          minOverrideInterval_ = std::max(0.0, p.as_double());
        } else if (p.get_name() == "lookahead_time") {
          lookaheadTime_ = std::max(publishPeriod_, p.as_double());
        }
      }
      return result;
    });
}

JointPositionController::CallbackReturn JointPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring joint_position_controller...");

  std::string message;
  if (api_->createDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  if (api_->setAxisSelection(jointAxes_, message) != ErrorCode::None) {
    api_->closeDevice();
    return CallbackReturn::FAILURE;
  }

  if (api_->openChannel(static_cast<unsigned int>(bufferChannel_), message) != ErrorCode::None) {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    api_->closeDevice();
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(this->get_logger(), "%s", message.c_str());

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is configured");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  execActiveSub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local(),
    std::bind(&JointPositionController::execActiveCallback, this, _1));

  jointTrajectorySub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    jointTrajectoryTopic_, rclcpp::QoS(1).best_effort(),
    std::bind(&JointPositionController::jointTrajectoryCallback, this, _1));

  watchdogTimer_ = this->create_wall_timer(
    10ms, std::bind(&JointPositionController::watchdogCallback, this));

  retryTimer_ = this->create_wall_timer(
    2ms, [this]() {
      if (pending_ && !inExecution_.load()) {
        send();
      }
    });

  LifecycleNode::on_activate(previous_state);

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is active");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  retryTimer_.reset();
  watchdogTimer_.reset();
  jointTrajectorySub_.reset();
  execActiveSub_.reset();

  haltAxes("deactivating");

  LifecycleNode::on_deactivate(previous_state);

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is inactive");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  api_->closeDevice();

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is cleaned up");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(previous_state);
  }

  return on_cleanup(previous_state);
}

void JointPositionController::execActiveCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (inExecution_.exchange(msg->data) == msg->data) {
    return;
  }

  if (msg->data) {
    haltAxes("move_group execution started");
  }

  RCLCPP_INFO(
    this->get_logger(), "Servo commands %s (move_group execution %s)",
    msg->data ? "blocked" : "allowed", msg->data ? "started" : "finished");
}

void JointPositionController::haltAxes(const char * reason)
{
  const bool wasStreaming = streaming_;
  streaming_ = false;
  havePrevious_ = false;
  pending_ = false;
  if (!wasStreaming) {
    return;
  }

  std::string message;
  if (api_->stop(2000, message) != ErrorCode::None) {
    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Servo stream halted: %s", reason);
}

bool JointPositionController::readPoint(
  const trajectory_msgs::msg::JointTrajectory & traj,
  std::vector<double> & position, std::vector<double> & velocity)
{
  const auto & pt = traj.points.back();
  const size_t count = pt.positions.size();

  if (count != axes_.size() || traj.joint_names.size() != count) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Dropped trajectory: %zu positions, %zu names for %zu axes",
      count, traj.joint_names.size(), axes_.size());
    return false;
  }

  const bool hasVelocity = pt.velocities.size() == count;
  position.assign(count, 0.0);
  velocity.assign(count, 0.0);

  for (size_t j = 0; j < count; ++j) {
    const auto it = indexByName_.find(traj.joint_names[j]);
    if (it == indexByName_.end()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dropped trajectory: joint '%s' is not in joint_name", traj.joint_names[j].c_str());
      return false;
    }

    const size_t i = it->second;
    position[i] = pt.positions[j];
    velocity[i] = hasVelocity ? pt.velocities[j] : 0.0;

    if (!std::isfinite(position[i]) || !std::isfinite(velocity[i])) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dropped trajectory: joint '%s' is not finite", traj.joint_names[j].c_str());
      return false;
    }
  }

  return true;
}

// Each Servo point restarts one linear interpolation, aimed at where Servo will be
// `horizon` from now: target = p + v * horizon. Every axis gets the speed that covers
// its distance in `horizon`, so the command velocity is v + (p - posCmd) / horizon:
// Servo's velocity fed forward plus a correction of the remaining gap. The horizon is
// long enough that the profile never starts braking before the next point arrives, so
// consecutive points blend into one continuous motion. With Servo at rest the target
// is p itself and the axes settle there without overshoot.
void JointPositionController::track(
  const std::vector<double> & position, const std::vector<double> & velocity)
{
  const size_t joints = axes_.size();
  const auto now = std::chrono::steady_clock::now();

  // Servo advances its position by one publish_period per point, so the difference of
  // consecutive points is its velocity, already smoothed. A point lost in transit spans
  // two periods. After a pause, fall back to the velocity in the message.
  std::vector<double> v = velocity;
  if (havePrevious_) {
    const double periods = std::round(
      std::chrono::duration<double>(now - previousTime_).count() / publishPeriod_);
    if (periods <= 3.0) {
      const double dt = std::max(periods, 1.0) * publishPeriod_;
      for (size_t i = 0; i < joints; ++i) {
        v[i] = (position[i] - previous_[i]) / dt;
      }
    }
  }
  previous_ = position;
  previousTime_ = now;
  havePrevious_ = true;
  velocity_ = v;
  pending_ = true;
  send();
}

// The engine refuses a new interpolation override while it is still blending the last
// one, which takes longer than Servo's period. The newest point therefore stays
// pending and is resent, extrapolated to the current time, every retry tick until the
// engine accepts it, so the axes switch to fresh data as soon as they can.
void JointPositionController::send()
{
  const size_t joints = axes_.size();
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - previousTime_).count();
  std::vector<double> v = velocity_;

  std::string message;
  std::vector<double> posCmd;
  std::vector<double> velCmd;
  if (api_->getStatus(axes_, posCmd, velCmd, message) != ErrorCode::None) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Dropped point: %s", message.c_str());
    streaming_ = false;
    havePrevious_ = false;
    pending_ = false;
    return;
  }

  double horizon = lookaheadTime_;
  bool moving = false;
  servoMoving_ = false;
  for (size_t i = 0; i < joints; ++i) {
    v[i] = std::clamp(v[i], -maxVelocity_[i], maxVelocity_[i]);
    horizon = std::max(
      horizon, publishPeriod_ + std::fabs(v[i]) / (2.0 * maxAcceleration_[i]));
    moving = moving || std::fabs(velCmd[i]) > kStillVelocity;
    servoMoving_ = servoMoving_ || std::fabs(v[i]) > kStillVelocity;
  }

  std::vector<double> target(joints);
  std::vector<double> speed(joints);
  double largestStep = 0.0;
  for (size_t i = 0; i < joints; ++i) {
    target[i] = previous_[i] + v[i] * (age + horizon);
    const double step = std::fabs(target[i] - posCmd[i]);
    largestStep = std::max(largestStep, step);
    speed[i] = std::clamp(step / horizon, kVelocityFloor, maxVelocity_[i]);
  }

  if (largestStep <= positionTolerance_ && !moving) {
    pending_ = false;
    return;
  }

  const auto sendTime = std::chrono::steady_clock::now();
  if (streaming_ && moving &&
    std::chrono::duration<double>(sendTime - lastAccept_).count() < minOverrideInterval_)
  {
    return;  // keep pending; the retry timer sends it once the interval has passed
  }

  // Only the newest point is ever queued: while the previous override still waits
  // in the channel, keep this one pending and record it once the channel is empty.
  int64_t remaining = 0;
  int64_t errors = 0;
  if (api_->channelStatus(remaining, errors, message) != ErrorCode::None) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", message.c_str());
    return;
  }
  if (errors > bufferErrors_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "API buffer reported %lld failed blocks", errors - bufferErrors_);
    bufferErrors_ = errors;
  }
  if (remaining > 0) {
    ++retries_;
    return;
  }

  const int err = api_->recordOverride(axes_, target, speed, maxAcceleration_, message);
  pending_ = false;
  if (err != ErrorCode::None) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", message.c_str());
    return;
  }
  // Average spacing of accepted overrides, never below Servo's period: the profile
  // must keep moving at least that long before the next one takes over.
  const auto acceptTime = std::chrono::steady_clock::now();
  if (streaming_) {
    const double interval = std::chrono::duration<double>(acceptTime - lastAccept_).count();
    acceptInterval_ = std::max(
      publishPeriod_, 0.8 * acceptInterval_ + 0.2 * std::min(interval, 4.0 * publishPeriod_));
  }
  lastAccept_ = acceptTime;
  ++sent_;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "overrides accepted %zu, busy retries %zu, accepted every %.1f ms",
    sent_, retries_, acceptInterval_ * 1e3);

  if (!streaming_) {
    RCLCPP_INFO(this->get_logger(), "Servo stream started");
  }
  streaming_ = true;
}

// The stream stopped while Servo still had velocity: the last target lies up to one
// horizon ahead. Brake from the current command velocity at max_acceleration instead,
// along the current direction so the axes stay on a straight joint-space line.
void JointPositionController::brake(const char * reason)
{
  streaming_ = false;
  havePrevious_ = false;
  pending_ = false;

  std::string message;
  std::vector<double> posCmd;
  std::vector<double> velCmd;
  if (api_->getStatus(axes_, posCmd, velCmd, message) != ErrorCode::None) {
    RCLCPP_WARN(this->get_logger(), "Brake skipped: %s", message.c_str());
    return;
  }

  double stopTime = 0.0;
  for (size_t i = 0; i < axes_.size(); ++i) {
    stopTime = std::max(stopTime, std::fabs(velCmd[i]) / maxAcceleration_[i]);
  }

  if (stopTime > 0.0) {
    std::vector<double> target(axes_.size());
    std::vector<double> speed(axes_.size());
    for (size_t i = 0; i < axes_.size(); ++i) {
      target[i] = posCmd[i] + 0.5 * velCmd[i] * stopTime;
      speed[i] = std::max(std::fabs(velCmd[i]), kVelocityFloor);
    }
    if (api_->recordOverride(axes_, target, speed, maxAcceleration_, message) !=
      ErrorCode::None)
    {
      RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
    }
  }

  RCLCPP_INFO(this->get_logger(), "Servo stream ended: %s", reason);
}

void JointPositionController::jointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->points.empty() || inExecution_.load()) {
    return;
  }

  std::vector<double> position;
  std::vector<double> velocity;
  if (!readPoint(*msg, position, velocity)) {
    return;
  }

  track(position, velocity);
}

void JointPositionController::watchdogCallback()
{
  if (!streaming_) {
    return;
  }

  const double idle = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - previousTime_).count();
  if (idle <= streamTimeout_) {
    return;
  }

  // With Servo at rest the last command already ends on its final point.
  if (servoMoving_) {
    brake("stream timeout while moving");
    return;
  }
  streaming_ = false;
  havePrevious_ = false;
  pending_ = false;
  RCLCPP_INFO(this->get_logger(), "Servo stream ended: idle");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointPositionController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
