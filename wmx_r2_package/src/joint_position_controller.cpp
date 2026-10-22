// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

using std::placeholders::_1;
using wmx3Api::AxisSelection;
using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::Motion;
using wmx3Api::ProfileType;
using wmx3Api::WMX3Api;

class JointPositionController : public rclcpp::Node
{
public:
  JointPositionController();
  ~JointPositionController();

private:
  bool initialized_ = false;
  std::atomic<bool> initializing_{false};
  std::atomic<bool> in_execution_{false};

  WMX3Api wmx3Lib_;
  std::unique_ptr<CoreMotion> wmx3LibCm_;
  CoreMotionStatus cmStatus_;
  Motion::LinearIntplCommand intpl_;
  AxisSelection axisSel_;

  int err_ = 0;
  char errString_[256];

  std::vector<int64_t> jointAxes_;
  std::vector<std::string> jointNames_;
  std::string jointTrajectoryTopic_;
  double accelRatio_ = 0.0;
  double defaultVelocity_ = 0.0;
  double minStep_ = 0.0;
  double maxJointVelocity_ = 3.0;       // rad/s ceiling, applied as a uniform scale
  double maxJointAcceleration_ = 0.0;   // rad/s^2 ceiling, same scale; 0 disables

  std::map<std::string, int> axisByName_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execActiveSub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr jointTrajectorySub_;

  std::thread init_thread_;

  void setRosParameter();
  void onEngineReady(const std_msgs::msg::Bool::SharedPtr msg);
  void onExecActive(const std_msgs::msg::Bool::SharedPtr msg);
  void runInitSequence();
  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  bool buildCommand(const trajectory_msgs::msg::JointTrajectory & traj);
  void motionErrorToString(int code, char * buf, size_t size);
};

JointPositionController::JointPositionController()
: Node("joint_position_controller")
{
  RCLCPP_INFO(this->get_logger(), "start joint_position_controller");

  setRosParameter();

  auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
  engineReadySub_ = this->create_subscription<std_msgs::msg::Bool>(
    "wmx/engine/ready", ready_qos,
    std::bind(&JointPositionController::onEngineReady, this, _1));

  execActiveSub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local(),
    std::bind(&JointPositionController::onExecActive, this, _1));

  RCLCPP_INFO(this->get_logger(), "joint_position_controller waiting for engine...");
}

JointPositionController::~JointPositionController()
{
  RCLCPP_INFO(this->get_logger(), "Stop joint_position_controller");

  if (init_thread_.joinable()) {
    init_thread_.join();
  }

  if (initialized_) {
    wmx3LibCm_->motion->Stop(&axisSel_);
    wmx3LibCm_->motion->Wait(&axisSel_);

    err_ = wmx3Lib_.CloseDevice();
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(this->get_logger(), "Failed to close device. Error=%d (%s)", err_, errString_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Device closed");
    }
  }

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is stopped");
}

void JointPositionController::setRosParameter()
{
  this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  this->declare_parameter<std::vector<std::string>>(
    "joint_name", {"j1", "j2", "j3", "j4", "j5", "j6"});
  this->declare_parameter<std::string>(
    "joint_trajectory_topic", "/joint_trajectory_topic/no_param");
  this->declare_parameter<double>("accel_ratio", 0.5);
  this->declare_parameter<double>("default_velocity", 0.1);
  this->declare_parameter<double>("min_step", 0.1);
  this->declare_parameter<double>("max_joint_velocity", 3.0);
  this->declare_parameter<double>("max_joint_acceleration", 0.0);

  this->get_parameter("joint_axes", jointAxes_);
  this->get_parameter("joint_name", jointNames_);
  this->get_parameter("joint_trajectory_topic", jointTrajectoryTopic_);
  this->get_parameter("accel_ratio", accelRatio_);
  this->get_parameter("default_velocity", defaultVelocity_);
  this->get_parameter("min_step", minStep_);
  this->get_parameter("max_joint_velocity", maxJointVelocity_);
  this->get_parameter("max_joint_acceleration", maxJointAcceleration_);

  if (maxJointVelocity_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "max_joint_velocity must be > 0 (got %.3f); using 3.0", maxJointVelocity_);
    maxJointVelocity_ = 3.0;
  }
  if (maxJointAcceleration_ < 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "max_joint_acceleration must be >= 0 (got %.3f); disabling the acceleration clamp",
      maxJointAcceleration_);
    maxJointAcceleration_ = 0.0;
  }

  for (size_t i = 0; i < jointNames_.size() && i < jointAxes_.size(); ++i) {
    axisByName_[jointNames_[i]] = static_cast<int>(jointAxes_[i]);
  }

  axisSel_.axisCount = jointAxes_.size();
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    axisSel_.axis[i] = static_cast<int>(jointAxes_[i]);
  }

  std::string joint_axes_str;
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    if (i > 0) {joint_axes_str += ", ";}
    joint_axes_str += std::to_string(jointAxes_[i]);
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", joint_axes_str.c_str());
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_topic: %s", jointTrajectoryTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "accel_ratio: %f", accelRatio_);
  RCLCPP_INFO(this->get_logger(), "default_velocity: %f", defaultVelocity_);
  RCLCPP_INFO(this->get_logger(), "min_step: %f", minStep_);
  RCLCPP_INFO(this->get_logger(), "max_joint_velocity: %.3f rad/s", maxJointVelocity_);
  if (maxJointAcceleration_ > 0.0) {
    RCLCPP_INFO(
      this->get_logger(), "max_joint_acceleration: %.1f rad/s^2", maxJointAcceleration_);
  } else {
    RCLCPP_INFO(this->get_logger(), "max_joint_acceleration: disabled");
  }
  RCLCPP_INFO(this->get_logger(), "===========================");
}

void JointPositionController::onEngineReady(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data || initialized_ || initializing_.exchange(true)) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Engine ready — starting init on dedicated thread...");

  // Join any previous thread (e.g. from a failed retry)
  if (init_thread_.joinable()) {
    init_thread_.join();
  }

  // Spawn dedicated thread so the blocking retry loop doesn't block executor
  init_thread_ = std::thread(&JointPositionController::runInitSequence, this);
}

void JointPositionController::onExecActive(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (in_execution_.exchange(msg->data) == msg->data) {
    return;
  }
  RCLCPP_INFO(
    this->get_logger(), "Servo commands %s (move_group execution %s)",
    msg->data ? "blocked" : "allowed", msg->data ? "started" : "finished");
}

void JointPositionController::runInitSequence()
{
  unsigned int timeout = 10000;
  static constexpr int kMaxDeviceRetries = 30;

  for (int attempt = 1; attempt <= kMaxDeviceRetries; ++attempt) {
    err_ = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout);
    if (err_ == ErrorCode::None) {
      break;
    }
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    if (err_ == ErrorCode::StartProcessLockError) {
      RCLCPP_WARN(
        this->get_logger(), "Device lock busy, retrying in 1s... (%d/%d)",
        attempt, kMaxDeviceRetries);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to attach to device. Error=%d (%s)", err_,
        errString_);
      initializing_ = false;
      return;
    }
  }

  if (err_ != ErrorCode::None) {
    RCLCPP_FATAL(
      this->get_logger(), "Device lock busy after %d retries, giving up", kMaxDeviceRetries);
    initializing_ = false;
    return;
  }

  wmx3Lib_.SetDeviceName("joint_position_controller");

  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");

  wmx3LibCm_ = std::make_unique<CoreMotion>(&wmx3Lib_);

  jointTrajectorySub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    jointTrajectoryTopic_, 1,
    std::bind(&JointPositionController::jointTrajectoryCallback, this, _1));

  initialized_ = true;
  engineReadySub_.reset();

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is ready");
}

// A motion error comes from a CoreMotion call (GetStatus, StartLinearIntplPos),
// so it carries a CoreMotion error code. Decoding it with the device-level
// WMX3Api::ErrorToString yields an empty string for those codes, which is why
// rejected setpoints used to log as "()". Try CoreMotion first, fall back to
// the device decoder, then admit defeat rather than printing empty brackets.
void JointPositionController::motionErrorToString(int code, char * buf, size_t size)
{
  const auto n = static_cast<unsigned int>(size);
  buf[0] = '\0';
  CoreMotion::ErrorToString(code, buf, n);
  if (buf[0] == '\0') {
    wmx3Lib_.ErrorToString(code, buf, n);
  }
  if (buf[0] == '\0') {
    snprintf(buf, size, "unrecognised code");
  }
}

bool JointPositionController::buildCommand(const trajectory_msgs::msg::JointTrajectory & traj)
{
  const auto & pt = traj.points.back();
  const size_t count = pt.positions.size();

  if (count != jointAxes_.size()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Dropped trajectory: %zu positions for %zu axes", count, jointAxes_.size());
    return false;
  }

  const double dt = rclcpp::Duration(pt.time_from_start).seconds();

  err_ = wmx3LibCm_->GetStatus(&cmStatus_);
  if (err_ != ErrorCode::None) {
    motionErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "CoreMotion::GetStatus failed, cannot place segment origin. Error=%d (%s)",
      err_, errString_);
    return false;
  }

  intpl_.axisCount = count;
  double largestStep = 0.0;
  double peakVelocity = 0.0;
  double peakAccel = 0.0;

  for (size_t i = 0; i < count; ++i) {
    int axis;
    if (traj.joint_names.empty()) {
      axis = static_cast<int>(jointAxes_[i]);
    } else {
      const auto it = axisByName_.find(traj.joint_names[i]);
      if (it == axisByName_.end()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Dropped trajectory: joint '%s' is not in joint_name", traj.joint_names[i].c_str());
        return false;
      }
      axis = it->second;
    }

    const double step = std::fabs(pt.positions[i] - cmStatus_.axesStatus[axis].posCmd);
    largestStep = std::fmax(largestStep, step);

    const double velocity = (dt > 0.0) ? step / dt : defaultVelocity_;
    const double accel = velocity / (accelRatio_ * ((dt > 0.0) ? dt : 1.0));
    peakVelocity = std::fmax(peakVelocity, velocity);
    peakAccel = std::fmax(peakAccel, accel);

    intpl_.axis[i] = axis;
    intpl_.target[i] = pt.positions[i];
    intpl_.maxVelocity[i] = velocity;
    intpl_.maxAcc[i] = accel;
    intpl_.maxDec[i] = accel;
  }

  if (largestStep < minStep_) {
    return false;
  }

  // Velocity is step / dt with nothing above it, so one setpoint far from the
  // current commanded position (or one with a tiny time_from_start) asks for a
  // velocity and acceleration the drive cannot deliver: the engine spends
  // seconds blending the oversized move, rejects every later setpoint with
  // ProcessingOverride, and the amplifier alarms. Scale the whole move down
  // uniformly rather than clamping axes one by one: clamping one axis while the
  // others run free would bend the coordinated path, whereas slowing every
  // axis by the same factor keeps it straight and only costs time (the arm
  // lags the stream). Acceleration is velocity / (accelRatio * dt), so the one
  // scale factor bounds both.
  double scale = 1.0;
  if (peakVelocity > maxJointVelocity_) {
    scale = std::fmax(scale, peakVelocity / maxJointVelocity_);
  }
  if (maxJointAcceleration_ > 0.0 && peakAccel > maxJointAcceleration_) {
    scale = std::fmax(scale, peakAccel / maxJointAcceleration_);
  }
  if (scale > 1.0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Setpoint needs %.3f rad/s and %.1f rad/s^2 (limits %.3f, %.1f; 0 = off); "
      "scaling the move by %.2f (the arm will lag the stream)",
      peakVelocity, peakAccel, maxJointVelocity_, maxJointAcceleration_, 1.0 / scale);
    for (size_t i = 0; i < count; ++i) {
      intpl_.maxVelocity[i] /= scale;
      intpl_.maxAcc[i] /= scale;
      intpl_.maxDec[i] /= scale;
    }
  }

  intpl_.profile.type = ProfileType::Trapezoidal;
  intpl_.profile.velocity = 0.0;
  intpl_.profile.acc = 0.0;
  intpl_.profile.dec = 0.0;

  return true;
}

void JointPositionController::jointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->points.empty() || in_execution_.load() || !buildCommand(*msg)) {
    return;
  }

  err_ = wmx3LibCm_->motion->StartLinearIntplPos(&intpl_);
  if (err_ != ErrorCode::None) {
    motionErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "StartLinearIntplPos failed. Error=%d (%s)", err_, errString_);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointPositionController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
