// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "differential_drive_controller.hpp"

#include <thread>

#include <chrono>
#include <cmath>
#include <functional>
#include <vector>

using std::placeholders::_1;

using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionAxisStatus;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::EngineState;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;
using wmx3Api::Velocity;

namespace
{
std::chrono::nanoseconds periodFromRate(int rate)
{
  return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / static_cast<double>(rate)));
}

std::string errorToString(int err)
{
  char errString[256] = {};
  CoreMotion::ErrorToString(err, errString, sizeof(errString));
  return errString;
}
}  // namespace

DifferentialDriveControllerApi::DifferentialDriveControllerApi(
  const rclcpp::Logger & logger, const Config & config)
: logger_(logger), config_(config), cm_(&wmx3Lib_)
{
}

DifferentialDriveControllerApi::~DifferentialDriveControllerApi()
{
  closeDevice();
}

int DifferentialDriveControllerApi::createDevice(std::string & message)
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

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void DifferentialDriveControllerApi::closeDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorToString(err).c_str());
    return;
  }

  RCLCPP_INFO(logger_, "Device closed");
}

int DifferentialDriveControllerApi::getStatus(
  int leftAxis, int rightAxis,
  AxisFeedback & left, AxisFeedback & right, bool & communicating,
  std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  communicating = false;

  if (leftAxis < 0 || leftAxis >= wmx3Api::constants::maxAxes ||
    rightAxis < 0 || rightAxis >= wmx3Api::constants::maxAxes)
  {
    message = "Invalid wheel axes " + std::to_string(leftAxis) + "/" +
      std::to_string(rightAxis) + ": must be in [0, " +
      std::to_string(wmx3Api::constants::maxAxes) + ").";
    return ErrorCode::ArgumentOutOfRange;
  }

  CoreMotionStatus status;
  const int err = cm_.GetStatus(&status);
  if (err != ErrorCode::None) {
    message = "GetStatus failed. Error=" + std::to_string(err) + " (" + errorToString(err) + ")";
    return err;
  }

  const CoreMotionAxisStatus & rawLeft = status.axesStatus[leftAxis];
  const CoreMotionAxisStatus & rawRight = status.axesStatus[rightAxis];

  left = {rawLeft.actualVelocity, rawLeft.servoOn, rawLeft.ampAlarm};
  right = {rawRight.actualVelocity, rawRight.servoOn, rawRight.ampAlarm};
  communicating = status.engineState == EngineState::T::Communicating;

  return ErrorCode::None;
}

int DifferentialDriveControllerApi::startVel(int axis, double omega, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  Velocity::VelCommand velCommand;
  velCommand.axis = axis;
  velCommand.profile.velocity = omega;
  velCommand.profile.type = ProfileType::T::TimeAccTrapezoidal;
  velCommand.profile.accTimeMilliseconds = config_.accTimeMilliseconds;
  velCommand.profile.decTimeMilliseconds = config_.decTimeMilliseconds;

  const int err = cm_.velocity->StartVel(&velCommand);
  if (err != ErrorCode::None) {
    message = "Failed to move motor " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Axis " + std::to_string(axis) + " running at " + std::to_string(omega);
  return ErrorCode::None;
}

DifferentialDriveController::DifferentialDriveController()
: LifecycleNode("differential_drive_controller"),
  prevFeedbackTime_(0, 0, RCL_ROS_TIME),
  lastCmdVelTime_(0, 0, RCL_ROS_TIME)
{
  setRosParameter();

  controlCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  DifferentialDriveControllerApi::Config config;
  config.accTimeMilliseconds = accTime_;
  config.decTimeMilliseconds = decTime_;
  api_ = std::make_unique<DifferentialDriveControllerApi>(this->get_logger(), config);

  RCLCPP_INFO(
    this->get_logger(),
    "differential_drive_controller is unconfigured, waiting for configure...");
}

DifferentialDriveController::~DifferentialDriveController()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "differential_drive_controller stopped");
}

DifferentialDriveController::CallbackReturn DifferentialDriveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring differential_drive_controller...");

  if (!parametersValid()) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid parameters, refusing to configure. Fix the config and retry.");
    return CallbackReturn::FAILURE;
  }

  std::string message;
  if (api_->createDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(this->get_logger(), "differential_drive_controller is configured");
  return CallbackReturn::SUCCESS;
}

DifferentialDriveController::CallbackReturn DifferentialDriveController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  haveFeedbackTime_ = false;
  haveCmdVel_ = false;
  {
    std::lock_guard<std::mutex> lock(driveMutex_);
    driveEnabled_ = true;
    sentOmegaValid_ = false;
    sentOmegaLeft_ = 0.0;
    sentOmegaRight_ = 0.0;
  }

  cmdOmegaPub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    cmdOmegaTopic_, 1);
  encoderOmegaPub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    encoderOmegaTopic_, 1);
  encoderOdometryPub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    encoderOdometryTopic_, 1);
  if (publishTf_) {
    tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  rclcpp::SubscriptionOptions cmdVelOptions;
  cmdVelOptions.callback_group = controlCbGroup_;
  cmdVelStampedSub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    cmdVelTopic_, 1, std::bind(&DifferentialDriveController::cmdStampedCallback, this, _1),
    cmdVelOptions);

  LifecycleNode::on_activate(previous_state);

  controlTimer_ = this->create_wall_timer(
    periodFromRate(rate_), std::bind(&DifferentialDriveController::controlStep, this),
    controlCbGroup_);
  feedbackTimer_ = this->create_wall_timer(
    periodFromRate(rate_), std::bind(&DifferentialDriveController::publishMotorFeedback, this),
    controlCbGroup_);

  RCLCPP_INFO(this->get_logger(), "differential_drive_controller is active");
  return CallbackReturn::SUCCESS;
}

DifferentialDriveController::CallbackReturn DifferentialDriveController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  controlTimer_.reset();
  feedbackTimer_.reset();

  std::lock_guard<std::mutex> lock(driveMutex_);

  driveEnabled_ = false;
  startVel(leftAxis_, 0.0);
  startVel(rightAxis_, 0.0);
  sentOmegaLeft_ = 0.0;
  sentOmegaRight_ = 0.0;
  sentOmegaValid_ = false;

  LifecycleNode::on_deactivate(previous_state);

  cmdVelStampedSub_.reset();
  tfBroadcaster_.reset();
  cmdOmegaPub_.reset();
  encoderOmegaPub_.reset();
  encoderOdometryPub_.reset();

  RCLCPP_INFO(this->get_logger(), "differential_drive_controller is inactive");
  return CallbackReturn::SUCCESS;
}

DifferentialDriveController::CallbackReturn DifferentialDriveController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  api_->closeDevice();

  RCLCPP_INFO(this->get_logger(), "differential_drive_controller is cleaned up");
  return CallbackReturn::SUCCESS;
}

DifferentialDriveController::CallbackReturn DifferentialDriveController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(previous_state);
  }

  return on_cleanup(previous_state);
}

void DifferentialDriveController::cmdStampedCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp, this->get_clock()->get_clock_type());

  lastCmdVel_ = msg->twist;
  lastCmdVelTime_ = (stamp.nanoseconds() > 0) ? stamp : this->get_clock()->now();
  haveCmdVel_ = true;
}

void DifferentialDriveController::controlStep()
{
  DifferentialDriveControllerApi::AxisFeedback left;
  DifferentialDriveControllerApi::AxisFeedback right;
  bool communicating = false;
  std::string message;

  if (api_->getStatus(leftAxis_, rightAxis_, left, right, communicating, message) !=
    ErrorCode::None)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "%s", message.c_str());
    stopWheelsOnFault();
    return;
  }

  if (!communicating) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Communication or engine off. Please start the engine or communication");
    stopWheelsOnFault();
    return;
  }

  if (left.ampAlarm || right.ampAlarm) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Servo alarm on. Please clear servo alarm");
    stopWheelsOnFault();
    return;
  }
  if (!left.servoOn || !right.servoOn) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Servo off. Please set servo on");
    stopWheelsOnFault();
    return;
  }

  const rclcpp::Time now = this->get_clock()->now();
  const bool stale = !haveCmdVel_ || (now - lastCmdVelTime_).seconds() > cmdVelTimeout_;
  const double cmdLinear = stale ? 0.0 : lastCmdVel_.linear.x;
  const double cmdAngular = stale ? 0.0 : lastCmdVel_.angular.z;

  double targetLeft = 0.0;
  double targetRight = 0.0;
  inverseKinematics(cmdLinear, cmdAngular, targetLeft, targetRight);
  commandWheels(targetLeft, targetRight);
}

void DifferentialDriveController::publishMotorFeedback()
{
  std::lock_guard<std::mutex> lock(driveMutex_);

  if (!driveEnabled_) {
    return;
  }

  const rclcpp::Time now = this->get_clock()->now();

  DifferentialDriveControllerApi::AxisFeedback left;
  DifferentialDriveControllerApi::AxisFeedback right;
  bool communicating = false;
  std::string message;

  if (api_->getStatus(leftAxis_, rightAxis_, left, right, communicating, message) !=
    ErrorCode::None || !communicating)
  {
    haveFeedbackTime_ = false;
    return;
  }

  double bodyLinear = 0.0;
  double bodyAngular = 0.0;
  forwardKinematics(left.actualVelocity, right.actualVelocity, bodyLinear, bodyAngular);

  if (haveFeedbackTime_) {
    const double dt = (now - prevFeedbackTime_).seconds();
    if (std::isfinite(dt) && dt > 0.0) {
      odometryPoseCalculation(bodyLinear * dt, bodyAngular * dt);
    }
  }
  prevFeedbackTime_ = now;
  haveFeedbackTime_ = true;

  publishCmdOmega(now);
  publishOmega(now, left.actualVelocity, right.actualVelocity);
  publishOdometry(now, bodyLinear, bodyAngular);
  if (publishTf_) {publishTf(now);}
}

void DifferentialDriveController::commandWheels(double omegaLeft, double omegaRight)
{
  std::lock_guard<std::mutex> lock(driveMutex_);

  if (!driveEnabled_) {
    return;
  }

  if (sentOmegaValid_ && omegaLeft == sentOmegaLeft_ && omegaRight == sentOmegaRight_) {
    return;
  }

  const bool okLeft = startVel(leftAxis_, omegaLeft);
  const bool okRight = startVel(rightAxis_, omegaRight);
  sentOmegaLeft_ = omegaLeft;
  sentOmegaRight_ = omegaRight;
  sentOmegaValid_ = okLeft && okRight;
}

void DifferentialDriveController::stopWheelsOnFault()
{
  std::lock_guard<std::mutex> lock(driveMutex_);

  if (!driveEnabled_) {
    return;
  }

  sentOmegaLeft_ = 0.0;
  sentOmegaRight_ = 0.0;

  if (!sentOmegaValid_) {
    return;
  }

  sentOmegaValid_ = false;
  startVel(leftAxis_, 0.0);
  startVel(rightAxis_, 0.0);
}

bool DifferentialDriveController::startVel(int axis, double omega)
{
  std::string message;
  return api_->startVel(axis, omega, message) == ErrorCode::None;
}

void DifferentialDriveController::inverseKinematics(
  double linear, double angular, double & omegaLeft, double & omegaRight) const
{
  omegaLeft = (2.0 * linear - angular * wheelToWheel_) / (2.0 * wheelRadius_);
  omegaRight = (2.0 * linear + angular * wheelToWheel_) / (2.0 * wheelRadius_);
}

void DifferentialDriveController::forwardKinematics(
  double omegaLeft, double omegaRight, double & linear, double & angular) const
{
  linear = (omegaRight + omegaLeft) * wheelRadius_ / 2.0;
  angular = (omegaRight - omegaLeft) * wheelRadius_ / wheelToWheel_;
}

void DifferentialDriveController::odometryPoseCalculation(double ds, double dtheta)
{
  if (!std::isfinite(ds) || !std::isfinite(dtheta)) {return;}
  const double half = 0.5 * dtheta;
  const double mid = poseTheta_ + half;
  const double k = ds * sinc(half);
  poseX_ += k * std::cos(mid);
  poseY_ += k * std::sin(mid);
  poseTheta_ += dtheta;
}

void DifferentialDriveController::publishCmdOmega(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name = jointName_;
  msg.velocity = {sentOmegaLeft_, sentOmegaRight_};
  cmdOmegaPub_->publish(msg);
}

void DifferentialDriveController::publishOmega(
  const rclcpp::Time & stamp, double omegaLeft, double omegaRight)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name = jointName_;
  msg.velocity = {omegaLeft, omegaRight};
  encoderOmegaPub_->publish(msg);
}

void DifferentialDriveController::publishOdometry(
  const rclcpp::Time & stamp, double linear, double angular)
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = odomFrame_;
  msg.child_frame_id = baseFrame_;

  msg.pose.pose.position.x = poseX_;
  msg.pose.pose.position.y = poseY_;
  msg.pose.pose.orientation = yawToQuaternion(poseTheta_);

  msg.twist.twist.linear.x = linear;
  msg.twist.twist.linear.y = 0.0;
  msg.twist.twist.angular.z = angular;

  constexpr double kSmall = 0.01;
  constexpr double kLarge = 99999.0;
  msg.pose.covariance[0] = kSmall;    // x
  msg.pose.covariance[7] = kSmall;    // y
  msg.pose.covariance[14] = kLarge;   // z
  msg.pose.covariance[21] = kLarge;   // roll
  msg.pose.covariance[28] = kLarge;   // pitch
  msg.pose.covariance[35] = kSmall;   // yaw
  msg.twist.covariance[0] = kSmall;   // vx
  msg.twist.covariance[7] = kSmall;   // vy
  msg.twist.covariance[14] = kLarge;  // vz
  msg.twist.covariance[21] = kLarge;  // v_roll
  msg.twist.covariance[28] = kLarge;  // v_pitch
  msg.twist.covariance[35] = kSmall;  // vyaw

  encoderOdometryPub_->publish(msg);
}

void DifferentialDriveController::publishTf(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = odomFrame_;
  tf.child_frame_id = baseFrame_;
  tf.transform.translation.x = poseX_;
  tf.transform.translation.y = poseY_;
  tf.transform.rotation = yawToQuaternion(poseTheta_);
  tfBroadcaster_->sendTransform(tf);
}

geometry_msgs::msg::Quaternion DifferentialDriveController::yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double DifferentialDriveController::sinc(double a)
{
  if (std::abs(a) < 1e-8) {return 1.0 - a * a / 6.0;}
  return std::sin(a) / a;
}

bool DifferentialDriveController::parametersValid() const
{
  bool valid = true;
  if (rate_ <= 0) {
    RCLCPP_ERROR(this->get_logger(), "rate must be > 0, got %d", rate_);
    valid = false;
  }
  if (wheelRadius_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "wheel_radius must be > 0, got %f", wheelRadius_);
    valid = false;
  }
  if (wheelToWheel_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "wheel_to_wheel must be > 0, got %f", wheelToWheel_);
    valid = false;
  }
  if (jointName_.size() < 2) {
    RCLCPP_ERROR(
      this->get_logger(), "joint_name needs [left, right], got %zu entries",
      jointName_.size());
    valid = false;
  }
  return valid;
}

void DifferentialDriveController::setRosParameter()
{
  leftAxis_ = this->declare_parameter<int>("left_axis", 0);
  rightAxis_ = this->declare_parameter<int>("right_axis", 1);

  rate_ = this->declare_parameter<int>("rate", 100);
  accTime_ = this->declare_parameter<double>("acc_time", 1.0);
  decTime_ = this->declare_parameter<double>("dec_time", 1.0);
  wheelRadius_ = this->declare_parameter<double>("wheel_radius", 0.095);
  wheelToWheel_ = this->declare_parameter<double>("wheel_to_wheel", 0.55);

  cmdVelTimeout_ = this->declare_parameter<double>("cmd_vel_timeout", 0.25);
  publishTf_ = this->declare_parameter<bool>("publish_tf", false);
  odomFrame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  baseFrame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  jointName_ = this->declare_parameter<std::vector<std::string>>(
    "joint_name", std::vector<std::string>{"left_wheel_joint", "right_wheel_joint"});

  cmdVelTopic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_safe");
  cmdOmegaTopic_ = this->declare_parameter<std::string>("cmd_omega_topic", "/omega_cmd");
  encoderOmegaTopic_ = this->declare_parameter<std::string>("encoder_omega_topic", "/omega_enc");
  encoderOdometryTopic_ = this->declare_parameter<std::string>(
    "encoder_odometry_topic", "/odom_enc");

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "left_axis: %d, right_axis: %d", leftAxis_, rightAxis_);
  RCLCPP_INFO(this->get_logger(), "rate: %d", rate_);
  RCLCPP_INFO(this->get_logger(), "acc_time: %f, dec_time: %f", accTime_, decTime_);
  RCLCPP_INFO(this->get_logger(), "wheel_radius: %f", wheelRadius_);
  RCLCPP_INFO(this->get_logger(), "wheel_to_wheel: %f", wheelToWheel_);
  RCLCPP_INFO(this->get_logger(), "cmd_vel_timeout: %f", cmdVelTimeout_);
  RCLCPP_INFO(this->get_logger(), "publish_tf: %s", publishTf_ ? "true" : "false");
  RCLCPP_INFO(
    this->get_logger(), "odom_frame: %s, base_frame: %s",
    odomFrame_.c_str(), baseFrame_.c_str());
  if (jointName_.size() >= 2) {
    RCLCPP_INFO(
      this->get_logger(), "joint_name: [%s, %s]",
      jointName_[0].c_str(), jointName_[1].c_str());
  }
  RCLCPP_INFO(this->get_logger(), "cmd_vel_topic: %s", cmdVelTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "cmd_omega_topic: %s", cmdOmegaTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "encoder_omega_topic: %s", encoderOmegaTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "encoder_odometry_topic: %s", encoderOdometryTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "===========================");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DifferentialDriveController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
