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

namespace ddl = diff_drive;

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

  left = {rawLeft.actualPos, rawLeft.actualVelocity, rawLeft.servoOn, rawLeft.ampAlarm};
  right = {rawRight.actualPos, rawRight.actualVelocity, rawRight.servoOn, rawRight.ampAlarm};
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
  prevLoopTime_(0, 0, RCL_ROS_TIME),
  prevAccelTime_(0, 0, RCL_ROS_TIME),
  lastCmdTime_(0, 0, RCL_ROS_TIME)
{
  setRosParameter();

  model_ = ddl::DiffDriveModel{wheelRadius_, wheelToWheel_};
  accel_ = std::make_unique<ddl::AccelEstimator>(accelAlpha_);

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
  encoderOmegaPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    encoderOmegaTopic_, 1);
  encoderOdometryPub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    encoderOdometryTopic_, 1);
  odomDeltasPub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    odomDeltasTopic_, 1);
  odomAccelPub_ = this->create_publisher<geometry_msgs::msg::AccelStamped>(
    odomAccelTopic_, 1);
  if (publishTf_) {
    tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  cmdVelStampedSub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    cmdVelTopic_, 1, std::bind(&DifferentialDriveController::cmdStampedCallback, this, _1));

  LifecycleNode::on_activate(previous_state);

  havePrev_ = false;
  haveCmd_ = false;
  lastSentValid_ = false;

  controlTimer_ = this->create_wall_timer(
    periodFromRate(rate_), std::bind(&DifferentialDriveController::controlStep, this));

  RCLCPP_INFO(this->get_logger(), "differential_drive_controller is active");
  return CallbackReturn::SUCCESS;
}

DifferentialDriveController::CallbackReturn DifferentialDriveController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  controlTimer_.reset();

  startVel(leftAxis_, 0.0);
  startVel(rightAxis_, 0.0);
  lastSentValid_ = false;

  LifecycleNode::on_deactivate(previous_state);

  cmdVelStampedSub_.reset();
  tfBroadcaster_.reset();
  encoderOmegaPub_.reset();
  encoderOdometryPub_.reset();
  odomDeltasPub_.reset();
  odomAccelPub_.reset();

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
  cmdVelMsg_ = msg->twist;
  lastCmdTime_ = this->get_clock()->now();
  haveCmd_ = true;
}

void DifferentialDriveController::controlStep()
{
  const rclcpp::Time now = this->get_clock()->now();

  DifferentialDriveControllerApi::AxisFeedback left;
  DifferentialDriveControllerApi::AxisFeedback right;
  bool communicating = false;
  std::string message;

  if (api_->getStatus(leftAxis_, rightAxis_, left, right, communicating, message) !=
    ErrorCode::None)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "%s", message.c_str());
    havePrev_ = false;
    stopWheelsOnFault();
    return;
  }

  if (!communicating) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Communication or engine off. Please start the engine or communication");
    havePrev_ = false;
    stopWheelsOnFault();
    return;
  }

  const ddl::WheelOmega enc{left.actualVelocity, right.actualVelocity};
  const ddl::BodyVel body = model_.forward(enc);  // {vx, vyaw}, vy is 0 for diff-drive

  if (havePrev_) {
    const double dt = (now - prevLoopTime_).seconds();
    const double dPhiLeft = left.actualPos - prevPosLeft_;
    const double dPhiRight = right.actualPos - prevPosRight_;
    const bool finiteDt = std::isfinite(dt) && dt > 0.0;
    const bool jumped = finiteDt &&
      (std::abs(dPhiLeft - left.actualVelocity * dt) > jumpGuardTol_ ||
      std::abs(dPhiRight - right.actualVelocity * dt) > jumpGuardTol_);
    if (jumped) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Encoder position jump (homing/rollover?) -- re-baselining odometry this cycle");
    } else {
      const ddl::BodyVel d = model_.forwardDelta(dPhiLeft, dPhiRight);  // {ds, dtheta}
      integrator_.odometryPoseCalculation(d.linear, d.angular);
      deltas_.odometryDeltaAccumulation(d.linear, d.angular);
    }
  }
  prevPosLeft_ = left.actualPos;
  prevPosRight_ = right.actualPos;
  prevLoopTime_ = now;
  havePrev_ = true;

  publishOmega(enc);
  publishOdometry(now, body);
  publishDeltas(now);
  publishAccel(now, body);
  if (publishTf_) {publishTf(now);}

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

  const bool stale = !haveCmd_ || (now - lastCmdTime_).seconds() > cmdVelTimeout_;
  const ddl::BodyVel cmd =
    stale ? ddl::BodyVel{0.0, 0.0} : ddl::BodyVel{cmdVelMsg_.linear.x, cmdVelMsg_.angular.z};

  const ddl::WheelOmega target = model_.inverse(cmd);
  commandWheels(target.left, target.right);
}

void DifferentialDriveController::commandWheels(double omegaLeft, double omegaRight)
{
  if (lastSentValid_ && omegaLeft == lastSentLeft_ && omegaRight == lastSentRight_) {
    return;
  }

  const bool okLeft = startVel(leftAxis_, omegaLeft);
  const bool okRight = startVel(rightAxis_, omegaRight);
  if (okLeft && okRight) {
    lastSentLeft_ = omegaLeft;
    lastSentRight_ = omegaRight;
    lastSentValid_ = true;
  } else {
    lastSentValid_ = false;
  }
}

void DifferentialDriveController::stopWheelsOnFault()
{
  if (!lastSentValid_) {
    return;
  }

  lastSentValid_ = false;
  startVel(leftAxis_, 0.0);
  startVel(rightAxis_, 0.0);
}

bool DifferentialDriveController::startVel(int axis, double omega)
{
  std::string message;
  return api_->startVel(axis, omega, message) == ErrorCode::None;
}

void DifferentialDriveController::publishOmega(const ddl::WheelOmega & enc)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {enc.left, enc.right};
  encoderOmegaPub_->publish(msg);
}

void DifferentialDriveController::publishOdometry(
  const rclcpp::Time & stamp, const ddl::BodyVel & body)
{
  const ddl::Pose2D & pose = integrator_.pose();

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = odomFrame_;
  msg.child_frame_id = baseFrame_;

  msg.pose.pose.position.x = pose.x;
  msg.pose.pose.position.y = pose.y;
  msg.pose.pose.orientation = yawToQuaternion(pose.theta);

  msg.twist.twist.linear.x = body.linear;
  msg.twist.twist.linear.y = 0.0;
  msg.twist.twist.angular.z = body.angular;

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

void DifferentialDriveController::publishDeltas(const rclcpp::Time & stamp)
{
  const ddl::OdomDelta delta = deltas_.take();
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = odomFrame_;
  msg.twist.linear.x = delta.linear;
  msg.twist.angular.z = delta.angular;
  odomDeltasPub_->publish(msg);
}

void DifferentialDriveController::publishAccel(
  const rclcpp::Time & stamp, const ddl::BodyVel & body)
{
  if (!haveAccelClock_) {
    prevAccelTime_ = stamp;
    haveAccelClock_ = true;
    return;
  }
  const double dtAccel = (stamp - prevAccelTime_).seconds();
  const double targetDt = (accelPublishRate_ > 0.0) ? 1.0 / accelPublishRate_ : 0.0;
  if (dtAccel < targetDt) {
    return;
  }

  const ddl::BodyAccel accel = accel_->update(body, dtAccel);
  geometry_msgs::msg::AccelStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = baseFrame_;
  msg.accel.linear.x = accel.linear;
  msg.accel.angular.z = accel.angular;
  odomAccelPub_->publish(msg);

  prevAccelTime_ = stamp;
}

void DifferentialDriveController::publishTf(const rclcpp::Time & stamp)
{
  const ddl::Pose2D & pose = integrator_.pose();
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = odomFrame_;
  tf.child_frame_id = baseFrame_;
  tf.transform.translation.x = pose.x;
  tf.transform.translation.y = pose.y;
  tf.transform.rotation = yawToQuaternion(pose.theta);
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
  accelPublishRate_ = this->declare_parameter<double>("accel_publish_rate", 10.0);
  accelAlpha_ = this->declare_parameter<double>("accel_alpha", 0.3);
  publishTf_ = this->declare_parameter<bool>("publish_tf", false);
  odomFrame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  baseFrame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  jumpGuardTol_ = this->declare_parameter<double>("jump_guard_tol", 0.5);

  cmdVelTopic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_safe");
  encoderOmegaTopic_ = this->declare_parameter<std::string>("encoder_omega_topic", "/omega_enc");
  encoderOdometryTopic_ = this->declare_parameter<std::string>(
    "encoder_odometry_topic", "/odom_enc");
  odomDeltasTopic_ = this->declare_parameter<std::string>("odom_deltas_topic", "/odom_deltas");
  odomAccelTopic_ = this->declare_parameter<std::string>("odom_accel_topic", "/odom_accel");

  if (rate_ <= 0) {
    RCLCPP_WARN(this->get_logger(), "rate must be > 0; falling back to 100 Hz");
    rate_ = 100;
  }
  if (wheelRadius_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "wheel_radius must be > 0; falling back to 0.095");
    wheelRadius_ = 0.095;
  }
  if (wheelToWheel_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "wheel_to_wheel must be > 0; falling back to 0.55");
    wheelToWheel_ = 0.55;
  }
  if (accelPublishRate_ < 0.0) {
    RCLCPP_WARN(this->get_logger(), "accel_publish_rate must be >= 0; falling back to 10.0");
    accelPublishRate_ = 10.0;
  }
  if (!(jumpGuardTol_ > 0.0)) {
    RCLCPP_WARN(this->get_logger(), "jump_guard_tol must be > 0; falling back to 0.5");
    jumpGuardTol_ = 0.5;
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "left_axis: %d, right_axis: %d", leftAxis_, rightAxis_);
  RCLCPP_INFO(this->get_logger(), "rate: %d", rate_);
  RCLCPP_INFO(this->get_logger(), "acc_time: %f, dec_time: %f", accTime_, decTime_);
  RCLCPP_INFO(this->get_logger(), "wheel_radius: %f", wheelRadius_);
  RCLCPP_INFO(this->get_logger(), "wheel_to_wheel: %f", wheelToWheel_);
  RCLCPP_INFO(this->get_logger(), "cmd_vel_timeout: %f", cmdVelTimeout_);
  RCLCPP_INFO(
    this->get_logger(), "accel_publish_rate: %f, accel_alpha: %f",
    accelPublishRate_, accelAlpha_);
  RCLCPP_INFO(this->get_logger(), "publish_tf: %s", publishTf_ ? "true" : "false");
  RCLCPP_INFO(
    this->get_logger(), "odom_frame: %s, base_frame: %s",
    odomFrame_.c_str(), baseFrame_.c_str());
  RCLCPP_INFO(this->get_logger(), "jump_guard_tol: %f", jumpGuardTol_);
  RCLCPP_INFO(this->get_logger(), "cmd_vel_topic: %s", cmdVelTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "encoder_omega_topic: %s", encoderOmegaTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "encoder_odometry_topic: %s", encoderOdometryTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "odom_deltas_topic: %s", odomDeltasTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "odom_accel_topic: %s", odomAccelTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "===========================");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DifferentialDriveController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
