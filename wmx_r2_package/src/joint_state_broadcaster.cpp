// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "joint_state_broadcaster.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include "wmx_qos_compat.hpp"

using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionAxisStatus;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::IO;

namespace
{
std::chrono::nanoseconds periodFromRate(int rate)
{
  return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / static_cast<double>(rate)));
}

constexpr int kServiceMaxRetries = 5;
constexpr std::chrono::seconds kServiceWaitTimeout{10};
constexpr std::chrono::seconds kServiceCallTimeout{15};
constexpr std::chrono::seconds kServiceRetryDelay{2};

std::string errorToString(int err)
{
  char errString[256] = {};
  CoreMotion::ErrorToString(err, errString, sizeof(errString));
  return errString;
}

}  // namespace

JointStateBroadcasterApi::JointStateBroadcasterApi(const rclcpp::Logger & logger)
: logger_(logger), cm_(&wmx3Lib_), io_(&wmx3Lib_)
{
}

JointStateBroadcasterApi::~JointStateBroadcasterApi()
{
  closeDevice();
}

int JointStateBroadcasterApi::createDevice(std::string & message)
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
  io_ = IO(&wmx3Lib_);

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void JointStateBroadcasterApi::closeDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorToString(err).c_str());
    return;
  }

  RCLCPP_INFO(logger_, "Device closed");
}

int JointStateBroadcasterApi::getStatus(
  const std::vector<int64_t> & axes, std::vector<AxisFeedback> & feedback,
  std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  feedback.clear();

  CoreMotionStatus status;
  const int err = cm_.GetStatus(&status);
  if (err != ErrorCode::None) {
    message = "GetStatus failed. Error=" + std::to_string(err) + " (" + errorToString(err) + ")";
    return err;
  }

  feedback.reserve(axes.size());
  for (const int64_t axis : axes) {
    if (axis < 0 || axis >= wmx3Api::constants::maxAxes) {
      message = "Invalid axis " + std::to_string(axis) + ": must be in [0, " +
        std::to_string(wmx3Api::constants::maxAxes) + ").";
      feedback.clear();
      return ErrorCode::ArgumentOutOfRange;
    }
    const CoreMotionAxisStatus & raw = status.axesStatus[axis];
    feedback.push_back({raw.actualPos, raw.actualVelocity});
  }

  return ErrorCode::None;
}

int JointStateBroadcasterApi::getOutBit(
  int32_t addr, int32_t bit, uint8_t & data, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = io_.GetOutBit(addr, bit, &data);
  if (err != ErrorCode::None) {
    message = "GetOutBit failed. addr=" + std::to_string(addr) + " bit=" + std::to_string(bit) +
      ". Error=" + std::to_string(err) + " (" + errorToString(err) + ")";
    return err;
  }

  return ErrorCode::None;
}

int JointStateBroadcasterApi::setServoOn(int axis, int newStatus, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = cm_.axisControl->SetServoOn(axis, newStatus);
  if (err != ErrorCode::None) {
    message = "Servo " + std::to_string(axis) + " error to " + (newStatus ? "on" : "off") +
      ". Error=" + std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Servo " + std::to_string(axis) + " " + (newStatus ? "on" : "off");
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

JointStateBroadcaster::JointStateBroadcaster()
: LifecycleNode("joint_state_broadcaster")
{
  setRosParameter();

  api_ = std::make_unique<JointStateBroadcasterApi>(this->get_logger());

  axisClientCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  clearAlarmClient_ = this->create_client<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/clear_amp_alarm", servicesQos(), axisClientCbGroup_);

  setAxisOnClient_ = this->create_client<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/set_servo_on", servicesQos(), axisClientCbGroup_);

  getAxisParamClient_ = this->create_client<wmx_r2_message::srv::GetAxisParam>(
    "wmx/engine/get_axis_param", servicesQos(), axisClientCbGroup_);

  RCLCPP_INFO(
    this->get_logger(), "joint_state_broadcaster is unconfigured, waiting for configure...");
}

JointStateBroadcaster::~JointStateBroadcaster()
{
  servoOff();
  api_.reset();

  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster stopped");
}

void JointStateBroadcaster::setRosParameter()
{
  jointAxes_ = this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  jointFeedbackRate_ = this->declare_parameter<int>("joint_feedback_rate", 0);
  gripperOpenValue_ = this->declare_parameter<float>("gripper_open_value", 0);
  gripperCloseValue_ = this->declare_parameter<float>("gripper_close_value", 0);
  jointNames_ = this->declare_parameter<std::vector<std::string>>(
    "joint_name", {"j1", "j2", "j3", "j4", "j5", "j6"});
  gripperJointNames_ = this->declare_parameter<std::vector<std::string>>(
    "gripper_joint_name", std::vector<std::string>{});
  gripperAddress_ = this->declare_parameter<std::vector<int64_t>>(
    "gripper_address", std::vector<int64_t>{0, 0});
  encoderJointTopic_ = this->declare_parameter<std::string>(
    "encoder_joint_topic", "/encoder_joint_topic/no_param");
  isaacsimJointTopic_ = this->declare_parameter<std::string>(
    "isaacsim_joint_topic", "/isaacsim_joint_topic/no_param");
  gazeboPositionTopic_ = this->declare_parameter<std::string>(
    "gazebo_position_joint_topic", "");
  gazeboVelocityTopic_ = this->declare_parameter<std::string>(
    "gazebo_velocity_joint_topic", "");
  gazeboPositionAxes_ = this->declare_parameter<std::vector<int64_t>>(
    "gazebo_position_joint_axes", std::vector<int64_t>{});
  gazeboVelocityAxes_ = this->declare_parameter<std::vector<int64_t>>(
    "gazebo_velocity_joint_axes", std::vector<int64_t>{});

  if (jointFeedbackRate_ <= 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "joint_feedback_rate must be > 0, got %d. Falling back to 100 Hz.", jointFeedbackRate_);
    jointFeedbackRate_ = 100;
  }

  if (jointNames_.size() < jointAxes_.size()) {
    RCLCPP_WARN(
      this->get_logger(),
      "joint_name has %zu entries for %zu axes; dropping the unnamed axes.",
      jointNames_.size(), jointAxes_.size());
    jointAxes_.resize(jointNames_.size());
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_feedback_rate: %d", jointFeedbackRate_);

  std::string jointNamesText;
  for (size_t i = 0; i < jointNames_.size(); ++i) {
    if (i > 0) {jointNamesText += ", ";}
    jointNamesText += jointNames_[i];
  }
  RCLCPP_INFO(this->get_logger(), "joint_name: [%s]", jointNamesText.c_str());

  std::string jointAxesText;
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    if (i > 0) {jointAxesText += ", ";}
    jointAxesText += std::to_string(jointAxes_[i]);
  }
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", jointAxesText.c_str());

  if (!gripperJointNames_.empty()) {
    std::string gripperJointNamesText;
    for (size_t i = 0; i < gripperJointNames_.size(); ++i) {
      if (i > 0) {gripperJointNamesText += ", ";}
      gripperJointNamesText += gripperJointNames_[i];
    }
    RCLCPP_INFO(this->get_logger(), "gripper_joint_name: [%s]", gripperJointNamesText.c_str());
    RCLCPP_INFO(this->get_logger(), "gripper_open_value: %f", gripperOpenValue_);
    RCLCPP_INFO(this->get_logger(), "gripper_close_value: %f", gripperCloseValue_);
    if (gripperAddress_.size() >= 2) {
      RCLCPP_INFO(
        this->get_logger(), "gripper_address: [%ld, %ld]",
        gripperAddress_[0], gripperAddress_[1]);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "gripper_address needs [byte, bit], got %zu entries; the gripper joints "
        "will not be published.", gripperAddress_.size());
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "gripper_joint_name: [] (no gripper)");
  }
  RCLCPP_INFO(this->get_logger(), "encoder_joint_topic: %s", encoderJointTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "isaacsim_joint_topic: %s", isaacsimJointTopic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "gazebo_position_joint_topic: %s",
    gazeboPositionTopic_.empty() ? "(disabled)" : gazeboPositionTopic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "gazebo_velocity_joint_topic: %s",
    gazeboVelocityTopic_.empty() ? "(disabled)" : gazeboVelocityTopic_.c_str());
  if (gazeboPositionTopic_.empty() != gazeboPositionAxes_.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "gazebo_position_joint_topic and gazebo_position_joint_axes must both be set; "
      "the position publisher stays off.");
  }
  if (gazeboVelocityTopic_.empty() != gazeboVelocityAxes_.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "gazebo_velocity_joint_topic and gazebo_velocity_joint_axes must both be set; "
      "the velocity publisher stays off.");
  }
  RCLCPP_INFO(this->get_logger(), "===========================");
}

void JointStateBroadcaster::servoOff()
{
  for (const int64_t axis : jointAxes_) {
    std::string message;
    api_->setServoOn(static_cast<int>(axis), 0, message);
  }
}

JointStateBroadcaster::CallbackReturn JointStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring joint_state_broadcaster...");

  std::string message;
  if (api_->createDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  if (getAxisParam(message)) {
    RCLCPP_INFO(this->get_logger(), "WMX parameters read: %s", message.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "Could not read WMX parameters: %s", message.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster is configured");
  return CallbackReturn::SUCCESS;
}

JointStateBroadcaster::CallbackReturn JointStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  std::vector<int64_t> zeroData(jointAxes_.size(), 0);
  std::vector<int64_t> onData(jointAxes_.size(), 1);

  if (!callSetAxesService(clearAlarmClient_, "wmx/axes/clear_amp_alarm", jointAxes_, zeroData)) {
    RCLCPP_ERROR(this->get_logger(), "Activation failed at clear_alarm");
    return CallbackReturn::FAILURE;
  }

  if (!callSetAxesService(setAxisOnClient_, "wmx/axes/set_servo_on", jointAxes_, onData)) {
    RCLCPP_ERROR(this->get_logger(), "Activation failed at set_on");
    return CallbackReturn::FAILURE;
  }

  encoderJointPub_ = this->create_publisher<sensor_msgs::msg::JointState>(encoderJointTopic_, 1);
  isaacsimJointPub_ = this->create_publisher<sensor_msgs::msg::JointState>(isaacsimJointTopic_, 1);
  if (!gazeboPositionTopic_.empty() && !gazeboPositionAxes_.empty()) {
    gazeboPositionPub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(gazeboPositionTopic_, 1);
  }
  if (!gazeboVelocityTopic_.empty() && !gazeboVelocityAxes_.empty()) {
    gazeboVelocityPub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(gazeboVelocityTopic_, 1);
  }

  LifecycleNode::on_activate(previous_state);

  encoderJointTimer_ = this->create_wall_timer(
    periodFromRate(jointFeedbackRate_),
    std::bind(&JointStateBroadcaster::publishJointState, this));

  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster is active");
  return CallbackReturn::SUCCESS;
}

JointStateBroadcaster::CallbackReturn JointStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  encoderJointTimer_.reset();
  servoOff();

  LifecycleNode::on_deactivate(previous_state);

  encoderJointPub_.reset();
  isaacsimJointPub_.reset();
  gazeboPositionPub_.reset();
  gazeboVelocityPub_.reset();

  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster is inactive");
  return CallbackReturn::SUCCESS;
}

JointStateBroadcaster::CallbackReturn JointStateBroadcaster::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  api_->closeDevice();

  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster is cleaned up");
  return CallbackReturn::SUCCESS;
}

JointStateBroadcaster::CallbackReturn JointStateBroadcaster::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(previous_state);
  }

  return on_cleanup(previous_state);
}

bool JointStateBroadcaster::callSetAxesService(
  rclcpp::Client<wmx_r2_message::srv::SetAxes>::SharedPtr client,
  const std::string & serviceName,
  const std::vector<int64_t> & axes,
  const std::vector<int64_t> & data)
{
  for (int attempt = 1; attempt <= kServiceMaxRetries; attempt++) {
    if (!client->wait_for_service(kServiceWaitTimeout)) {
      RCLCPP_WARN(
        this->get_logger(), "Service %s not available yet (attempt %d/%d)",
        serviceName.c_str(), attempt, kServiceMaxRetries);
      continue;
    }

    auto request = std::make_shared<wmx_r2_message::srv::SetAxes::Request>();
    request->axis.assign(axes.begin(), axes.end());
    request->data.assign(data.begin(), data.end());

    RCLCPP_INFO(
      this->get_logger(), "Calling %s (attempt %d/%d)",
      serviceName.c_str(), attempt, kServiceMaxRetries);

    auto future = client->async_send_request(request);
    if (future.wait_for(kServiceCallTimeout) != std::future_status::ready) {
      client->remove_pending_request(future);
      RCLCPP_WARN(
        this->get_logger(), "Service call %s timed out (attempt %d/%d)",
        serviceName.c_str(), attempt, kServiceMaxRetries);
      std::this_thread::sleep_for(kServiceRetryDelay);
      continue;
    }

    auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(
        this->get_logger(), "%s failed: %s",
        serviceName.c_str(), response->message.c_str());
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(), "%s succeeded: %s",
      serviceName.c_str(), response->message.c_str());
    return true;
  }

  RCLCPP_ERROR(
    this->get_logger(), "Service call %s failed after %d attempts",
    serviceName.c_str(), kServiceMaxRetries);
  return false;
}

bool JointStateBroadcaster::getAxisParam(std::string & message)
{
  if (!getAxisParamClient_->wait_for_service(kServiceWaitTimeout)) {
    message = std::string(getAxisParamClient_->get_service_name()) + " is not available";
    return false;
  }

  auto request = std::make_shared<wmx_r2_message::srv::GetAxisParam::Request>();
  request->axis.assign(jointAxes_.begin(), jointAxes_.end());

  auto future = getAxisParamClient_->async_send_request(request);
  if (future.wait_for(kServiceCallTimeout) != std::future_status::ready) {
    getAxisParamClient_->remove_pending_request(future);
    message = std::string(getAxisParamClient_->get_service_name()) + " timed out";
    return false;
  }

  const auto response = future.get();
  message = response->message;

  for (const std::string & line : response->axis_param) {
    RCLCPP_INFO(this->get_logger(), "%s", line.c_str());
  }

  return response->success;
}

void JointStateBroadcaster::publishJointState()
{
  std::vector<JointStateBroadcasterApi::AxisFeedback> feedback;
  std::string message;

  if (api_->getStatus(jointAxes_, feedback, message) != ErrorCode::None) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Joint state not published: %s", message.c_str());
    return;
  }

  sensor_msgs::msg::JointState encoderJointMsg;
  for (size_t i = 0; i < feedback.size(); ++i) {
    encoderJointMsg.name.push_back(jointNames_[i]);
    encoderJointMsg.position.push_back(feedback[i].actualPos);
    encoderJointMsg.velocity.push_back(feedback[i].actualVelocity);
  }

  if (gripperAddress_.size() >= 2) {
    uint8_t gripperData = 0;
    const bool gripperRead = api_->getOutBit(
      static_cast<int32_t>(gripperAddress_[0]), static_cast<int32_t>(gripperAddress_[1]),
      gripperData, message) == ErrorCode::None;

    if (!gripperRead) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Gripper joints not published: %s", message.c_str());
    } else {
      for (const std::string & name : gripperJointNames_) {
        encoderJointMsg.name.push_back(name);
        encoderJointMsg.position.push_back(gripperData ? gripperCloseValue_ : gripperOpenValue_);
        encoderJointMsg.velocity.push_back(0.000);
      }
    }
  }

  isaacsimJointPub_->publish(encoderJointMsg);
  encoderJointMsg.header.stamp = this->get_clock()->now();
  encoderJointPub_->publish(encoderJointMsg);

  std_msgs::msg::Float64MultiArray gazeboJointMsg;

  if (gazeboVelocityPub_) {
    gazeboJointMsg.data.clear();
    for (int64_t axis : gazeboVelocityAxes_) {
      const auto it = std::find(jointAxes_.begin(), jointAxes_.end(), axis);
      const size_t i = static_cast<size_t>(std::distance(jointAxes_.begin(), it));
      gazeboJointMsg.data.push_back(i < feedback.size() ? feedback[i].actualVelocity : 0.0);
    }
    gazeboVelocityPub_->publish(gazeboJointMsg);
  }

  if (gazeboPositionPub_) {
    gazeboJointMsg.data.clear();
    for (int64_t axis : gazeboPositionAxes_) {
      const auto it = std::find(jointAxes_.begin(), jointAxes_.end(), axis);
      const size_t i = static_cast<size_t>(std::distance(jointAxes_.begin(), it));
      gazeboJointMsg.data.push_back(i < feedback.size() ? feedback[i].actualPos : 0.0);
    }
    for (size_t i = feedback.size(); i < encoderJointMsg.position.size(); ++i) {
      gazeboJointMsg.data.push_back(encoderJointMsg.position[i]);
    }
    gazeboPositionPub_->publish(gazeboJointMsg);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStateBroadcaster>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
