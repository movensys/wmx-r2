// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "joint_state_broadcaster.hpp"

#include <chrono>

#include "wmx_qos_compat.hpp"

using std::placeholders::_1;

using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::IO;

JointStateBroadcasterApi::JointStateBroadcasterApi(const rclcpp::Logger & logger)
: logger_(logger)
{
}

JointStateBroadcasterApi::~JointStateBroadcasterApi()
{
  if (cm_) {
    releaseDevice();
  }
}

std::string JointStateBroadcasterApi::errorText(int err)
{
  char errString[256] = {};
  wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
  return errString;
}

int JointStateBroadcasterApi::attachDevice(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (cm_) {
    message = "Already attached to the WMX3 device";
    return ErrorCode::None;
  }

  const int err = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout_);
  if (err != ErrorCode::None) {
    message = "Failed to attach to device. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    return err;
  }

  wmx3Lib_.SetDeviceName(deviceName_);

  cm_ = std::make_unique<CoreMotion>(&wmx3Lib_);
  io_ = std::make_unique<IO>(&wmx3Lib_);

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void JointStateBroadcasterApi::releaseDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  io_.reset();
  cm_.reset();

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorText(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "Device closed");
  }
}

int JointStateBroadcasterApi::getStatus(
  const std::vector<int64_t> & axes, std::vector<AxisFeedback> & feedback, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot read status. Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  CoreMotionStatus status;
  const int err = cm_->GetStatus(&status);

  feedback.clear();
  feedback.reserve(axes.size());
  for (const int64_t axis : axes) {
    AxisFeedback entry;
    entry.actualPos = status.axesStatus[axis].actualPos;
    entry.actualVelocity = status.axesStatus[axis].actualVelocity;
    feedback.push_back(entry);
  }

  return err;
}

int JointStateBroadcasterApi::getOutputBit(
  int32_t byte, int32_t bit, int32_t & value, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!io_) {
    message = "Cannot read output bit. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  unsigned char data = 0;
  const int err = io_->GetOutBit(byte, bit, &data);
  value = static_cast<int32_t>(data);
  return err;
}

int JointStateBroadcasterApi::setServoOn(int axis, int on, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot set servo on axis " + std::to_string(axis) +
      ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm_->axisControl->SetServoOn(axis, on);
  if (err != ErrorCode::None) {
    message = "Servo " + std::to_string(axis) + " error to " + (on ? "on" : "off") +
      ". Error=" + std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Servo " + std::to_string(axis) + " " + (on ? "on" : "off");
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

JointStateBroadcaster::JointStateBroadcaster()
: LifecycleNode("joint_state_broadcaster")
{
  api_ = std::make_unique<JointStateBroadcasterApi>(this->get_logger());

  setRosParameter();

  axisClientCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  clearAmpAlarmClient_ = this->create_client<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/clear_amp_alarm", servicesQos(), axisClientCbGroup_);

  setServoOnClient_ = this->create_client<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/set_servo_on", servicesQos(), axisClientCbGroup_);

  RCLCPP_INFO(
    this->get_logger(), "joint_state_broadcaster is unconfigured, waiting for configure...");
}

JointStateBroadcaster::~JointStateBroadcaster()
{
  servoOff();
  api_.reset();

  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster stopped");
}

bool JointStateBroadcaster::isNodeActive() const
{
  return isNodeActive_.load();
}

void JointStateBroadcaster::servoOff()
{
  if (!isNodeActive() || !api_ || !api_->isDeviceOpen()) {
    return;
  }

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
  if (api_->attachDevice(message) != ErrorCode::None) {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    return CallbackReturn::FAILURE;
  }

  encoderJointPub_ = this->create_publisher<sensor_msgs::msg::JointState>(encoderJointTopic_, 1);
  isaacsimJointPub_ = this->create_publisher<sensor_msgs::msg::JointState>(isaacsimJointTopic_, 1);
  gazeboJointPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(gazeboJointTopic_, 1);

  encoderJointTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / jointFeedbackRate_),
    std::bind(&JointStateBroadcaster::publishJointState, this));
  encoderJointTimer_->cancel();

  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster is configured");
  return CallbackReturn::SUCCESS;
}

JointStateBroadcaster::CallbackReturn JointStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  std::vector<int64_t> zeroData(jointAxes_.size(), 0);
  std::vector<int64_t> onData(jointAxes_.size(), 1);

  if (!callSetAxesService(
      clearAmpAlarmClient_, "wmx/axes/clear_amp_alarm", jointAxes_, zeroData))
  {
    RCLCPP_ERROR(this->get_logger(), "Activation failed at clear_amp_alarm");
    return CallbackReturn::FAILURE;
  }

  if (!callSetAxesService(setServoOnClient_, "wmx/axes/set_servo_on", jointAxes_, onData)) {
    RCLCPP_ERROR(this->get_logger(), "Activation failed at set_servo_on");
    return CallbackReturn::FAILURE;
  }

  LifecycleNode::on_activate(previous_state);
  isNodeActive_ = true;
  encoderJointTimer_->reset();

  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster is active");
  return CallbackReturn::SUCCESS;
}

JointStateBroadcaster::CallbackReturn JointStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  encoderJointTimer_->cancel();
  servoOff();
  isNodeActive_ = false;

  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster is inactive");
  return CallbackReturn::SUCCESS;
}

JointStateBroadcaster::CallbackReturn JointStateBroadcaster::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  isNodeActive_ = false;

  encoderJointTimer_.reset();
  encoderJointPub_.reset();
  isaacsimJointPub_.reset();
  gazeboJointPub_.reset();

  if (api_->isDeviceOpen()) {
    api_->releaseDevice();
  }

  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster is cleaned up");
  return CallbackReturn::SUCCESS;
}

JointStateBroadcaster::CallbackReturn JointStateBroadcaster::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

bool JointStateBroadcaster::callSetAxesService(
  rclcpp::Client<wmx_r2_message::srv::SetAxes>::SharedPtr client,
  const std::string & service_name,
  const std::vector<int64_t> & index,
  const std::vector<int64_t> & data)
{
  const int max_retries = 5;
  const auto service_timeout = std::chrono::seconds(10);
  const auto call_timeout = std::chrono::seconds(15);

  if (!client->wait_for_service(service_timeout)) {
    RCLCPP_ERROR(this->get_logger(), "Service %s not available", service_name.c_str());
    return false;
  }

  for (int attempt = 1; attempt <= max_retries; attempt++) {
    auto request = std::make_shared<wmx_r2_message::srv::SetAxes::Request>();
    request->indices.assign(index.begin(), index.end());
    request->data.assign(data.begin(), data.end());

    RCLCPP_INFO(
      this->get_logger(), "Calling %s (attempt %d/%d)",
      service_name.c_str(), attempt, max_retries);

    auto future = client->async_send_request(request);
    if (future.wait_for(call_timeout) != std::future_status::ready) {
      client->remove_pending_request(future);
      RCLCPP_WARN(
        this->get_logger(), "Service call %s timed out (attempt %d/%d)",
        service_name.c_str(), attempt, max_retries);
      continue;
    }

    auto response = future.get();
    if (!response->success) {
      if (response->message.find("not initialized") != std::string::npos) {
        RCLCPP_WARN(
          this->get_logger(),
          "%s: server not ready yet (attempt %d/%d), retrying...",
          service_name.c_str(), attempt, max_retries);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        continue;
      }
      RCLCPP_ERROR(
        this->get_logger(), "%s failed: %s",
        service_name.c_str(), response->message.c_str());
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(), "%s succeeded: %s",
      service_name.c_str(), response->message.c_str());
    return true;
  }

  RCLCPP_ERROR(
    this->get_logger(), "Service call %s failed after %d attempts",
    service_name.c_str(), max_retries);
  return false;
}

void JointStateBroadcaster::setRosParameter()
{
  this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  this->declare_parameter<int>("joint_feedback_rate", 0);
  this->declare_parameter<float>("gripper_open_value", 0);
  this->declare_parameter<float>("gripper_close_value", 0);
  this->declare_parameter<std::vector<std::string>>(
    "joint_name", {"j1", "j2", "j3", "j4", "j5", "j6"});
  this->declare_parameter<std::vector<std::string>>(
    "gripper_joint_name", std::vector<std::string>{});
  this->declare_parameter<std::vector<int64_t>>("gripper_address", std::vector<int64_t>{0, 0});
  this->declare_parameter<std::string>("encoder_joint_topic", "/encoder_joint_topic/no_param");
  this->declare_parameter<std::string>("isaacsim_joint_topic", "/isaacsim_joint_topic/no_param");
  this->declare_parameter<std::string>("gazebo_joint_topic", "/gazebo_joint_topic/no_param");

  this->get_parameter("joint_axes", jointAxes_);
  this->get_parameter("joint_feedback_rate", jointFeedbackRate_);
  this->get_parameter("gripper_open_value", gripperOpenValue_);
  this->get_parameter("gripper_close_value", gripperCloseValue_);
  this->get_parameter("joint_name", jointNames_);
  this->get_parameter("gripper_joint_name", gripperJointNames_);
  this->get_parameter("gripper_address", gripperAddress_);
  this->get_parameter("encoder_joint_topic", encoderJointTopic_);
  this->get_parameter("isaacsim_joint_topic", isaacsimJointTopic_);
  this->get_parameter("gazebo_joint_topic", gazeboJointTopic_);

  if (jointFeedbackRate_ <= 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "joint_feedback_rate must be > 0, got %d. Falling back to 100 Hz.", jointFeedbackRate_);
    jointFeedbackRate_ = 100;
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_feedback_rate: %d", jointFeedbackRate_);

  std::string joint_names_str;
  for (size_t i = 0; i < jointNames_.size(); ++i) {
    if (i > 0) {joint_names_str += ", ";}
    joint_names_str += jointNames_[i];
  }
  RCLCPP_INFO(this->get_logger(), "joint_name: [%s]", joint_names_str.c_str());

  std::string joint_axes_str;
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    if (i > 0) {joint_axes_str += ", ";}
    joint_axes_str += std::to_string(jointAxes_[i]);
  }
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", joint_axes_str.c_str());

  if (!gripperJointNames_.empty()) {
    std::string gripper_joint_names_str;
    for (size_t i = 0; i < gripperJointNames_.size(); ++i) {
      if (i > 0) {gripper_joint_names_str += ", ";}
      gripper_joint_names_str += gripperJointNames_[i];
    }
    RCLCPP_INFO(this->get_logger(), "gripper_joint_name: [%s]", gripper_joint_names_str.c_str());
    RCLCPP_INFO(this->get_logger(), "gripper_open_value: %f", gripperOpenValue_);
    RCLCPP_INFO(this->get_logger(), "gripper_close_value: %f", gripperCloseValue_);
    if (gripperAddress_.size() >= 2) {
      RCLCPP_INFO(
        this->get_logger(), "gripper_address: [%ld, %ld]",
        gripperAddress_[0], gripperAddress_[1]);
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "gripper_joint_name: [] (no gripper)");
  }
  RCLCPP_INFO(this->get_logger(), "encoder_joint_topic: %s", encoderJointTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "isaacsim_joint_topic: %s", isaacsimJointTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "gazebo_joint_topic: %s", gazeboJointTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "===========================");
}

void JointStateBroadcaster::publishJointState()
{
  std::string message;

  std::vector<JointStateBroadcasterApi::AxisFeedback> feedback;
  api_->getStatus(jointAxes_, feedback, message);

  sensor_msgs::msg::JointState encoderJointMsg_;
  std_msgs::msg::Float64MultiArray gazeboJointMsg_;

  for (size_t i = 0; i < feedback.size(); ++i) {
    encoderJointMsg_.name.push_back(jointNames_[i]);
    encoderJointMsg_.position.push_back(feedback[i].actualPos);
    encoderJointMsg_.velocity.push_back(feedback[i].actualVelocity);
  }

  for (size_t i = 0; i < gripperJointNames_.size() && gripperAddress_.size() >= 1; ++i) {
    encoderJointMsg_.name.push_back(gripperJointNames_[i]);

    int32_t gripperData = 0;
    api_->getOutputBit(gripperAddress_[0], gripperAddress_[1], gripperData, message);

    if (gripperData) {
      encoderJointMsg_.position.push_back(gripperCloseValue_);
      encoderJointMsg_.velocity.push_back(0.000);
    } else {
      encoderJointMsg_.position.push_back(gripperOpenValue_);
      encoderJointMsg_.velocity.push_back(0.000);
    }
  }

  isaacsimJointPub_->publish(encoderJointMsg_);
  encoderJointMsg_.header.stamp = this->get_clock()->now();
  encoderJointPub_->publish(encoderJointMsg_);

  gazeboJointMsg_.data = encoderJointMsg_.position;
  gazeboJointPub_->publish(gazeboJointMsg_);
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
