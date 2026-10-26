// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "gripper_controller.hpp"

#include <cstdlib>

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::IO;

namespace
{
std::string ioErrorText(int err)
{
  char errString[256] = {};
  IO::ErrorToString(err, errString, sizeof(errString));
  return errString;
}
}  // namespace

GripperControllerApi::GripperControllerApi(const rclcpp::Logger & logger)
: logger_(logger)
{
}

GripperControllerApi::~GripperControllerApi()
{
  if (io_) {
    releaseDevice();
  }
}

std::string GripperControllerApi::errorText(int err)
{
  char errString[256] = {};
  wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
  return errString;
}

int GripperControllerApi::attachDevice(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (io_) {
    message = "Already attached to the WMX3 device";
    return ErrorCode::None;
  }

  const int err = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout_);
  if (err != ErrorCode::None) {
    message = "Failed to attach to device. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  wmx3Lib_.SetDeviceName(deviceName_);
  io_ = std::make_unique<IO>(&wmx3Lib_);

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void GripperControllerApi::releaseDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  io_.reset();

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorText(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "Device closed");
  }
}

int GripperControllerApi::setOutputBit(
  int32_t byte, int32_t bit, int32_t value, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!io_) {
    message = "Cannot set output bit. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = io_->SetOutBit(byte, bit, (value ? 1 : 0));
  if (err != ErrorCode::None) {
    message = ioErrorText(err);
    return err;
  }

  message = "Set output bit " + std::to_string(byte) + "." + std::to_string(bit) + " = " +
    std::to_string(value);
  return ErrorCode::None;
}

int GripperControllerApi::setOutputByte(int32_t byte, int32_t value, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!io_) {
    message = "Cannot set output byte. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = io_->SetOutByte(byte, static_cast<unsigned char>(value));
  if (err != ErrorCode::None) {
    message = ioErrorText(err);
    return err;
  }

  message = "Set output byte " + std::to_string(byte) + " = " + std::to_string(value);
  return ErrorCode::None;
}

int GripperControllerApi::getOutputByte(int32_t byte, int32_t & value, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!io_) {
    message = "Cannot read output byte. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  unsigned char data = 0;
  const int err = io_->GetOutByte(byte, &data);
  if (err != ErrorCode::None) {
    message = ioErrorText(err);
    return err;
  }

  value = static_cast<int32_t>(data);
  message = "Output byte " + std::to_string(byte) + " = " + std::to_string(value);
  return ErrorCode::None;
}

int GripperControllerApi::getInputBit(
  int32_t byte, int32_t bit, int32_t & value, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!io_) {
    message = "Cannot read input bit. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  unsigned char data = 0;
  const int err = io_->GetInBit(byte, bit, &data);
  if (err != ErrorCode::None) {
    message = ioErrorText(err);
    return err;
  }

  value = static_cast<int32_t>(data);
  message = "Input bit " + std::to_string(byte) + "." + std::to_string(bit) + " = " +
    std::to_string(value);
  return ErrorCode::None;
}

GripperController::GripperController()
: LifecycleNode("gripper_controller")
{
  api_ = std::make_unique<GripperControllerApi>(this->get_logger());

  setRosParameter();

  RCLCPP_INFO(this->get_logger(), "gripper_controller is unconfigured, waiting for configure...");
}

GripperController::~GripperController()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "gripper_controller stopped");
}

bool GripperController::isNodeActive()
{
  return this->get_current_state().id() ==
         lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

std::string GripperController::notActiveMessage()
{
  return "gripper_controller is not active (state: " +
         this->get_current_state().label() + ").";
}

GripperController::CallbackReturn GripperController::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring gripper_controller...");

  std::string message;
  if (api_->attachDevice(message) != ErrorCode::None) {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    return CallbackReturn::FAILURE;
  }

  const char * manipulatorModel = std::getenv("MANIPULATOR_MODEL");
  if (manipulatorModel && std::string(manipulatorModel) == "dobot_cr3a") {
    dobotCR3AGripperSetup();
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Skipping dobotCR3AGripperSetup (MANIPULATOR_MODEL=%s)",
      manipulatorModel ? manipulatorModel : "not set");
  }

  setGripperService_ = this->create_service<std_srvs::srv::SetBool>(
    wmxGripperTopic_,
    std::bind(
      &GripperController::setGripper, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "gripper_controller is configured");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  RCLCPP_INFO(this->get_logger(), "gripper_controller is active");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "gripper_controller is inactive");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_cleanup(const rclcpp_lifecycle::State &)
{
  setGripperService_.reset();
  if (api_->isDeviceOpen()) {
    api_->releaseDevice();
  }

  RCLCPP_INFO(this->get_logger(), "gripper_controller is cleaned up");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void GripperController::dobotCR3AGripperSetup()
{
  std::string message;

  if (api_->setOutputByte(28, 113, message) != ErrorCode::None) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[dobot_cr3a] gripper setup failed (SetOutByte): %s", message.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "[dobot_cr3a] gripper power byte set");

  int32_t gripperSwitchData = 0;
  if (api_->getOutputByte(28, gripperSwitchData, message) != ErrorCode::None) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[dobot_cr3a] gripper setup failed (GetOutByte): %s", message.c_str());
    return;
  }

  int32_t gripperPowerData = 0;
  if (api_->getInputBit(0, 1, gripperPowerData, message) != ErrorCode::None) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[dobot_cr3a] gripper setup failed (GetInBit): %s", message.c_str());
    return;
  }

  if (gripperSwitchData == 113 && gripperPowerData == 1) {
    RCLCPP_INFO(this->get_logger(), "[dobot_cr3a] gripper is on and ready");
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "[dobot_cr3a] gripper state unexpected: switchData=%d, powerData=%d",
      gripperSwitchData, gripperPowerData);
  }
}

void GripperController::setRosParameter()
{
  this->declare_parameter<std::string>("wmx_gripper_topic", "/wmx_gripper_topic/no_param");
  this->declare_parameter<std::vector<int64_t>>("gripper_address", std::vector<int64_t>{0, 0});

  this->get_parameter("wmx_gripper_topic", wmxGripperTopic_);
  this->get_parameter("gripper_address", gripperAddress_);

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "wmx_gripper_topic: %s", wmxGripperTopic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "gripper_address: [%ld, %ld]",
    gripperAddress_[0], gripperAddress_[1]);
  RCLCPP_INFO(this->get_logger(), "===========================");
}

void GripperController::setGripper(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  const char * action = request->data ? "Close" : "Open";

  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  const int err = api_->setOutputBit(
    gripperAddress_[0], gripperAddress_[1], request->data ? 1 : 0, message);

  if (err != ErrorCode::None) {
    RCLCPP_ERROR(this->get_logger(), "Gripper fails to %s: %s", action, message.c_str());
    response->success = false;
    response->message = request->data ? "Failed to close gripper" : "Failed to open gripper";
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Gripper success to %s", action);
  response->success = true;
  response->message = request->data ? "Gripper closed successfully" : "Gripper opened successfully";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
