// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "gripper_controller.hpp"

#include <thread>

#include <chrono>

#include <cstdlib>

using std::placeholders::_1;
using std::placeholders::_2;

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::IO;

namespace
{
constexpr int32_t kCr3aGripperPowerByte = 28;
constexpr int32_t kCr3aGripperPowerValue = 113;
constexpr int32_t kCr3aGripperSenseByte = 0;
constexpr int32_t kCr3aGripperSenseBit = 1;

std::string errorToString(int err)
{
  char errString[256] = {};
  IO::ErrorToString(err, errString, sizeof(errString));
  return errString;
}
}  // namespace

GripperControllerApi::GripperControllerApi(const rclcpp::Logger & logger)
: logger_(logger), io_(&wmx3Lib_)
{
}

GripperControllerApi::~GripperControllerApi()
{
  closeDevice();
}

int GripperControllerApi::createDevice(std::string & message)
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

  io_ = IO(&wmx3Lib_);

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void GripperControllerApi::closeDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorToString(err).c_str());
    return;
  }

  RCLCPP_INFO(logger_, "Device closed");
}

int GripperControllerApi::setOutBit(
  int32_t addr, int32_t bit, uint8_t data, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = io_.SetOutBit(addr, bit, data);
  if (err != ErrorCode::None) {
    message = "SetOutBit failed. addr=" + std::to_string(addr) + " bit=" + std::to_string(bit) +
      " data=" + std::to_string(data) + ". Error=" + std::to_string(err) +
      " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "SetOutBit success. addr=" + std::to_string(addr) + " bit=" + std::to_string(bit) +
    " data=" + std::to_string(data);
  return ErrorCode::None;
}

int GripperControllerApi::setOutByte(int32_t addr, uint8_t data, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = io_.SetOutByte(addr, data);
  if (err != ErrorCode::None) {
    message = "SetOutByte failed. addr=" + std::to_string(addr) + " data=" +
      std::to_string(data) + ". Error=" + std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "SetOutByte success. addr=" + std::to_string(addr) + " data=" + std::to_string(data);
  return ErrorCode::None;
}

int GripperControllerApi::getOutByte(int32_t addr, uint8_t & data, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = io_.GetOutByte(addr, &data);
  if (err != ErrorCode::None) {
    message = "GetOutByte failed. addr=" + std::to_string(addr) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "GetOutByte success. addr=" + std::to_string(addr) + " data=" + std::to_string(data);
  return ErrorCode::None;
}

int GripperControllerApi::getInBit(
  int32_t addr, int32_t bit, uint8_t & data, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = io_.GetInBit(addr, bit, &data);
  if (err != ErrorCode::None) {
    message = "GetInBit failed. addr=" + std::to_string(addr) + " bit=" + std::to_string(bit) +
      ". Error=" + std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "GetInBit success. addr=" + std::to_string(addr) + " bit=" + std::to_string(bit) +
    " data=" + std::to_string(data);
  return ErrorCode::None;
}

GripperController::GripperController()
: LifecycleNode("gripper_controller")
{
  setRosParameter();

  api_ = std::make_unique<GripperControllerApi>(this->get_logger());

  RCLCPP_INFO(this->get_logger(), "gripper_controller is unconfigured, waiting for configure...");
}

GripperController::~GripperController()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "gripper_controller stopped");
}

void GripperController::setRosParameter()
{
  wmxGripperTopic_ = this->declare_parameter<std::string>(
    "wmx_gripper_topic", "/wmx_gripper_topic/no_param");
  gripperAddress_ = this->declare_parameter<std::vector<int64_t>>(
    "gripper_address", std::vector<int64_t>{0, 0});

  if (gripperAddress_.size() < 2) {
    RCLCPP_WARN(
      this->get_logger(),
      "gripper_address needs [byte, bit], got %zu entries. Falling back to [0, 0].",
      gripperAddress_.size());
    gripperAddress_ = {0, 0};
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "wmx_gripper_topic: %s", wmxGripperTopic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "gripper_address: [%ld, %ld]",
    gripperAddress_[0], gripperAddress_[1]);
  RCLCPP_INFO(this->get_logger(), "===========================");
}

GripperController::CallbackReturn GripperController::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring gripper_controller...");

  std::string message;
  if (api_->createDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  const char * manipulatorModel = std::getenv("MANIPULATOR_MODEL");
  if (manipulatorModel && std::string(manipulatorModel) == "dobot_cr3a") {
    dobotCR3AGripperSetup();
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Skipping dobotCR3AGripperSetup (MANIPULATOR_MODEL=%s)",
      manipulatorModel ? manipulatorModel : "not set");
  }

  RCLCPP_INFO(this->get_logger(), "gripper_controller is configured");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  setGripperService_ = this->create_service<std_srvs::srv::SetBool>(
    wmxGripperTopic_,
    std::bind(&GripperController::setGripperCallback, this, _1, _2));

  LifecycleNode::on_activate(previous_state);

  RCLCPP_INFO(this->get_logger(), "gripper_controller is active");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_deactivate(previous_state);

  setGripperService_.reset();

  RCLCPP_INFO(this->get_logger(), "gripper_controller is inactive");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  api_->closeDevice();

  RCLCPP_INFO(this->get_logger(), "gripper_controller is cleaned up");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(previous_state);
  }

  return on_cleanup(previous_state);
}

void GripperController::dobotCR3AGripperSetup()
{
  std::string message;

  if (api_->setOutByte(
      kCr3aGripperPowerByte, kCr3aGripperPowerValue, message) != ErrorCode::None)
  {
    RCLCPP_ERROR(this->get_logger(), "[dobot_cr3a] gripper setup failed: %s", message.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "[dobot_cr3a] gripper power byte set");

  uint8_t switchData = 0;
  if (api_->getOutByte(kCr3aGripperPowerByte, switchData, message) != ErrorCode::None) {
    RCLCPP_ERROR(this->get_logger(), "[dobot_cr3a] gripper setup failed: %s", message.c_str());
    return;
  }

  uint8_t powerData = 0;
  if (api_->getInBit(
      kCr3aGripperSenseByte, kCr3aGripperSenseBit, powerData, message) != ErrorCode::None)
  {
    RCLCPP_ERROR(this->get_logger(), "[dobot_cr3a] gripper setup failed: %s", message.c_str());
    return;
  }

  if (switchData == kCr3aGripperPowerValue && powerData == 1) {
    RCLCPP_INFO(this->get_logger(), "[dobot_cr3a] gripper is on and ready");
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "[dobot_cr3a] gripper state unexpected: switchData=%u, powerData=%u",
      switchData, powerData);
  }
}

void GripperController::setGripperCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  const uint8_t bitData = request->data ? 1 : 0;
  const char * action = request->data ? "close" : "open";

  std::string message;
  if (api_->setOutBit(
      static_cast<int32_t>(gripperAddress_[0]), static_cast<int32_t>(gripperAddress_[1]),
      bitData, message) != ErrorCode::None)
  {
    RCLCPP_ERROR(this->get_logger(), "Gripper fails to %s: %s", action, message.c_str());
    response->success = false;
    response->message = std::string("Failed to ") + action + " gripper";
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Gripper success to %s", action);
  response->success = true;
  response->message = std::string("Gripper ") + action + "ed successfully";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
