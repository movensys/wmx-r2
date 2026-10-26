// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_io_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

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

std::string failureText(const std::string & call, const std::string & where, int err)
{
  return call + " failed " + where + ". Error=" + std::to_string(err) +
         " (" + ioErrorText(err) + ")";
}
}  // namespace

WmxIoNodeApi::WmxIoNodeApi(const rclcpp::Logger & logger)
: logger_(logger)
{
}

WmxIoNodeApi::~WmxIoNodeApi()
{
  if (wmxIo_) {
    releaseDevice();
  }
}

int WmxIoNodeApi::attachDevice(std::string & message)
{
  if (wmxIo_) {
    message = "Already attached to the WMX3 device";
    return ErrorCode::None;
  }

  const int err = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout_);
  if (err != ErrorCode::None) {
    if (err == ErrorCode::StartProcessLockError) {
      message = "Failed to attach to device (lock busy, will retry on next signal).";
      RCLCPP_WARN(logger_, "%s", message.c_str());
    } else {
      message = "Failed to attach to device. Error=" + std::to_string(err) +
        " (" + ioErrorText(err) + ")";
      RCLCPP_ERROR(logger_, "%s", message.c_str());
    }
    return err;
  }

  wmx3Lib_.SetDeviceName(deviceName_);
  wmxIo_ = std::make_unique<IO>(&wmx3Lib_);

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void WmxIoNodeApi::releaseDevice()
{
  wmxIo_.reset();

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, ioErrorText(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "Device closed");
  }
}

int WmxIoNodeApi::getInputBit(int32_t byte, int32_t bit, int32_t & value, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot read input bit. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  unsigned char data = 0;
  const int err = wmxIo_->GetInBitEx(byte, bit, &data);
  if (err != ErrorCode::None) {
    message = failureText(
      "GetInBitEx", "byte=" + std::to_string(byte) + " bit=" + std::to_string(bit), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  value = static_cast<int32_t>(data);
  message = "Input bit " + std::to_string(byte) + "." + std::to_string(bit) + " = " +
    std::to_string(data);
  return ErrorCode::None;
}

int WmxIoNodeApi::getOutputBit(int32_t byte, int32_t bit, int32_t & value, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot read output bit. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  unsigned char data = 0;
  const int err = wmxIo_->GetOutBitEx(byte, bit, &data);
  if (err != ErrorCode::None) {
    message = failureText(
      "GetOutBitEx", "byte=" + std::to_string(byte) + " bit=" + std::to_string(bit), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  value = static_cast<int32_t>(data);
  message = "Output bit " + std::to_string(byte) + "." + std::to_string(bit) + " = " +
    std::to_string(data);
  return ErrorCode::None;
}

int WmxIoNodeApi::getInputBytes(
  int32_t byte, int32_t length, std::vector<uint8_t> & data, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot read input bytes. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  if (length <= 0) {
    message = "Invalid length: must be > 0";
    return ErrorCode::IOSizeOutOfRange;
  }

  std::vector<unsigned char> raw(length, 0);
  const int err = wmxIo_->GetInBytesEx(byte, length, raw.data());
  if (err != ErrorCode::None) {
    message = failureText(
      "GetInBytesEx", "byte=" + std::to_string(byte) + " length=" + std::to_string(length), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  data.assign(raw.begin(), raw.end());
  message = "Read " + std::to_string(length) + " input bytes from byte " + std::to_string(byte);
  return ErrorCode::None;
}

int WmxIoNodeApi::getOutputBytes(
  int32_t byte, int32_t length, std::vector<uint8_t> & data, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot read output bytes. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  if (length <= 0) {
    message = "Invalid length: must be > 0";
    return ErrorCode::IOSizeOutOfRange;
  }

  std::vector<unsigned char> raw(length, 0);
  const int err = wmxIo_->GetOutBytesEx(byte, length, raw.data());
  if (err != ErrorCode::None) {
    message = failureText(
      "GetOutBytesEx", "byte=" + std::to_string(byte) + " length=" + std::to_string(length), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  data.assign(raw.begin(), raw.end());
  message = "Read " + std::to_string(length) + " output bytes from byte " + std::to_string(byte);
  return ErrorCode::None;
}

int WmxIoNodeApi::setOutputBit(int32_t byte, int32_t bit, int32_t value, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot set output bit. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  if (value != 0 && value != 1) {
    message = "Invalid value: must be 0 or 1";
    return ErrorCode::ArgumentOutOfRange;
  }

  const int err = wmxIo_->SetOutBitEx(byte, bit, (value ? 1 : 0));
  if (err != ErrorCode::None) {
    message = failureText(
      "SetOutBitEx",
      "byte=" + std::to_string(byte) + " bit=" + std::to_string(bit) +
      " value=" + std::to_string(value), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set output bit " + std::to_string(byte) + "." + std::to_string(bit) + " = " +
    std::to_string(value);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxIoNodeApi::setOutputBytes(
  int32_t byte, const std::vector<uint8_t> & data, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot set output bytes. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  if (data.empty()) {
    message = "No data provided";
    return ErrorCode::IOSizeOutOfRange;
  }

  const int length = static_cast<int>(data.size());
  std::vector<unsigned char> raw(data.begin(), data.end());

  const int err = wmxIo_->SetOutBytesEx(byte, length, raw.data());
  if (err != ErrorCode::None) {
    message = failureText(
      "SetOutBytesEx", "byte=" + std::to_string(byte) + " length=" + std::to_string(length), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set " + std::to_string(length) + " output bytes from byte " + std::to_string(byte);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

WmxIoNode::WmxIoNode()
: LifecycleNode("wmx_io_node")
{
  api_ = std::make_unique<WmxIoNodeApi>(this->get_logger());
  RCLCPP_INFO(this->get_logger(), "wmx_io_node is unconfigured, waiting for configure...");
}

WmxIoNode::~WmxIoNode()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "wmx_io_node stopped");
}

bool WmxIoNode::isNodeActive()
{
  return this->get_current_state().id() ==
         lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

std::string WmxIoNode::notActiveMessage()
{
  return "wmx_io_node is not active (state: " +
         this->get_current_state().label() + ").";
}

WmxIoNode::CallbackReturn WmxIoNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring wmx_io_node...");

  std::string message;
  if (api_->attachDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  getInputBitService_ = this->create_service<wmx_r2_message::srv::GetIoBit>(
    "wmx/io/get_input_bit",
    std::bind(&WmxIoNode::getInputBitCallback, this, _1, _2));

  getOutputBitService_ = this->create_service<wmx_r2_message::srv::GetIoBit>(
    "wmx/io/get_output_bit",
    std::bind(&WmxIoNode::getOutputBitCallback, this, _1, _2));

  getInputBytesService_ = this->create_service<wmx_r2_message::srv::GetIoBytes>(
    "wmx/io/get_input_bytes",
    std::bind(&WmxIoNode::getInputBytesCallback, this, _1, _2));

  getOutputBytesService_ = this->create_service<wmx_r2_message::srv::GetIoBytes>(
    "wmx/io/get_output_bytes",
    std::bind(&WmxIoNode::getOutputBytesCallback, this, _1, _2));

  setOutputBitService_ = this->create_service<wmx_r2_message::srv::SetIoBit>(
    "wmx/io/set_output_bit",
    std::bind(&WmxIoNode::setOutputBitCallback, this, _1, _2));

  setOutputBytesService_ = this->create_service<wmx_r2_message::srv::SetIoBytes>(
    "wmx/io/set_output_bytes",
    std::bind(&WmxIoNode::setOutputBytesCallback, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "wmx_io_node is configured");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  RCLCPP_INFO(this->get_logger(), "wmx_io_node is active");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "wmx_io_node is inactive");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  getInputBitService_.reset();
  getOutputBitService_.reset();
  getInputBytesService_.reset();
  getOutputBytesService_.reset();
  setOutputBitService_.reset();
  setOutputBytesService_.reset();

  if (api_->isDeviceOpen()) {
    api_->releaseDevice();
  }

  RCLCPP_INFO(this->get_logger(), "wmx_io_node is cleaned up");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void WmxIoNode::getInputBitCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetIoBit::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetIoBit::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  int32_t value = 0;
  std::string message;
  const int err = api_->getInputBit(request->byte, request->bit, value, message);

  response->success = (err == ErrorCode::None);
  response->value = value;
  response->message = message;
}

void WmxIoNode::getOutputBitCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetIoBit::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetIoBit::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  int32_t value = 0;
  std::string message;
  const int err = api_->getOutputBit(request->byte, request->bit, value, message);

  response->success = (err == ErrorCode::None);
  response->value = value;
  response->message = message;
}

void WmxIoNode::getInputBytesCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::vector<uint8_t> data;
  std::string message;
  const int err = api_->getInputBytes(request->byte, request->length, data, message);

  response->success = (err == ErrorCode::None);
  response->data = data;
  response->message = message;
}

void WmxIoNode::getOutputBytesCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::vector<uint8_t> data;
  std::string message;
  const int err = api_->getOutputBytes(request->byte, request->length, data, message);

  response->success = (err == ErrorCode::None);
  response->data = data;
  response->message = message;
}

void WmxIoNode::setOutputBitCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetIoBit::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetIoBit::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  const int err = api_->setOutputBit(request->byte, request->bit, request->value, message);

  response->success = (err == ErrorCode::None);
  response->message = message;
}

void WmxIoNode::setOutputBytesCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetIoBytes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetIoBytes::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  const int err = api_->setOutputBytes(request->byte, request->data, message);

  response->success = (err == ErrorCode::None);
  response->message = message;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxIoNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
