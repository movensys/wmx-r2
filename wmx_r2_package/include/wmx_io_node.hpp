// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_IO_NODE_HPP_
#define WMX_IO_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "wmx_r2_message/srv/get_io_bit.hpp"
#include "wmx_r2_message/srv/get_io_bytes.hpp"
#include "wmx_r2_message/srv/set_io_bit.hpp"
#include "wmx_r2_message/srv/set_io_bytes.hpp"

#include "WMX3Api.h"
#include "IOApi.h"

class WmxIoNodeApi
{
public:
  explicit WmxIoNodeApi(const rclcpp::Logger & logger);
  ~WmxIoNodeApi();

  int attachDevice(std::string & message);
  void releaseDevice();

  int getInputBit(int32_t byte, int32_t bit, int32_t & value, std::string & message);
  int getInputBytes(
    int32_t byte, int32_t length, std::vector<uint8_t> & data, std::string & message);
  int getOutputBit(int32_t byte, int32_t bit, int32_t & value, std::string & message);
  int getOutputBytes(
    int32_t byte, int32_t length, std::vector<uint8_t> & data, std::string & message);

  int setOutputBit(int32_t byte, int32_t bit, int32_t value, std::string & message);
  int setOutputBytes(int32_t byte, const std::vector<uint8_t> & data, std::string & message);

  bool isDeviceOpen() const {return wmxIo_ != nullptr;}

private:
  rclcpp::Logger logger_;

  const char * deviceName_ = "wmx_io_node";
  unsigned int timeout_ = 10000;

  wmx3Api::WMX3Api wmx3Lib_;
  std::unique_ptr<wmx3Api::IO> wmxIo_;
};

class WmxIoNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  WmxIoNode();
  ~WmxIoNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::unique_ptr<WmxIoNodeApi> api_;

  rclcpp::Service<wmx_r2_message::srv::GetIoBit>::SharedPtr getInputBitService_;
  rclcpp::Service<wmx_r2_message::srv::GetIoBit>::SharedPtr getOutputBitService_;
  rclcpp::Service<wmx_r2_message::srv::GetIoBytes>::SharedPtr getInputBytesService_;
  rclcpp::Service<wmx_r2_message::srv::GetIoBytes>::SharedPtr getOutputBytesService_;
  rclcpp::Service<wmx_r2_message::srv::SetIoBit>::SharedPtr setOutputBitService_;
  rclcpp::Service<wmx_r2_message::srv::SetIoBytes>::SharedPtr setOutputBytesService_;

  bool isNodeActive();
  std::string notActiveMessage();

  void getInputBitCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetIoBit::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoBit::Response> response);
  void getOutputBitCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetIoBit::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoBit::Response> response);
  void getInputBytesCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Response> response);
  void getOutputBytesCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Response> response);
  void setOutputBitCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetIoBit::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetIoBit::Response> response);
  void setOutputBytesCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetIoBytes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetIoBytes::Response> response);
};

#endif  // WMX_IO_NODE_HPP_
