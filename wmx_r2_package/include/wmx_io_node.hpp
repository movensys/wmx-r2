// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_IO_NODE_HPP_
#define WMX_IO_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "wmx_r2_message/srv/get_io_bit.hpp"
#include "wmx_r2_message/srv/get_io_bits.hpp"
#include "wmx_r2_message/srv/get_io_byte.hpp"
#include "wmx_r2_message/srv/get_io_bytes.hpp"
#include "wmx_r2_message/srv/set_io_bit.hpp"
#include "wmx_r2_message/srv/set_io_bits.hpp"
#include "wmx_r2_message/srv/set_io_byte.hpp"
#include "wmx_r2_message/srv/set_io_bytes.hpp"

#include "WMX3Api.h"
#include "IOApi.h"

class WmxIoNodeApi
{
public:
  explicit WmxIoNodeApi(const rclcpp::Logger & logger);
  ~WmxIoNodeApi();

  int createDevice(std::string & message);
  void closeDevice();

  int getInBit(int32_t addr, int32_t bit, uint8_t & data, std::string & message);
  int getInBits(
    const std::vector<int32_t> & addr, const std::vector<int32_t> & bit,
    std::vector<uint8_t> & data, std::string & message);
  int getInByte(int32_t addr, uint8_t & data, std::string & message);
  int getInBytes(
    int32_t addr, int32_t size, std::vector<uint8_t> & data, std::string & message);
  int getOutBit(int32_t addr, int32_t bit, uint8_t & data, std::string & message);
  int getOutBits(
    const std::vector<int32_t> & addr, const std::vector<int32_t> & bit,
    std::vector<uint8_t> & data, std::string & message);
  int getOutByte(int32_t addr, uint8_t & data, std::string & message);
  int getOutBytes(
    int32_t addr, int32_t size, std::vector<uint8_t> & data, std::string & message);

  int setOutBit(int32_t addr, int32_t bit, uint8_t data, std::string & message);
  int setOutBits(
    const std::vector<int32_t> & addr, const std::vector<int32_t> & bit,
    const std::vector<uint8_t> & data, std::string & message);
  int setOutByte(int32_t addr, uint8_t data, std::string & message);
  int setOutBytes(int32_t addr, const std::vector<uint8_t> & data, std::string & message);

private:
  rclcpp::Logger logger_;

  const char * deviceName_ = "wmx_io_node";
  unsigned int timeout_ = 10000;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::IO wmxIo_;
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

  rclcpp::Service<wmx_r2_message::srv::GetIoBit>::SharedPtr getInBitService_;
  rclcpp::Service<wmx_r2_message::srv::GetIoBits>::SharedPtr getInBitsService_;
  rclcpp::Service<wmx_r2_message::srv::GetIoByte>::SharedPtr getInByteService_;
  rclcpp::Service<wmx_r2_message::srv::GetIoBytes>::SharedPtr getInBytesService_;
  rclcpp::Service<wmx_r2_message::srv::GetIoBit>::SharedPtr getOutBitService_;
  rclcpp::Service<wmx_r2_message::srv::GetIoBits>::SharedPtr getOutBitsService_;
  rclcpp::Service<wmx_r2_message::srv::GetIoByte>::SharedPtr getOutByteService_;
  rclcpp::Service<wmx_r2_message::srv::GetIoBytes>::SharedPtr getOutBytesService_;
  rclcpp::Service<wmx_r2_message::srv::SetIoBit>::SharedPtr setOutBitService_;
  rclcpp::Service<wmx_r2_message::srv::SetIoBits>::SharedPtr setOutBitsService_;
  rclcpp::Service<wmx_r2_message::srv::SetIoByte>::SharedPtr setOutByteService_;
  rclcpp::Service<wmx_r2_message::srv::SetIoBytes>::SharedPtr setOutBytesService_;

  void getInBitCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetIoBit::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoBit::Response> response);
  void getInBitsCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetIoBits::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoBits::Response> response);
  void getInByteCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetIoByte::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoByte::Response> response);
  void getInBytesCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Response> response);
  void getOutBitCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetIoBit::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoBit::Response> response);
  void getOutBitsCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetIoBits::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoBits::Response> response);
  void getOutByteCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetIoByte::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoByte::Response> response);
  void getOutBytesCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Response> response);
  void setOutBitCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetIoBit::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetIoBit::Response> response);
  void setOutBitsCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetIoBits::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetIoBits::Response> response);
  void setOutByteCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetIoByte::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetIoByte::Response> response);
  void setOutBytesCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetIoBytes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetIoBytes::Response> response);
};

#endif  // WMX_IO_NODE_HPP_
