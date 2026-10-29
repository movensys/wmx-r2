// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_ETHERCAT_NODE_HPP_
#define WMX_ETHERCAT_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "wmx_r2_message/srv/ecat_get_master_info.hpp"
#include "wmx_r2_message/srv/ecat_register_read.hpp"
#include "wmx_r2_message/srv/ecat_reset_statistics.hpp"
#include "wmx_r2_message/srv/ecat_scan_network.hpp"
#include "wmx_r2_message/srv/ecat_start_hotconnect.hpp"

#include "WMX3Api.h"
#include "EcApi.h"

class WmxEtherCatNodeApi
{
public:
  explicit WmxEtherCatNodeApi(const rclcpp::Logger & logger);
  ~WmxEtherCatNodeApi();

  int createDevice(std::string & message);
  void closeDevice();

  int getMasterInfo(
    int32_t masterId, wmx3Api::ecApi::EcMasterInfo & info, std::string & message);
  int registerRead(
    int32_t masterId, int32_t slaveId, int32_t regAddr, int32_t len,
    std::vector<uint8_t> & data, std::string & message);
  int resetStatistics(int32_t masterId, std::string & message);
  int scanNetwork(int32_t masterId, std::string & message);
  int startHotconnect(int32_t masterId, std::string & message);

private:
  rclcpp::Logger logger_;

  const char * deviceName_ = "wmx_ethercat_node";
  unsigned int timeout_ = 10000;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::ecApi::Ecat wmxEcat_;
};

class WmxEtherCatNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  WmxEtherCatNode();
  ~WmxEtherCatNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::unique_ptr<WmxEtherCatNodeApi> api_;

  rclcpp::Service<wmx_r2_message::srv::EcatGetMasterInfo>::SharedPtr getMasterInfoService_;
  rclcpp::Service<wmx_r2_message::srv::EcatRegisterRead>::SharedPtr registerReadService_;
  rclcpp::Service<wmx_r2_message::srv::EcatResetStatistics>::SharedPtr resetStatisticsService_;
  rclcpp::Service<wmx_r2_message::srv::EcatScanNetwork>::SharedPtr scanNetworkService_;
  rclcpp::Service<wmx_r2_message::srv::EcatStartHotconnect>::SharedPtr startHotconnectService_;

  void getMasterInfoCallback(
    const std::shared_ptr<wmx_r2_message::srv::EcatGetMasterInfo::Request> request,
    std::shared_ptr<wmx_r2_message::srv::EcatGetMasterInfo::Response> response);

  void registerReadCallback(
    const std::shared_ptr<wmx_r2_message::srv::EcatRegisterRead::Request> request,
    std::shared_ptr<wmx_r2_message::srv::EcatRegisterRead::Response> response);

  void resetStatisticsCallback(
    const std::shared_ptr<wmx_r2_message::srv::EcatResetStatistics::Request> request,
    std::shared_ptr<wmx_r2_message::srv::EcatResetStatistics::Response> response);

  void scanNetworkCallback(
    const std::shared_ptr<wmx_r2_message::srv::EcatScanNetwork::Request> request,
    std::shared_ptr<wmx_r2_message::srv::EcatScanNetwork::Response> response);

  void startHotconnectCallback(
    const std::shared_ptr<wmx_r2_message::srv::EcatStartHotconnect::Request> request,
    std::shared_ptr<wmx_r2_message::srv::EcatStartHotconnect::Response> response);
};

#endif  // WMX_ETHERCAT_NODE_HPP_
