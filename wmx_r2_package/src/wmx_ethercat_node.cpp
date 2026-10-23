// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_ethercat_node.hpp"

#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::ecApi::Ecat;

namespace
{
std::string errorToString(int err)
{
  char errString[256] = {};
  Ecat::ErrorToString(err, errString, sizeof(errString));
  return errString;
}

std::string failureText(const std::string & call, const std::string & where, int err)
{
  return call + " failed. " + where + " Error=" + std::to_string(err) +
         " (" + errorToString(err) + ")";
}

std::string masterIdText(int32_t masterId)
{
  return "masterId=" + std::to_string(masterId);
}

}  // namespace

WmxEtherCatNodeApi::WmxEtherCatNodeApi(const rclcpp::Logger & logger)
: logger_(logger), wmxEcat_(&wmx3Lib_)
{
}

WmxEtherCatNodeApi::~WmxEtherCatNodeApi()
{
  closeDevice();
}

int WmxEtherCatNodeApi::createDevice(std::string & message)
{
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

  wmxEcat_ = Ecat(&wmx3Lib_);

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void WmxEtherCatNodeApi::closeDevice()
{
  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorToString(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "Device closed");
  }
}

int WmxEtherCatNodeApi::getMasterInfo(
  int32_t masterId, wmx3Api::ecApi::EcMasterInfo & info, std::string & message)
{
  const int err = wmxEcat_.GetMasterInfo(masterId, &info);
  if (err != ErrorCode::None) {
    message = failureText("GetMasterInfo", masterIdText(masterId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Master " + std::to_string(masterId) + ": slaves=" +
    std::to_string(info.numOfSlaves);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxEtherCatNodeApi::registerRead(
  int32_t masterId, int32_t slaveId, int32_t regAddr, int32_t len,
  std::vector<uint8_t> & data, std::string & message)
{
  if (regAddr < 0 || regAddr > 0xFFF) {
    message = "Invalid reg_addr: must be in [0x000, 0xFFF].";
    return ErrorCode::ArgumentOutOfRange;
  }

  if (len <= 0 || len > 0x1000) {
    message = "Invalid len: must be in [1, 4096] bytes.";
    return ErrorCode::ArgumentOutOfRange;
  }

  if (regAddr + len > 0x1000) {
    message = "reg_addr + len exceeds 0x1000.";
    return ErrorCode::ArgumentOutOfRange;
  }

  std::vector<unsigned char> raw(len, 0);

  const int err = wmxEcat_.RegisterRead(masterId, slaveId, regAddr, len, raw.data());
  if (err != ErrorCode::None) {
    message = failureText(
      "RegisterRead",
      "slaveId=" + std::to_string(slaveId) + " reg=" + std::to_string(regAddr) +
      " len=" + std::to_string(len), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  data.assign(raw.begin(), raw.end());
  message = "RegisterRead success. slaveId=" + std::to_string(slaveId) + " reg=" +
    std::to_string(regAddr) + " len=" + std::to_string(len);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxEtherCatNodeApi::resetStatistics(int32_t masterId, std::string & message)
{
  int err = wmxEcat_.ResetRefClockInfo(masterId);
  if (err != ErrorCode::None) {
    message = failureText("ResetRefClockInfo", masterIdText(masterId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = wmxEcat_.ResetTransmitStatisticsInfo(masterId);
  if (err != ErrorCode::None) {
    message = failureText("ResetTransmitStatisticsInfo", masterIdText(masterId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = wmxEcat_.ScanNetwork(masterId);
  if (err != ErrorCode::None) {
    message = failureText("ScanNetwork", masterIdText(masterId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Statistics reset and ScanNetwork done. " + masterIdText(masterId);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxEtherCatNodeApi::scanNetwork(int32_t masterId, std::string & message)
{
  const int err = wmxEcat_.ScanNetwork(masterId);
  if (err != ErrorCode::None) {
    message = failureText("ScanNetwork", masterIdText(masterId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "ScanNetwork done. " + masterIdText(masterId);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxEtherCatNodeApi::startHotconnect(int32_t masterId, std::string & message)
{
  const int err = wmxEcat_.StartHotconnect(masterId);
  if (err != ErrorCode::None) {
    message = failureText("StartHotconnect", masterIdText(masterId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "StartHotconnect done. " + masterIdText(masterId);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

WmxEtherCatNode::WmxEtherCatNode()
: LifecycleNode("wmx_ethercat_node")
{
  api_ = std::make_unique<WmxEtherCatNodeApi>(this->get_logger());
  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is unconfigured, waiting for configure...");
}

WmxEtherCatNode::~WmxEtherCatNode()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node stopped");
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring wmx_ethercat_node...");

  std::string message;
  if (api_->createDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is configured");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  getMasterInfoService_ = this->create_service<wmx_r2_message::srv::EcatGetMasterInfo>(
    "wmx/ecat/get_master_info",
    std::bind(&WmxEtherCatNode::getMasterInfoCallback, this, _1, _2));

  registerReadService_ = this->create_service<wmx_r2_message::srv::EcatRegisterRead>(
    "wmx/ecat/register_read",
    std::bind(&WmxEtherCatNode::registerReadCallback, this, _1, _2));

  resetStatisticsService_ = this->create_service<wmx_r2_message::srv::EcatResetStatistics>(
    "wmx/ecat/reset_statistics",
    std::bind(&WmxEtherCatNode::resetStatisticsCallback, this, _1, _2));

  scanNetworkService_ = this->create_service<wmx_r2_message::srv::EcatScanNetwork>(
    "wmx/ecat/scan_network",
    std::bind(&WmxEtherCatNode::scanNetworkCallback, this, _1, _2));

  startHotconnectService_ = this->create_service<wmx_r2_message::srv::EcatStartHotconnect>(
    "wmx/ecat/start_hotconnect",
    std::bind(&WmxEtherCatNode::startHotconnectCallback, this, _1, _2));

  LifecycleNode::on_activate(previous_state);
  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is active");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_deactivate(previous_state);

  getMasterInfoService_.reset();
  registerReadService_.reset();
  resetStatisticsService_.reset();
  scanNetworkService_.reset();
  startHotconnectService_.reset();

  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is inactive");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  api_->closeDevice();

  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is cleaned up");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(previous_state);
  }

  return on_cleanup(previous_state);
}

void WmxEtherCatNode::getMasterInfoCallback(
  const std::shared_ptr<wmx_r2_message::srv::EcatGetMasterInfo::Request> request,
  std::shared_ptr<wmx_r2_message::srv::EcatGetMasterInfo::Response> response)
{
  auto infoBuffer = std::make_unique<wmx3Api::ecApi::EcMasterInfo>();
  wmx3Api::ecApi::EcMasterInfo & info = *infoBuffer;
  std::string message;

  if (api_->getMasterInfo(request->master_id, info, message) != ErrorCode::None) {
    response->success = false;
    response->message = message;
    return;
  }

  // Master fields
  response->state = static_cast<int32_t>(info.state);
  response->mode = static_cast<int32_t>(info.mode);
  response->comm_period = info.statisticsInfo.commPeriod;
  response->total_axes_num = info.statisticsInfo.totalAxesNum;
  response->total_input_size = info.statisticsInfo.totalInputSize;
  response->total_output_size = info.statisticsInfo.totalOutputSize;
  response->ring_num = info.statisticsInfo.ringNum;
  response->total_rx_pdo_size = info.statisticsInfo.totalRxPdoSize;
  response->total_tx_pdo_size = info.statisticsInfo.totalTxPdoSize;
  response->tx_delay = info.statisticsInfo.txDelay;
  response->min_tx_delay = info.statisticsInfo.minTxDelay;
  response->max_tx_delay = info.statisticsInfo.maxTxDelay;
  response->packet_loss = info.statisticsInfo.packetLoss;
  response->packet_timeout = info.statisticsInfo.packetTimeout;
  response->over_cycle = info.statisticsInfo.overCycle;
  response->num_of_slaves = static_cast<int32_t>(info.numOfSlaves);

  const unsigned int slaveCount = std::min<unsigned int>(
    info.numOfSlaves, wmx3Api::ecApi::constants::maxSlaves);
  if (slaveCount != info.numOfSlaves) {
    RCLCPP_WARN(
      this->get_logger(), "Master reported %u slaves, reading the first %u only",
      info.numOfSlaves, slaveCount);
  }

  // Per-slave arrays
  for (unsigned int i = 0; i < slaveCount; ++i) {
    const wmx3Api::ecApi::EcSlaveInfo & s = info.slaves[i];
    response->slave_ids.push_back(static_cast<int32_t>(s.id));
    response->slave_states.push_back(static_cast<int32_t>(s.state));
    response->slave_al_status_codes.push_back(static_cast<int32_t>(s.alStatusCode));
    response->slave_positions.push_back(static_cast<int32_t>(s.position));
    response->slave_addresses.push_back(static_cast<int32_t>(s.address));
    response->slave_offline.push_back(s.offline);
    response->slave_inaccessible.push_back(s.inaccessible);
    response->slave_new_slaves.push_back(s.newSlave);
    response->slave_reverse_slaves.push_back(s.reverseSlave);
    response->slave_vendor_ids.push_back(s.vendorId);
    response->slave_product_codes.push_back(s.productCode);
    response->slave_revision_nos.push_back(s.revisionNo);
    response->slave_serial_nos.push_back(s.serialNo);
    response->slave_aliases.push_back(s.alias);
    response->slave_input_addrs.push_back(static_cast<int32_t>(s.inputAddr));
    response->slave_input_sizes.push_back(static_cast<int32_t>(s.inputSize));
    response->slave_output_addrs.push_back(static_cast<int32_t>(s.outputAddr));
    response->slave_output_sizes.push_back(static_cast<int32_t>(s.outputSize));
    response->slave_num_of_axes.push_back(static_cast<int32_t>(s.numOfAxes));
  }

  response->success = true;
  response->message = message;
}

void WmxEtherCatNode::registerReadCallback(
  const std::shared_ptr<wmx_r2_message::srv::EcatRegisterRead::Request> request,
  std::shared_ptr<wmx_r2_message::srv::EcatRegisterRead::Response> response)
{
  std::string message;
  response->success =
    api_->registerRead(
    request->master_id, request->slave_id, request->reg_addr, request->len,
    response->data, message) == ErrorCode::None;
  response->message = message;
}

void WmxEtherCatNode::resetStatisticsCallback(
  const std::shared_ptr<wmx_r2_message::srv::EcatResetStatistics::Request> request,
  std::shared_ptr<wmx_r2_message::srv::EcatResetStatistics::Response> response)
{
  std::string message;
  response->success = api_->resetStatistics(request->master_id, message) == ErrorCode::None;
  response->message = message;
}

void WmxEtherCatNode::scanNetworkCallback(
  const std::shared_ptr<wmx_r2_message::srv::EcatScanNetwork::Request> request,
  std::shared_ptr<wmx_r2_message::srv::EcatScanNetwork::Response> response)
{
  std::string message;
  response->success = api_->scanNetwork(request->master_id, message) == ErrorCode::None;
  response->message = message;
}

void WmxEtherCatNode::startHotconnectCallback(
  const std::shared_ptr<wmx_r2_message::srv::EcatStartHotconnect::Request> request,
  std::shared_ptr<wmx_r2_message::srv::EcatStartHotconnect::Response> response)
{
  std::string message;
  response->success = api_->startHotconnect(request->master_id, message) == ErrorCode::None;
  response->message = message;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxEtherCatNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
