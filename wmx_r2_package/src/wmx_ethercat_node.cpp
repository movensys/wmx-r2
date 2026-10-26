// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_ethercat_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::ecApi::Ecat;

namespace
{
std::string ecatErrorText(int err)
{
  char errString[256] = {};
  Ecat::ErrorToString(err, errString, sizeof(errString));
  return errString;
}

std::string failureText(const std::string & call, const std::string & where, int err)
{
  return call + " failed. " + where + " Error=" + std::to_string(err) +
         " (" + ecatErrorText(err) + ")";
}
}  // namespace

WmxEtherCatNodeApi::WmxEtherCatNodeApi(const rclcpp::Logger & logger)
: logger_(logger)
{
}

WmxEtherCatNodeApi::~WmxEtherCatNodeApi()
{
  if (wmxEcat_) {
    releaseDevice();
  }
}

int WmxEtherCatNodeApi::attachDevice(std::string & message)
{
  if (wmxEcat_) {
    message = "Already attached to the WMX3 device";
    return ErrorCode::None;
  }

  const int err = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout_);
  if (err != ErrorCode::None) {
    if (err == ErrorCode::StartProcessLockError) {
      message = "Failed to attach to device (lock busy, will retry on next signal).";
      RCLCPP_WARN(logger_, "%s", message.c_str());
    } else {
      char errString[256] = {};
      wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
      message = "Failed to attach to device. Error=" + std::to_string(err) +
        " (" + errString + ")";
      RCLCPP_ERROR(logger_, "%s", message.c_str());
    }
    return err;
  }

  wmx3Lib_.SetDeviceName(deviceName_);
  wmxEcat_ = std::make_unique<Ecat>(&wmx3Lib_);

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void WmxEtherCatNodeApi::releaseDevice()
{
  wmxEcat_.reset();

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    char errString[256] = {};
    wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errString);
  } else {
    RCLCPP_INFO(logger_, "Device closed");
  }
}

int WmxEtherCatNodeApi::getMasterInfo(
  int32_t masterId, wmx3Api::ecApi::EcMasterInfo & info, std::string & message)
{
  if (!wmxEcat_) {
    message = "Cannot read master info. EtherCAT is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = wmxEcat_->GetMasterInfo(masterId, &info);
  if (err != ErrorCode::None) {
    message = failureText("GetMasterInfo", "masterId=" + std::to_string(masterId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Master " + std::to_string(masterId) + ": slaves=" + std::to_string(info.numOfSlaves);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxEtherCatNodeApi::registerRead(
  int32_t masterId, int32_t slaveId, int32_t regAddress, int32_t length,
  std::vector<uint8_t> & data, std::string & message)
{
  if (!wmxEcat_) {
    message = "Cannot read register. EtherCAT is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  if (regAddress < 0 || regAddress > 0xFFF) {
    message = "Invalid reg_address: must be in [0x000, 0xFFF].";
    return ErrorCode::ArgumentOutOfRange;
  }

  if (length <= 0 || length > 0x1000) {
    message = "Invalid length: must be in [1, 4096].";
    return ErrorCode::ArgumentOutOfRange;
  }

  if (regAddress + length > 0x1000) {
    message = "reg_address + length exceeds 0x1000.";
    return ErrorCode::ArgumentOutOfRange;
  }

  std::vector<unsigned char> raw(length, 0);
  const int err = wmxEcat_->RegisterRead(masterId, slaveId, regAddress, length, raw.data());

  char where[128] = {};
  snprintf(
    where, sizeof(where), "slaveId=%d reg=0x%03X length=%d", slaveId, regAddress, length);

  if (err != ErrorCode::None) {
    message = failureText("RegisterRead", where, err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  data.assign(raw.begin(), raw.end());
  message = "RegisterRead success. " + std::string(where);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxEtherCatNodeApi::resetStatistics(int32_t masterId, std::string & message)
{
  if (!wmxEcat_) {
    message = "Cannot reset statistics. EtherCAT is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  int err = wmxEcat_->ResetRefClockInfo(masterId);
  if (err != ErrorCode::None) {
    message = failureText("ResetRefClockInfo", "masterId=" + std::to_string(masterId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = wmxEcat_->ResetTransmitStatisticsInfo(masterId);
  if (err != ErrorCode::None) {
    message = failureText(
      "ResetTransmitStatisticsInfo", "masterId=" + std::to_string(masterId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  wmxEcat_->ScanNetwork(masterId);

  message = "Statistics reset and ScanNetwork done. masterId=" + std::to_string(masterId);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxEtherCatNodeApi::scanNetwork(int32_t masterId, std::string & message)
{
  if (!wmxEcat_) {
    message = "Cannot scan network. EtherCAT is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = wmxEcat_->ScanNetwork(masterId);
  if (err != ErrorCode::None) {
    message = failureText("ScanNetwork", "masterId=" + std::to_string(masterId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "ScanNetwork done. masterId=" + std::to_string(masterId);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxEtherCatNodeApi::startHotconnect(int32_t masterId, std::string & message)
{
  if (!wmxEcat_) {
    message = "Cannot start hotconnect. EtherCAT is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = wmxEcat_->StartHotconnect(masterId);
  if (err != ErrorCode::None) {
    message = failureText("StartHotconnect", "masterId=" + std::to_string(masterId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "StartHotconnect done. masterId=" + std::to_string(masterId);
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

bool WmxEtherCatNode::isNodeActive()
{
  return this->get_current_state().id() ==
         lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

std::string WmxEtherCatNode::notActiveMessage()
{
  return "wmx_ethercat_node is not active (state: " +
         this->get_current_state().label() + ").";
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring wmx_ethercat_node...");

  std::string message;
  if (api_->attachDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  getNetworkStateService_ = this->create_service<wmx_r2_message::srv::EcatGetNetworkState>(
    "wmx/ecat/get_network_state",
    std::bind(&WmxEtherCatNode::getNetworkStateCallback, this, _1, _2));

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

  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is configured");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is active");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is inactive");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  getNetworkStateService_.reset();
  registerReadService_.reset();
  resetStatisticsService_.reset();
  scanNetworkService_.reset();
  startHotconnectService_.reset();

  if (api_->isDeviceOpen()) {
    api_->releaseDevice();
  }

  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is cleaned up");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void WmxEtherCatNode::getNetworkStateCallback(
  const std::shared_ptr<wmx_r2_message::srv::EcatGetNetworkState::Request> request,
  std::shared_ptr<wmx_r2_message::srv::EcatGetNetworkState::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  wmx3Api::ecApi::EcMasterInfo info;
  std::string message;

  if (api_->getMasterInfo(request->master_id, info, message) != ErrorCode::None) {
    response->success = false;
    response->message = message;
    return;
  }

  // Master fields
  response->master_state = static_cast<int32_t>(info.state);
  response->master_mode = static_cast<int32_t>(info.mode);
  response->comm_period = info.statisticsInfo.commPeriod;
  response->total_axes = info.statisticsInfo.totalAxesNum;
  response->total_input_size = info.statisticsInfo.totalInputSize;
  response->total_output_size = info.statisticsInfo.totalOutputSize;
  response->ring_num = info.statisticsInfo.ringNum;
  response->total_rxpdo_size = info.statisticsInfo.totalRxPdoSize;
  response->total_txpdo_size = info.statisticsInfo.totalTxPdoSize;
  response->tx_delay = info.statisticsInfo.txDelay;
  response->min_tx_delay = info.statisticsInfo.minTxDelay;
  response->max_tx_delay = info.statisticsInfo.maxTxDelay;
  response->packet_loss = info.statisticsInfo.packetLoss;
  response->packet_timeout = info.statisticsInfo.packetTimeout;
  response->over_cycle = info.statisticsInfo.overCycle;
  response->num_of_slaves = static_cast<int32_t>(info.numOfSlaves);

  // Per-slave arrays
  for (unsigned int i = 0; i < info.numOfSlaves; ++i) {
    const wmx3Api::ecApi::EcSlaveInfo & s = info.slaves[i];
    response->slave_ids.push_back(static_cast<int32_t>(s.id));
    response->slave_states.push_back(static_cast<int32_t>(s.state));
    response->slave_al_codes.push_back(static_cast<int32_t>(s.alStatusCode));
    response->slave_positions.push_back(static_cast<int32_t>(s.position));
    response->slave_addresses.push_back(static_cast<int32_t>(s.address));
    response->slave_offline.push_back(s.offline);
    response->slave_inaccessible.push_back(s.inaccessible);
    response->slave_new.push_back(s.newSlave);
    response->slave_reverse.push_back(s.reverseSlave);
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
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::vector<uint8_t> data;
  std::string message;
  const int err = api_->registerRead(
    request->master_id, request->slave_id, request->reg_address, request->length, data, message);

  response->success = (err == ErrorCode::None);
  response->data = data;
  response->message = message;
}

void WmxEtherCatNode::resetStatisticsCallback(
  const std::shared_ptr<wmx_r2_message::srv::EcatResetStatistics::Request> request,
  std::shared_ptr<wmx_r2_message::srv::EcatResetStatistics::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  const int err = api_->resetStatistics(request->master_id, message);

  response->success = (err == ErrorCode::None);
  response->message = message;
}

void WmxEtherCatNode::scanNetworkCallback(
  const std::shared_ptr<wmx_r2_message::srv::EcatScanNetwork::Request> request,
  std::shared_ptr<wmx_r2_message::srv::EcatScanNetwork::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  const int err = api_->scanNetwork(request->master_id, message);

  response->success = (err == ErrorCode::None);
  response->message = message;
}

void WmxEtherCatNode::startHotconnectCallback(
  const std::shared_ptr<wmx_r2_message::srv::EcatStartHotconnect::Request> request,
  std::shared_ptr<wmx_r2_message::srv::EcatStartHotconnect::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  const int err = api_->startHotconnect(request->master_id, message);

  response->success = (err == ErrorCode::None);
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
