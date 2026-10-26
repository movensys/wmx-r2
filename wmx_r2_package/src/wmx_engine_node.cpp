// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_engine_node.hpp"

#include <cinttypes>

#include "wmx_qos_compat.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
const char * engineStateLabel(wmx3Api::EngineState::T state)
{
  switch (state) {
    case wmx3Api::EngineState::Idle:          return "Idle";
    case wmx3Api::EngineState::Running:       return "Running";
    case wmx3Api::EngineState::Communicating: return "Communicating";
    case wmx3Api::EngineState::Shutdown:      return "Shutdown";
    case wmx3Api::EngineState::Unknown:       return "Unknown";
    default:                                  return "Invalid";
  }
}
}  // namespace

WmxEngineNodeApi::WmxEngineNodeApi(const rclcpp::Logger & logger, const Config & config)
: logger_(logger), config_(config)
{
}

WmxEngineNodeApi::~WmxEngineNodeApi()
{
  std::string message;
  stopCommunication(message);
  stopEngine(message);
}

std::string WmxEngineNodeApi::errorText(int err)
{
  char errString[256] = {};
  wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
  return errString;
}

int WmxEngineNodeApi::startEngine(std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  wmx3Api::EngineState::T state = wmx3Api::EngineState::Unknown;
  engineState(state);

  if (state == wmx3Api::EngineState::Running ||
    state == wmx3Api::EngineState::Communicating)
  {
    message = "Engine is already started (state '" + std::string(engineStateLabel(state)) + "')";
    RCLCPP_WARN(logger_, "%s", message.c_str());
    return wmx3Api::ErrorCode::None;
  }

  RCLCPP_INFO(
    logger_, "Starting engine... (core=%d, affinityMask=0x%" PRIx64 ")",
    config_.core, config_.affinityMask);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  int err = wmx3Api::ErrorCode::None;

  for (int attempt = 0; attempt < maxRetries_; attempt++) {
    if (attempt > 0) {
      RCLCPP_INFO(
        logger_, "Retrying device creation (attempt %d/%d)...", attempt + 1, maxRetries_);
      std::this_thread::sleep_for(std::chrono::milliseconds(retryDelay_));
    }

    err = wmx3Lib_.CreateDevice(
      WMX3_SDK_PATH, wmx3Api::DeviceType::DeviceTypeNormal, timeout_,
      config_.core, config_.affinityMask);

    if (err == wmx3Api::ErrorCode::None) {
      err = wmx3Lib_.SetDeviceName(deviceName_);

      if (err == wmx3Api::ErrorCode::None) {
        isEngineStarted_ = true;

        cm_ = std::make_unique<wmx3Api::CoreMotion>(&wmx3Lib_);
        deviceOpen_ = true;
        RCLCPP_INFO(logger_, "Device created (attempt %d)", attempt + 1);

        if (!config_.paramFilePath.empty()) {
          std::string paramMessage;
          loadWmxParams(config_.paramFilePath, paramMessage);
        }

        return startCommunication(message);
      }

      wmx3Lib_.CloseDevice();
    }

    if (err == wmx3Api::ErrorCode::CreateDeviceLockError) {
      RCLCPP_WARN(
        logger_, "Device lock error (attempt %d/%d). Waiting...", attempt + 1, maxRetries_);
    } else if (err == wmx3Api::ErrorCode::SetDeviceNameTimeout) {
      RCLCPP_WARN(
        logger_, "Failed to name the device '%s' (attempt %d/%d). Error=%d (%s)",
        deviceName_, attempt + 1, maxRetries_, err, errorText(err).c_str());
    } else {
      RCLCPP_WARN(
        logger_, "Failed to create device (attempt %d/%d). Error=%d (%s)",
        attempt + 1, maxRetries_, err, errorText(err).c_str());
    }
  }

  message = "Failed to create device after " + std::to_string(maxRetries_) +
    " attempts. Error=" + std::to_string(err) + " (" + errorText(err) + ")";
  RCLCPP_ERROR(logger_, "%s", message.c_str());
  return err;
}

int WmxEngineNodeApi::stopEngine(std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  cm_.reset();
  deviceOpen_ = false;

  int err = wmx3Lib_.CloseDevice();
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to close device. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  } else {
    message = "Device closed";
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  err = wmx3Lib_.StopEngine(timeout_);
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to stop engine. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  } else {
    message = "Engine stopped";
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  isCommStarted_ = false;
  isEngineStarted_ = false;

  return err;
}

int WmxEngineNodeApi::startCommunication(std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  const int err = wmx3Lib_.StartCommunication(timeout_);
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to start communication. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  } else {
    isCommStarted_ = true;
    message = "Communication started";
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  return err;
}

int WmxEngineNodeApi::stopCommunication(std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Communication is not started";
    return wmx3Api::ErrorCode::None;
  }

  const int err = wmx3Lib_.StopCommunication(timeout_);
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to stop communication. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  } else {
    message = "Communication stopped";
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  isCommStarted_ = false;

  return err;
}

int WmxEngineNodeApi::loadWmxParams(const std::string & path, std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot load params. Core motion is not exist.";
    return wmx3Api::ErrorCode::DeviceIsNull;
  }

  wmx3Api::Config::SystemParam sysParamErr;
  wmx3Api::Config::AxisParam axisParamErr;

  std::vector<char> pathBuffer(path.begin(), path.end());
  pathBuffer.push_back('\0');

  const int err = cm_->config->ImportAndSetAll(
    pathBuffer.data(), &sysParamErr, &axisParamErr);

  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to load params from " + path + ". Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Loaded params from: " + path;
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return wmx3Api::ErrorCode::None;
}

int WmxEngineNodeApi::getWmxParams(
  const std::vector<int32_t> & axes, std::vector<std::string> & dump, std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot get params. Core motion is not exist.";
    return wmx3Api::ErrorCode::DeviceIsNull;
  }

  for (int32_t i : axes) {
    if (i < 0 || i >= wmx3Api::constants::maxAxes) {
      message = "Axis " + std::to_string(i) + " is out of range (0.." +
        std::to_string(wmx3Api::constants::maxAxes - 1) + ")";
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      return wmx3Api::ErrorCode::AxisOutOfRange;
    }
  }

  wmx3Api::Config::AxisParam axisParam;
  const int err = cm_->config->GetAxisParam(&axisParam);

  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to read axis params. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  dump.reserve(dump.size() + axes.size() * 6);

  for (int32_t i : axes) {
    dump.push_back("=== Axis " + std::to_string(i) + " ===");

    dump.push_back("[AxisParam]");
    dump.push_back(
      "  GearRatio          = " + std::to_string(axisParam.gearRatioNumerator[i]) +
      " / " + std::to_string(axisParam.gearRatioDenominator[i]));
    dump.push_back(
      "  AxisPolarity       = " +
      std::to_string(static_cast<int>(axisParam.axisPolarity[i])));
    dump.push_back(
      "  CommandMode        = " +
      std::to_string(static_cast<int>(axisParam.axisCommandMode[i])));
    dump.push_back("");
  }

  message = "Read params for " + std::to_string(axes.size()) + " axes";
  return wmx3Api::ErrorCode::None;
}

int WmxEngineNodeApi::getEngineStatus(std::string & message)
{
  wmx3Api::EngineState::T state = wmx3Api::EngineState::Unknown;

  const int err = engineState(state);
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to read engine status. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = engineStateLabel(state);
  return wmx3Api::ErrorCode::None;
}

int WmxEngineNodeApi::engineState(wmx3Api::EngineState::T & state)
{
  wmx3Api::EngineStatus engineStatus;

  const int err = wmx3Lib_.GetEngineStatus(&engineStatus);
  if (err != wmx3Api::ErrorCode::None) {
    return err;
  }

  state = engineStatus.state;
  return wmx3Api::ErrorCode::None;
}

WmxEngineNode::WmxEngineNode()
: Node("wmx_engine_node")
{
  WmxEngineNodeApi::Config config;
  config.core = this->declare_parameter<int>("engine_core", -1);
  config.affinityMask = this->declare_parameter<int64_t>("engine_affinity_mask", 0);
  config.paramFilePath = this->declare_parameter<std::string>("wmx_param_file_path", "");

  api_ = std::make_unique<WmxEngineNodeApi>(this->get_logger(), config);

  managerCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  setEngineService_ = this->create_service<std_srvs::srv::SetBool>(
    "wmx/engine/set_engine",
    std::bind(&WmxEngineNode::setEngineCallback, this, _1, _2),
    servicesQos(), managerCbGroup_);

  setCommService_ = this->create_service<std_srvs::srv::SetBool>(
    "wmx/engine/set_comm",
    std::bind(&WmxEngineNode::setCommCallback, this, _1, _2),
    servicesQos(), managerCbGroup_);

  getEngineStatusService_ = this->create_service<std_srvs::srv::Trigger>(
    "wmx/engine/get_engine_status",
    std::bind(&WmxEngineNode::getEngineStatusCallback, this, _1, _2));

  loadWmxParamsService_ = this->create_service<wmx_r2_message::srv::LoadWmxParams>(
    "wmx/engine/load_wmx_params",
    std::bind(&WmxEngineNode::loadWmxParamsCallback, this, _1, _2),
    servicesQos(), managerCbGroup_);

  getWmxParamsService_ = this->create_service<wmx_r2_message::srv::GetWmxParams>(
    "wmx/engine/get_wmx_params",
    std::bind(&WmxEngineNode::getWmxParamsCallback, this, _1, _2),
    servicesQos(), managerCbGroup_);

  startThread_ = std::thread(
    [this]() {
      std::string message;
      api_->startEngine(message);
    });

  RCLCPP_INFO(this->get_logger(), "wmx_engine_node is ready");
}

WmxEngineNode::~WmxEngineNode()
{
  if (startThread_.joinable()) {
    startThread_.join();
  }
  api_.reset();
  std::this_thread::sleep_for(std::chrono::seconds(3));
  RCLCPP_INFO(this->get_logger(), "wmx_engine_node is stopped");
}

void WmxEngineNode::getEngineStatusCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!api_->isEngineStarted()) {
    response->success = false;
    response->message = "Engine is not started yet";
    return;
  }

  std::string message;
  response->success = api_->getEngineStatus(message) == wmx3Api::ErrorCode::None;
  response->message = message;
}

void WmxEngineNode::setEngineCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::string message;
  int err;

  if (request->data) {
    if (api_->isDeviceOpen()) {
      response->success = false;
      response->message = "Engine is already started";
      return;
    }
    err = api_->startEngine(message);
  } else {
    if (!api_->isDeviceOpen()) {
      response->success = false;
      response->message = "Engine is not started";
      return;
    }
    err = api_->stopEngine(message);
  }

  response->success = (err == wmx3Api::ErrorCode::None);
  response->message = message;
}

void WmxEngineNode::setCommCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!api_->isEngineStarted()) {
    response->success = false;
    response->message = "Engine is not started yet";
    return;
  }

  std::string message;
  const int err = request->data ?
    api_->startCommunication(message) : api_->stopCommunication(message);

  response->success = (err == wmx3Api::ErrorCode::None);
  response->message = message;
}

void WmxEngineNode::loadWmxParamsCallback(
  const std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Request> request,
  std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Response> response)
{
  if (!api_->isEngineStarted()) {
    response->success = false;
    response->message = "Engine is not started yet";
    return;
  }

  std::string message;
  response->success =
    api_->loadWmxParams(request->file_path, message) == wmx3Api::ErrorCode::None;
  response->message = message;
}

void WmxEngineNode::getWmxParamsCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Response> response)
{
  if (!api_->isDeviceOpen()) {
    response->success = false;
    response->message = "Cannot get params. Engine or core motion are not exist.";
    return;
  }

  std::string message;
  response->success =
    api_->getWmxParams(request->indices, response->params_dump, message) ==
    wmx3Api::ErrorCode::None;
  response->message = message;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxEngineNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
