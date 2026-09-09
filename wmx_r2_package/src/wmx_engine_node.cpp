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
: logger_(logger), config_(config), cm_(&wmx3Lib_)
{
}

WmxEngineNodeApi::~WmxEngineNodeApi()
{
  std::string message;
  stopCommunication(message);
  stopEngine(message);
}

std::string WmxEngineNodeApi::errorToString(int err)
{
  char errString[256] = {};
  wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
  return errString;
}

bool WmxEngineNodeApi::isEngineStarted()
{
  int deviceId = 0;
  return wmx3Lib_.GetDeviceID(&deviceId) == wmx3Api::ErrorCode::None;
}

bool WmxEngineNodeApi::isEngineCommunicating()
{
  wmx3Api::EngineStatus engineStatus;
  return wmx3Lib_.GetEngineStatus(&engineStatus) == wmx3Api::ErrorCode::None &&
         engineStatus.state == wmx3Api::EngineState::Communicating;
}

int WmxEngineNodeApi::startEngine(std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  if (isEngineStarted()) {
    if (isEngineCommunicating()) {
      message = "Engine is already started and communicating";
      RCLCPP_INFO(logger_, "%s", message.c_str());
      return wmx3Api::ErrorCode::None;
    }

    RCLCPP_INFO(logger_, "Device already created; starting communication only");
    return startCommunication(message);
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
        cm_ = wmx3Api::CoreMotion(&wmx3Lib_);
        RCLCPP_INFO(logger_, "Device created (attempt %d)", attempt + 1);

        if (!config_.wmxParamFilePath.empty()) {
          std::string paramMessage;
          const int paramErr = importAndSetAll(config_.wmxParamFilePath, paramMessage);
          if (paramErr != wmx3Api::ErrorCode::None) {
            message = "Refusing to start communication with default parameters. " + paramMessage;
            RCLCPP_ERROR(logger_, "%s", message.c_str());
            wmx3Lib_.CloseDevice();
            return paramErr;
          }
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
        deviceName_, attempt + 1, maxRetries_, err, errorToString(err).c_str());
    } else {
      RCLCPP_WARN(
        logger_, "Failed to create device (attempt %d/%d). Error=%d (%s)",
        attempt + 1, maxRetries_, err, errorToString(err).c_str());
    }
  }

  message = "Failed to create device after " + std::to_string(maxRetries_) +
    " attempts. Error=" + std::to_string(err) + " (" + errorToString(err) + ")";
  RCLCPP_ERROR(logger_, "%s", message.c_str());
  return err;
}

int WmxEngineNodeApi::stopEngine(std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  if (!isEngineStarted()) {
    message = "Engine is already stopped";
    RCLCPP_INFO(logger_, "%s", message.c_str());
    return wmx3Api::ErrorCode::None;
  }

  int err = wmx3Lib_.CloseDevice();
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to close device. Error=" + std::to_string(err) +
      " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  } else {
    message = "Device closed";
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  err = wmx3Lib_.StopEngine(timeout_);
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to stop engine. Error=" + std::to_string(err) +
      " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  } else {
    message = "Engine stopped";
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  return err;
}

int WmxEngineNodeApi::startCommunication(std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  if (isEngineCommunicating()) {
    message = "Communication is already started";
    RCLCPP_INFO(logger_, "%s", message.c_str());
    return wmx3Api::ErrorCode::None;
  }

  const int err = wmx3Lib_.StartCommunication(timeout_);
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to start communication. Error=" + std::to_string(err) +
      " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  } else {
    message = "Communication started";
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  return err;
}

int WmxEngineNodeApi::stopCommunication(std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  if (!isEngineCommunicating()) {
    message = "Communication is already stopped";
    RCLCPP_INFO(logger_, "%s", message.c_str());
    return wmx3Api::ErrorCode::None;
  }

  const int err = wmx3Lib_.StopCommunication(timeout_);
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to stop communication. Error=" + std::to_string(err) +
      " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  } else {
    message = "Communication stopped";
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  return err;
}

int WmxEngineNodeApi::importAndSetAll(const std::string & path, std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  wmx3Api::Config::SystemParam sysParamErr;
  wmx3Api::Config::AxisParam axisParamErr;

  std::vector<char> pathBuffer(path.begin(), path.end());
  pathBuffer.push_back('\0');

  const int err = cm_.config->ImportAndSetAll(
    pathBuffer.data(), &sysParamErr, &axisParamErr);

  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to load params from " + path + ". Error=" + std::to_string(err) +
      " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Loaded params from: " + path;
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return wmx3Api::ErrorCode::None;
}

int WmxEngineNodeApi::getAxisParam(
  const std::vector<int32_t> & axis, std::vector<std::string> & axisParam, std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  for (int32_t i : axis) {
    if (i < 0 || i >= wmx3Api::constants::maxAxes) {
      message = "Axis " + std::to_string(i) + " is out of range (0.." +
        std::to_string(wmx3Api::constants::maxAxes - 1) + ")";
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      return wmx3Api::ErrorCode::AxisOutOfRange;
    }
  }

  wmx3Api::Config::AxisParam param;
  const int err = cm_.config->GetAxisParam(&param);

  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to read axis params. Error=" + std::to_string(err) +
      " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  axisParam.reserve(axisParam.size() + axis.size() * 6);

  for (int32_t i : axis) {
    axisParam.push_back("=== Axis " + std::to_string(i) + " ===");

    axisParam.push_back("[AxisParam]");
    axisParam.push_back(
      "  GearRatio          = " + std::to_string(param.gearRatioNumerator[i]) +
      " / " + std::to_string(param.gearRatioDenominator[i]));
    axisParam.push_back(
      "  AxisPolarity       = " +
      std::to_string(static_cast<int>(param.axisPolarity[i])));
    axisParam.push_back(
      "  CommandMode        = " +
      std::to_string(static_cast<int>(param.axisCommandMode[i])));
    axisParam.push_back("");
  }

  message = "Read params for " + std::to_string(axis.size()) + " axes";
  return wmx3Api::ErrorCode::None;
}

int WmxEngineNodeApi::getEngineStatus(std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  wmx3Api::EngineStatus engineStatus;

  const int err = wmx3Lib_.GetEngineStatus(&engineStatus);
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to read engine status. Error=" + std::to_string(err) +
      " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = engineStateLabel(engineStatus.state);
  return wmx3Api::ErrorCode::None;
}

WmxEngineNode::WmxEngineNode()
: Node("wmx_engine_node")
{
  WmxEngineNodeApi::Config config;
  config.core = this->declare_parameter<int>("core", -1);
  config.affinityMask = this->declare_parameter<int64_t>("affinity_mask", 0);
  config.wmxParamFilePath = this->declare_parameter<std::string>("wmx_param_file_path", "");

  api_ = std::make_unique<WmxEngineNodeApi>(this->get_logger(), config);

  oneCbOnlyGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  setEngineService_ = this->create_service<std_srvs::srv::SetBool>(
    "wmx/engine/set_engine",
    std::bind(&WmxEngineNode::setEngineCallback, this, _1, _2),
    servicesQos(), oneCbOnlyGroup_);

  setCommunicationService_ = this->create_service<std_srvs::srv::SetBool>(
    "wmx/engine/set_communication",
    std::bind(&WmxEngineNode::setCommunicationCallback, this, _1, _2),
    servicesQos(), oneCbOnlyGroup_);

  getEngineStatusService_ = this->create_service<std_srvs::srv::Trigger>(
    "wmx/engine/get_engine_status",
    std::bind(&WmxEngineNode::getEngineStatusCallback, this, _1, _2),
    servicesQos(), oneCbOnlyGroup_);

  importAndSetAllService_ = this->create_service<wmx_r2_message::srv::ImportAndSetAll>(
    "wmx/engine/import_and_set_all",
    std::bind(&WmxEngineNode::importAndSetAllCallback, this, _1, _2),
    servicesQos(), oneCbOnlyGroup_);

  getAxisParamService_ = this->create_service<wmx_r2_message::srv::GetAxisParam>(
    "wmx/engine/get_axis_param",
    std::bind(&WmxEngineNode::getAxisParamCallback, this, _1, _2),
    servicesQos(), oneCbOnlyGroup_);

  startEngineThread_ = std::thread(
    [this]() {
      std::string message;
      api_->startEngine(message);
    });

  RCLCPP_INFO(this->get_logger(), "wmx_engine_node is ready");
}

WmxEngineNode::~WmxEngineNode()
{
  if (startEngineThread_.joinable()) {
    startEngineThread_.join();
  }
  api_.reset();
  std::this_thread::sleep_for(std::chrono::seconds(3));
  RCLCPP_INFO(this->get_logger(), "wmx_engine_node is stopped");
}

void WmxEngineNode::getEngineStatusCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::string message;
  response->success = api_->getEngineStatus(message) == wmx3Api::ErrorCode::None;
  response->message = message;
}

void WmxEngineNode::setEngineCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::string message;
  const int err = request->data ?
    api_->startEngine(message) : api_->stopEngine(message);

  response->success = (err == wmx3Api::ErrorCode::None);
  response->message = message;
}

void WmxEngineNode::setCommunicationCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::string message;
  const int err = request->data ?
    api_->startCommunication(message) : api_->stopCommunication(message);

  response->success = (err == wmx3Api::ErrorCode::None);
  response->message = message;
}

void WmxEngineNode::importAndSetAllCallback(
  const std::shared_ptr<wmx_r2_message::srv::ImportAndSetAll::Request> request,
  std::shared_ptr<wmx_r2_message::srv::ImportAndSetAll::Response> response)
{
  std::string message;
  response->success =
    api_->importAndSetAll(request->path, message) == wmx3Api::ErrorCode::None;
  response->message = message;
}

void WmxEngineNode::getAxisParamCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetAxisParam::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetAxisParam::Response> response)
{
  std::string message;
  response->success =
    api_->getAxisParam(request->axis, response->axis_param, message) ==
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
