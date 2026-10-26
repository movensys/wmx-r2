// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_r2_control/wmx_system_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

#ifndef WMX3_SDK_PATH
#define WMX3_SDK_PATH "/opt/wmx3/"
#endif

namespace wmx_r2_control
{

using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionAxisStatus;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::EngineState;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;
using wmx3Api::Velocity;

namespace
{
constexpr double kCmdEpsilon = 1e-9;

std::string errorText(int err)
{
  char errString[256] = {};
  CoreMotion::ErrorToString(err, errString, sizeof(errString));
  return errString;
}
}  // namespace

WmxSystemHardwareApi::WmxSystemHardwareApi(
  const rclcpp::Logger & logger, const Config & config)
: logger_(logger), config_(config)
{
}

WmxSystemHardwareApi::~WmxSystemHardwareApi()
{
  releaseDevice();
}

int WmxSystemHardwareApi::attachDevice(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (isDeviceAttached_) {
    message = "Already attached to the WMX3 device";
    return ErrorCode::None;
  }

  int err = ErrorCode::None;
  for (int attempt = 1; attempt <= config_.maxDeviceRetries; ++attempt) {
    err = wmx3Lib_.CreateDevice(
      config_.sdkPath.c_str(), DeviceType::DeviceTypeNormal, timeout_);
    if (err == ErrorCode::None) {
      break;
    }
    if (err != ErrorCode::StartProcessLockError) {
      message = "Failed to attach to device. Error=" + std::to_string(err) +
        " (" + errorText(err) + ")";
      RCLCPP_FATAL(logger_, "%s", message.c_str());
      return err;
    }
    RCLCPP_WARN(
      logger_, "WMX device lock busy, retrying in 1s... (%d/%d)",
      attempt, config_.maxDeviceRetries);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  if (err != ErrorCode::None) {
    message = "WMX device lock busy after " + std::to_string(config_.maxDeviceRetries) +
      " retries, giving up";
    RCLCPP_FATAL(logger_, "%s", message.c_str());
    return err;
  }

  err = wmx3Lib_.SetDeviceName(config_.deviceName.c_str());
  if (err != ErrorCode::None) {
    message = "Failed to name the device '" + config_.deviceName + "'. Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    // The device exists but is unnamed: close it so the next attempt starts clean.
    wmx3Lib_.CloseDevice();
    return err;
  }

  cm_ = CoreMotion(&wmx3Lib_);
  isDeviceAttached_ = true;

  message = "Attached to WMX3 device as '" + config_.deviceName + "'";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void WmxSystemHardwareApi::releaseDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!isDeviceAttached_) {
    return;
  }

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(
      logger_, "Failed to close WMX device. Error=%d (%s)", err, errorText(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "WMX device closed");
  }
  isDeviceAttached_ = false;
}

int WmxSystemHardwareApi::importAndSetAll(const std::string & path, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!isDeviceAttached_) {
    message = "Cannot import WMX params. Device is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm_.config->ImportAndSetAll(const_cast<char *>(path.c_str()));
  if (err != ErrorCode::None) {
    message = "Failed to import WMX params from " + path + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Applied WMX params from " + path;
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxSystemHardwareApi::getStatus(
  const std::vector<int> & axes, std::vector<AxisFeedback> & feedback,
  bool & communicating, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  communicating = false;
  feedback.clear();

  if (!isDeviceAttached_) {
    message = "Cannot read the axis status. Device is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  CoreMotionStatus status;
  const int err = cm_.GetStatus(&status);
  if (err != ErrorCode::None) {
    message = "GetStatus failed. Error=" + std::to_string(err) + " (" + errorText(err) + ")";
    return err;
  }

  feedback.reserve(axes.size());
  for (const int axis : axes) {
    if (axis < 0 || axis >= wmx3Api::constants::maxAxes) {
      message = "Invalid axis " + std::to_string(axis) + ": must be in [0, " +
        std::to_string(wmx3Api::constants::maxAxes) + ").";
      feedback.clear();
      return ErrorCode::ArgumentOutOfRange;
    }
    const CoreMotionAxisStatus & raw = status.axesStatus[axis];
    feedback.push_back({raw.actualPos, raw.actualVelocity, raw.servoOn, raw.ampAlarm});
  }

  communicating = status.engineState == EngineState::T::Communicating;
  return ErrorCode::None;
}

int WmxSystemHardwareApi::startVel(int axis, double omega, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!isDeviceAttached_) {
    message = "Cannot move axis " + std::to_string(axis) + ". Device is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  Velocity::VelCommand command;
  command.axis = axis;
  command.profile.velocity = omega;
  command.profile.type = ProfileType::T::TimeAccTrapezoidal;
  command.profile.accTimeMilliseconds = config_.accTimeMilliseconds;
  command.profile.decTimeMilliseconds = config_.decTimeMilliseconds;

  const int err = cm_.velocity->StartVel(&command);
  if (err != ErrorCode::None) {
    message = "StartVel failed on axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    return err;
  }

  message = "Axis " + std::to_string(axis) + " running at " + std::to_string(omega);
  return ErrorCode::None;
}

int WmxSystemHardwareApi::setServoOn(int axis, int on, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!isDeviceAttached_) {
    message = "Cannot set servo on axis " + std::to_string(axis) + ". Device is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm_.axisControl->SetServoOn(axis, on, servoOnTimeout_);
  if (err != ErrorCode::None) {
    message = "Failed to servo-" + std::string(on ? "on" : "off") + " axis " +
      std::to_string(axis) + ". Error=" + std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Servo " + std::to_string(axis) + " " + (on ? "on" : "off");
  return ErrorCode::None;
}

int WmxSystemHardwareApi::clearAmpAlarm(int axis, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!isDeviceAttached_) {
    message = "Cannot clear the alarm on axis " + std::to_string(axis) +
      ". Device is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm_.axisControl->ClearAmpAlarm(axis);
  if (err != ErrorCode::None) {
    message = "Failed to clear the amp alarm on axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_WARN(logger_, "%s", message.c_str());
    return err;
  }

  message = "Amp alarm cleared on axis " + std::to_string(axis);
  return ErrorCode::None;
}

std::string WmxSystemHardware::getHwParam(
  const std::string & key, const std::string & def) const
{
  auto it = info_.hardware_parameters.find(key);
  return (it != info_.hardware_parameters.end()) ? it->second : def;
}

#if WMX_HAS_HW_COMPONENT_INTERFACE_PARAMS
hardware_interface::CallbackReturn WmxSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return initImpl();
}
#else
hardware_interface::CallbackReturn WmxSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return initImpl();
}
#endif

hardware_interface::CallbackReturn WmxSystemHardware::initImpl()
{
  WmxSystemHardwareApi::Config config;
  config.sdkPath = getHwParam("wmx_sdk_path", WMX3_SDK_PATH);
  config.deviceName = getHwParam("device_name", "wmx_r2_control");
  config.accTimeMilliseconds = std::stod(getHwParam("acc_time_ms", "1.0"));
  config.decTimeMilliseconds = std::stod(getHwParam("dec_time_ms", "1.0"));
  config.maxDeviceRetries = std::stoi(getHwParam("max_device_retries", "30"));

  wmxParamFile_ = getHwParam("wmx_param_file", "");
  maxDeviceRetries_ = config.maxDeviceRetries;
  autoServoOn_ = (getHwParam("auto_servo_on", "true") == "true");

  api_ = std::make_unique<WmxSystemHardwareApi>(logger_, config);

  joints_.clear();
  joints_.reserve(info_.joints.size());
  axes_.clear();
  axes_.reserve(info_.joints.size());

  for (const auto & j : info_.joints) {
    WmxJoint joint;
    joint.name = j.name;

    auto axis_it = j.parameters.find("axis");
    if (axis_it == j.parameters.end()) {
      RCLCPP_FATAL(
        logger_, "Joint '%s' is missing required <param name=\"axis\">", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint.axis = std::stoi(axis_it->second);

    if (joint.axis < 0 || joint.axis >= wmx3Api::constants::maxAxes) {
      RCLCPP_FATAL(
        logger_, "Joint '%s' axis %d is out of range [0, %d)",
        j.name.c_str(), joint.axis, wmx3Api::constants::maxAxes);
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto & cmd_ifs = j.command_interfaces;
    if (cmd_ifs.empty()) {
      joint.mode = JointMode::StateOnly;
    } else if (cmd_ifs.size() == 1 && cmd_ifs[0].name == hardware_interface::HW_IF_VELOCITY) {
      joint.mode = JointMode::Velocity;
    } else {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s' must declare either no command_interface (state-only) or a "
        "single 'velocity' command_interface", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      logger_, "Joint '%s' -> WMX axis %d, mode=%s",
      joint.name.c_str(), joint.axis,
      joint.mode == JointMode::Velocity ? "velocity" : "state-only");

    axes_.push_back(joint.axis);
    joints_.push_back(joint);
  }

  RCLCPP_INFO(
    logger_,
    "WmxSystemHardware initialised: %zu joints, sdk_path=%s, param_file=%s",
    joints_.size(), config.sdkPath.c_str(),
    wmxParamFile_.empty() ? "(none)" : wmxParamFile_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool WmxSystemHardware::waitForCommunicating()
{
  for (int attempt = 1; attempt <= maxDeviceRetries_; ++attempt) {
    std::string message;
    bool communicating = false;
    if (api_->getStatus(axes_, feedback_, communicating, message) == ErrorCode::None &&
      communicating)
    {
      return true;
    }
    RCLCPP_WARN(
      logger_, "WMX engine not Communicating yet, waiting... (%d/%d)",
      attempt, maxDeviceRetries_);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return false;
}

void WmxSystemHardware::seedJointStates()
{
  std::string message;
  if (api_->getStatus(axes_, feedback_, communicating_, message) != ErrorCode::None) {
    RCLCPP_WARN(logger_, "Could not seed joint states: %s", message.c_str());
    return;
  }

  for (size_t i = 0; i < joints_.size() && i < feedback_.size(); ++i) {
    joints_[i].posState = feedback_[i].actualPos;
    joints_[i].velState = feedback_[i].actualVelocity;
    joints_[i].cmd = 0.0;
    joints_[i].lastCmd = std::numeric_limits<double>::quiet_NaN();
  }
}

hardware_interface::CallbackReturn WmxSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string message;
  if (api_->attachDevice(message) != ErrorCode::None) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!wmxParamFile_.empty()) {
    // Informational: a bad parameter file is reported but does not stop the
    // component, matching the previous behaviour.
    api_->importAndSetAll(wmxParamFile_, message);
  }

  seedJointStates();

  RCLCPP_INFO(logger_, "WmxSystemHardware configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!waitForCommunicating()) {
    RCLCPP_ERROR(
      logger_,
      "WMX engine is not Communicating after %d s. Start wmx_engine_node and "
      "EtherCAT communication first.", maxDeviceRetries_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (autoServoOn_) {
    std::string message;
    for (const WmxJoint & joint : joints_) {
      api_->clearAmpAlarm(joint.axis, message);
    }
    for (const WmxJoint & joint : joints_) {
      if (api_->setServoOn(joint.axis, 1, message) != ErrorCode::None) {
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    RCLCPP_INFO(logger_, "Servos enabled on all axes");
  }

  seedJointStates();

  RCLCPP_INFO(logger_, "WmxSystemHardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string message;

  for (const WmxJoint & joint : joints_) {
    if (joint.mode == JointMode::Velocity) {
      api_->startVel(joint.axis, 0.0, message);
    }
  }

  if (autoServoOn_) {
    for (const WmxJoint & joint : joints_) {
      api_->setServoOn(joint.axis, 0, message);
    }
  }

  RCLCPP_INFO(logger_, "WmxSystemHardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  api_->releaseDevice();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WmxSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & joint : joints_) {
    state_interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_POSITION, &joint.posState);
    state_interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velState);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WmxSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & joint : joints_) {
    if (joint.mode == JointMode::Velocity) {
      command_interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_VELOCITY, &joint.cmd);
    }
  }
  return command_interfaces;
}

hardware_interface::return_type WmxSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::string message;
  if (api_->getStatus(axes_, feedback_, communicating_, message) != ErrorCode::None) {
    RCLCPP_ERROR_THROTTLE(logger_, clock_, 1000, "%s", message.c_str());
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joints_.size() && i < feedback_.size(); ++i) {
    joints_[i].posState = feedback_[i].actualPos;
    joints_[i].velState = feedback_[i].actualVelocity;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WmxSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!communicating_) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 1000, "WMX engine not Communicating; skipping write()");
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < joints_.size() && i < feedback_.size(); ++i) {
    WmxJoint & joint = joints_[i];
    if (joint.mode != JointMode::Velocity) {
      continue;
    }

    const WmxSystemHardwareApi::AxisFeedback & axisStatus = feedback_[i];
    if (!axisStatus.servoOn || axisStatus.ampAlarm) {
      RCLCPP_WARN_THROTTLE(
        logger_, clock_, 2000,
        "Axis %d not ready (servoOn=%d, ampAlarm=%d); skipping velocity command",
        joint.axis, axisStatus.servoOn, axisStatus.ampAlarm);
      joint.lastCmd = std::numeric_limits<double>::quiet_NaN();
      continue;
    }

    if (std::isnan(joint.lastCmd) || std::fabs(joint.cmd - joint.lastCmd) > kCmdEpsilon) {
      std::string message;
      if (api_->startVel(joint.axis, joint.cmd, message) != ErrorCode::None) {
        RCLCPP_ERROR_THROTTLE(logger_, clock_, 1000, "%s", message.c_str());
      }
      joint.lastCmd = joint.cmd;
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace wmx_r2_control

PLUGINLIB_EXPORT_CLASS(
  wmx_r2_control::WmxSystemHardware, hardware_interface::SystemInterface)
