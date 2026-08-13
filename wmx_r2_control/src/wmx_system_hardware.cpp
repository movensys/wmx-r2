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
using wmx3Api::DeviceType;
using wmx3Api::EngineState;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;
using wmx3Api::Velocity;

namespace
{
constexpr double kCmdEpsilon = 1e-9;
}  // namespace

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
  sdk_path_ = getHwParam("wmx_sdk_path", WMX3_SDK_PATH);
  device_name_ = getHwParam("device_name", "wmx_r2_control");
  wmx_param_file_ = getHwParam("wmx_param_file", "");
  acc_time_ms_ = std::stod(getHwParam("acc_time_ms", "1.0"));
  dec_time_ms_ = std::stod(getHwParam("dec_time_ms", "1.0"));
  max_device_retries_ = std::stoi(getHwParam("max_device_retries", "30"));
  auto_servo_on_ = (getHwParam("auto_servo_on", "true") == "true");

  joints_.clear();
  joints_.reserve(info_.joints.size());

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

    joints_.push_back(joint);
  }

  RCLCPP_INFO(
    logger_,
    "WmxSystemHardware initialised: %zu joints, sdk_path=%s, param_file=%s",
    joints_.size(), sdk_path_.c_str(),
    wmx_param_file_.empty() ? "(none)" : wmx_param_file_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool WmxSystemHardware::attachDevice()
{
  const unsigned int timeout = 10000;
  int err = ErrorCode::None;

  for (int attempt = 1; attempt <= max_device_retries_; ++attempt) {
    err = wmx_.CreateDevice(sdk_path_.c_str(), DeviceType::DeviceTypeNormal, timeout);
    if (err == ErrorCode::None) {
      break;
    }
    wmx_.ErrorToString(err, err_str_, sizeof(err_str_));
    if (err == ErrorCode::StartProcessLockError) {
      RCLCPP_WARN(
        logger_, "WMX device lock busy, retrying in 1s... (%d/%d)", attempt, max_device_retries_);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      RCLCPP_FATAL(logger_, "Failed to attach to WMX device. Error=%d (%s)", err, err_str_);
      return false;
    }
  }

  if (err != ErrorCode::None) {
    RCLCPP_FATAL(
      logger_, "WMX device lock busy after %d retries, giving up", max_device_retries_);
    return false;
  }

  wmx_.SetDeviceName(device_name_.c_str());
  device_open_ = true;
  RCLCPP_INFO(logger_, "Attached to WMX3 device as '%s'", device_name_.c_str());
  return true;
}

void WmxSystemHardware::closeDevice()
{
  if (!device_open_) {
    return;
  }
  int err = wmx_.CloseDevice();
  if (err != ErrorCode::None) {
    wmx_.ErrorToString(err, err_str_, sizeof(err_str_));
    RCLCPP_ERROR(logger_, "Failed to close WMX device. Error=%d (%s)", err, err_str_);
  } else {
    RCLCPP_INFO(logger_, "WMX device closed");
  }
  device_open_ = false;
}

bool WmxSystemHardware::engineCommunicating()
{
  cm_->GetStatus(&cm_status_);
  return cm_status_.engineState == EngineState::T::Communicating;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!attachDevice()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cm_ = std::make_unique<CoreMotion>(&wmx_);

  if (!wmx_param_file_.empty()) {
    int err = cm_->config->ImportAndSetAll(const_cast<char *>(wmx_param_file_.c_str()));
    if (err != ErrorCode::None) {
      cm_->ErrorToString(err, err_str_, sizeof(err_str_));
      RCLCPP_ERROR(logger_, "Failed to import WMX params. Error=%d (%s)", err, err_str_);
    } else {
      RCLCPP_INFO(logger_, "Applied WMX params from %s", wmx_param_file_.c_str());
    }
  }

  cm_->GetStatus(&cm_status_);
  for (auto & joint : joints_) {
    joint.pos_state = cm_status_.axesStatus[joint.axis].actualPos;
    joint.vel_state = cm_status_.axesStatus[joint.axis].actualVelocity;
    joint.cmd = 0.0;
    joint.last_cmd = std::numeric_limits<double>::quiet_NaN();
  }

  RCLCPP_INFO(logger_, "WmxSystemHardware configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // wmx_engine_node is launched alongside the controller_manager and takes ~1s
  // to bring EtherCAT to Communicating. Wait for it (bounded) rather than
  // failing activation outright, which aborts ros2_control_node.
  bool communicating = false;
  for (int attempt = 1; attempt <= max_device_retries_; ++attempt) {
    if (engineCommunicating()) {
      communicating = true;
      break;
    }
    RCLCPP_WARN(
      logger_, "WMX engine not Communicating yet, waiting... (%d/%d)",
      attempt, max_device_retries_);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  if (!communicating) {
    RCLCPP_ERROR(
      logger_,
      "WMX engine is not Communicating after %d s. Start wmx_engine_node and "
      "EtherCAT communication first.", max_device_retries_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Clear amp alarms and enable the servos (StartVel/CyclicBuffer require
  // servo-on). Disable via the 'auto_servo_on' hardware param if servos are
  // managed elsewhere.
  if (auto_servo_on_) {
    for (auto & joint : joints_) {
      cm_->axisControl->ClearAmpAlarm(joint.axis);
    }
    for (auto & joint : joints_) {
      int err = cm_->axisControl->SetServoOn(joint.axis, 1, 2000);
      if (err != ErrorCode::None) {
        cm_->ErrorToString(err, err_str_, sizeof(err_str_));
        RCLCPP_ERROR(
          logger_, "Failed to servo-on axis %d. Error=%d (%s)", joint.axis, err, err_str_);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    cm_->axisControl->SetServoOn(6, 1, 2000);
    RCLCPP_INFO(logger_, "Servos enabled on all axes");
  }

  cm_->GetStatus(&cm_status_);
  for (auto & joint : joints_) {
    joint.pos_state = cm_status_.axesStatus[joint.axis].actualPos;
    joint.vel_state = cm_status_.axesStatus[joint.axis].actualVelocity;
    joint.cmd = 0.0;
    joint.last_cmd = std::numeric_limits<double>::quiet_NaN();
  }

  RCLCPP_INFO(logger_, "WmxSystemHardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & joint : joints_) {
    if (joint.mode == JointMode::Velocity) {
      startVelocity(joint, 0.0);
    }
  }
  if (auto_servo_on_) {
    for (auto & joint : joints_) {
      cm_->axisControl->SetServoOn(joint.axis, 0, 2000);
    }
    cm_->axisControl->SetServoOn(6, 0, 2000);
  }
  RCLCPP_INFO(logger_, "WmxSystemHardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  closeDevice();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WmxSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & joint : joints_) {
    state_interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_POSITION, &joint.pos_state);
    state_interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_VELOCITY, &joint.vel_state);
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
  int err = cm_->GetStatus(&cm_status_);
  if (err != ErrorCode::None) {
    cm_->ErrorToString(err, err_str_, sizeof(err_str_));
    RCLCPP_ERROR_THROTTLE(
      logger_, clock_, 1000, "GetStatus failed. Error=%d (%s)", err, err_str_);
    return hardware_interface::return_type::ERROR;
  }

  for (auto & joint : joints_) {
    joint.pos_state = cm_status_.axesStatus[joint.axis].actualPos;
    joint.vel_state = cm_status_.axesStatus[joint.axis].actualVelocity;
  }
  return hardware_interface::return_type::OK;
}

void WmxSystemHardware::startVelocity(const WmxJoint & joint, double omega)
{
  Velocity::VelCommand cmd;
  cmd.axis = joint.axis;
  cmd.profile.velocity = omega;
  cmd.profile.type = ProfileType::T::TimeAccTrapezoidal;
  cmd.profile.accTimeMilliseconds = acc_time_ms_;
  cmd.profile.decTimeMilliseconds = dec_time_ms_;

  int err = cm_->velocity->StartVel(&cmd);
  if (err != ErrorCode::None) {
    cm_->ErrorToString(err, err_str_, sizeof(err_str_));
    RCLCPP_ERROR_THROTTLE(
      logger_, clock_, 1000,
      "StartVel failed on axis %d. Error=%d (%s)", joint.axis, err, err_str_);
  }
}

hardware_interface::return_type WmxSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (cm_status_.engineState != EngineState::T::Communicating) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 1000, "WMX engine not Communicating; skipping write()");
    return hardware_interface::return_type::OK;
  }

  for (auto & joint : joints_) {
    if (joint.mode != JointMode::Velocity) {
      continue;
    }
    // Skip (don't spam StartVel errors) while the servo is off or alarmed.
    const auto & axis_status = cm_status_.axesStatus[joint.axis];
    if (!axis_status.servoOn || axis_status.ampAlarm) {
      RCLCPP_WARN_THROTTLE(
        logger_, clock_, 2000,
        "Axis %d not ready (servoOn=%d, ampAlarm=%d); skipping velocity command",
        joint.axis, axis_status.servoOn, axis_status.ampAlarm);
      joint.last_cmd = std::numeric_limits<double>::quiet_NaN();
      continue;
    }
    if (std::isnan(joint.last_cmd) || std::fabs(joint.cmd - joint.last_cmd) > kCmdEpsilon) {
      startVelocity(joint, joint.cmd);
      joint.last_cmd = joint.cmd;
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace wmx_r2_control

PLUGINLIB_EXPORT_CLASS(
  wmx_r2_control::WmxSystemHardware, hardware_interface::SystemInterface)
