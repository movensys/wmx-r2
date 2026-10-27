// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_r2_control/wmx_system_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
using wmx3Api::EStopLevel;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;
using wmx3Api::Velocity;

namespace
{
constexpr double kCmdEpsilon = 1e-9;

std::string errorToString(int err)
{
  char errString[256] = {};
  CoreMotion::ErrorToString(err, errString, sizeof(errString));
  return errString;
}

const char * modeName(JointMode mode)
{
  switch (mode) {
    case JointMode::Position:
      return "position";
    case JointMode::Velocity:
      return "velocity";
    default:
      return "state-only";
  }
}
}  // namespace

WmxSystemHardwareApi::WmxSystemHardwareApi(
  const rclcpp::Logger & logger, const Config & config)
: logger_(logger), config_(config), cm_(&wmx3Lib_)
{
}

WmxSystemHardwareApi::~WmxSystemHardwareApi()
{
  closeDevice();
}

int WmxSystemHardwareApi::createDevice(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  int err = ErrorCode::None;
  for (int attempt = 1; attempt <= config_.maxDeviceRetries; ++attempt) {
    err = wmx3Lib_.CreateDevice(
      config_.sdkPath.c_str(), DeviceType::DeviceTypeNormal, timeout_);
    if (err == ErrorCode::None) {
      break;
    }
    if (err != ErrorCode::StartProcessLockError) {
      message = "Failed to attach to device. Error=" + std::to_string(err) +
        " (" + errorToString(err) + ")";
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
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    // The device exists but is unnamed: close it so the next attempt starts clean.
    wmx3Lib_.CloseDevice();
    return err;
  }

  cm_ = CoreMotion(&wmx3Lib_);

  message = "Attached to WMX3 device as '" + config_.deviceName + "'";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void WmxSystemHardwareApi::closeDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(
      logger_, "Failed to close WMX device. Error=%d (%s)", err, errorToString(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "WMX device closed");
  }
}

int WmxSystemHardwareApi::importAndSetAll(const std::string & path, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = cm_.config->ImportAndSetAll(const_cast<char *>(path.c_str()));
  if (err != ErrorCode::None) {
    message = "Failed to import WMX params from " + path + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
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

  CoreMotionStatus status;
  const int err = cm_.GetStatus(&status);
  if (err != ErrorCode::None) {
    message = "GetStatus failed. Error=" + std::to_string(err) + " (" + errorToString(err) + ")";
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

  Velocity::VelCommand command;
  command.axis = axis;
  command.profile.velocity = omega;
  command.profile.type = ProfileType::T::TimeAccTrapezoidal;
  command.profile.accTimeMilliseconds = config_.accTimeMilliseconds;
  command.profile.decTimeMilliseconds = config_.decTimeMilliseconds;

  const int err = cm_.velocity->StartVel(&command);
  if (err != ErrorCode::None) {
    message = "StartVel failed on axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    return err;
  }

  message = "Axis " + std::to_string(axis) + " running at " + std::to_string(omega);
  return ErrorCode::None;
}

int WmxSystemHardwareApi::stop(int axis, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = cm_.motion->Stop(axis);
  if (err != ErrorCode::None) {
    message = "Stop failed on axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    return err;
  }

  message = "Stopped axis " + std::to_string(axis);
  return ErrorCode::None;
}

int WmxSystemHardwareApi::waitForMotionComplete(
  const std::vector<int> & axes, unsigned int timeoutMilliseconds, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (axes.empty()) {
    message = "No axes to wait for";
    return ErrorCode::None;
  }

  if (axes.size() > static_cast<size_t>(wmx3Api::constants::maxAxes)) {
    message = "Too many axes: " + std::to_string(axes.size()) + " requested, the SDK allows " +
      std::to_string(wmx3Api::constants::maxAxes) + ".";
    return ErrorCode::ArgumentOutOfRange;
  }

  wmx3Api::AxisSelection axisSel;
  axisSel.axisCount = static_cast<int>(axes.size());
  for (size_t i = 0; i < axes.size(); ++i) {
    axisSel.axis[i] = axes[i];
  }

  const int err = cm_.motion->Wait(&axisSel, timeoutMilliseconds);
  if (err != ErrorCode::None) {
    message = "Axes did not reach motion complete within " +
      std::to_string(timeoutMilliseconds) + " ms. Error=" + std::to_string(err) +
      " (" + errorToString(err) + ")";
    return err;
  }

  message = "All axes reached motion complete";
  return ErrorCode::None;
}

int WmxSystemHardwareApi::setServoOn(int axis, int newStatus, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = cm_.axisControl->SetServoOn(axis, newStatus, servoOnTimeout_);
  if (err != ErrorCode::None) {
    message = "Failed to servo-" + std::string(newStatus ? "on" : "off") + " axis " +
      std::to_string(axis) + ". Error=" + std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Servo " + std::to_string(axis) + " " + (newStatus ? "on" : "off");
  return ErrorCode::None;
}

int WmxSystemHardwareApi::clearAmpAlarm(int axis, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = cm_.axisControl->ClearAmpAlarm(axis);
  if (err != ErrorCode::None) {
    message = "Failed to clear the amp alarm on axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_WARN(logger_, "%s", message.c_str());
    return err;
  }

  message = "Amp alarm cleared on axis " + std::to_string(axis);
  return ErrorCode::None;
}

int WmxSystemHardwareApi::execEStopFinal(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = cm_.ExecEStop(EStopLevel::Final);
  if (err != ErrorCode::None) {
    message = "Emergency stop (Final) failed. Error=" + std::to_string(err) + " (" +
      errorToString(err) + ")";
    return err;
  }

  message = "Emergency stop (Final): servo off on all axes";
  return ErrorCode::None;
}

int WmxSystemHardwareApi::releaseEStopIfActive(bool & released, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  released = false;
  CoreMotionStatus status;
  int err = cm_.GetStatus(&status);
  if (err != ErrorCode::None) {
    message = "GetStatus failed before releasing the emergency stop. Error=" + std::to_string(err) +
      " (" + errorToString(err) + ")";
    return err;
  }
  if (!status.emergencyStop) {
    message = "No emergency stop to release";
    return ErrorCode::None;
  }

  err = cm_.ReleaseEStop();
  if (err != ErrorCode::None) {
    message = "Failed to release the emergency stop. Error=" + std::to_string(err) + " (" +
      errorToString(err) + ")";
    return err;
  }

  released = true;
  message = "Emergency stop released";
  return ErrorCode::None;
}

int WmxSystemHardwareApi::getPosCommand(int axis, double & position, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = cm_.axisControl->GetPosCommand(axis, &position);
  if (err != ErrorCode::None) {
    message = "GetPosCommand failed on axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Axis " + std::to_string(axis) + " command position " + std::to_string(position);
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
  stopTimeout_ = static_cast<unsigned int>(std::stoul(getHwParam("stop_timeout_ms", "2000")));

  cyclicBufferHorizonS_ = std::stod(getHwParam("cyclic_buffer_horizon_s", "5.0"));
  cyclicTargetPeriods_ = std::stoi(getHwParam("cyclic_target_periods", "2"));
  cyclicMaxAcc_ = std::stod(getHwParam("cyclic_max_acc", "0.0"));
  cyclicPushPeriodMs_ = std::stod(getHwParam("cyclic_push_period_ms", "10.0"));
  // See cyclic_stream_timing.hpp "HERMITE SUB-SAMPLING".
  const std::string subsample = getHwParam("cyclic_subsample", "linear");
  if (subsample != "linear" && subsample != "hermite") {
    RCLCPP_FATAL(
      logger_, "cyclic_subsample must be 'linear' or 'hermite', got '%s'", subsample.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  cyclicHermite_ = (subsample == "hermite");
  if (cyclicHermite_ && !(cyclicMaxAcc_ > 0.0)) {
    // hermite makes maxAcc mean real acceleration; running without the trip would
    // hide a bent or stalled segment entirely.
    RCLCPP_FATAL(logger_, "cyclic_subsample 'hermite' needs cyclic_max_acc > 0");
    return hardware_interface::CallbackReturn::ERROR;
  }

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
    } else if (cmd_ifs.size() == 1 && cmd_ifs[0].name == hardware_interface::HW_IF_POSITION) {
      joint.mode = JointMode::Position;
    } else if (cmd_ifs.size() == 1 && cmd_ifs[0].name == hardware_interface::HW_IF_VELOCITY) {
      joint.mode = JointMode::Velocity;
    } else if (cyclicHermite_ && cmd_ifs.size() == 2 &&
      cmd_ifs[0].name == hardware_interface::HW_IF_POSITION &&
      cmd_ifs[1].name == hardware_interface::HW_IF_VELOCITY)
    {
      // hermite: position + the controller's desired velocity
      joint.mode = JointMode::Position;
      joint.velFf = true;
    } else {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s' must declare either no command_interface (state-only), a "
        "single 'position' or 'velocity' command_interface, or (cyclic_subsample "
        "'hermite' only) 'position' then 'velocity'", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      logger_, "Joint '%s' -> WMX axis %d, mode=%s",
      joint.name.c_str(), joint.axis, modeName(joint.mode));

    axes_.push_back(joint.axis);
    joints_.push_back(joint);
  }

  bool hasVelocityJoint = false;
  positionJointIdx_.clear();
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].mode == JointMode::Position) {
      positionJointIdx_.push_back(i);
    } else if (joints_[i].mode == JointMode::Velocity) {
      hasVelocityJoint = true;
    }
  }

  // Position joints are driven by the cyclic buffer, which puts their axes into
  // DirectControl; velocity joints are driven by StartVel, which needs the axis
  // idle to accept a new profile. One ros2_control block cannot do both.
  if (!positionJointIdx_.empty() && hasVelocityJoint) {
    RCLCPP_FATAL(
      logger_,
      "Mixing 'position' and 'velocity' command interfaces in a single "
      "<ros2_control> block is not supported: position joints stream through the "
      "cyclic buffer (axes enter DirectControl) while velocity joints use "
      "StartVel. Declare them in separate <ros2_control> blocks.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // write() runs in the control loop and must not allocate.
  qCmdScratch_.assign(positionJointIdx_.size(), 0.0);
  vCmdScratch_.assign(positionJointIdx_.size(), 0.0);
  streamErr_.reserve(256);

  RCLCPP_INFO(
    logger_,
    "WmxSystemHardware initialised: %zu joints (%zu position-streamed), "
    "sdk_path=%s, param_file=%s",
    joints_.size(), positionJointIdx_.size(), config.sdkPath.c_str(),
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
    // 0 means "stop" for a velocity joint but "go to 0 rad" for a position one,
    // so a position joint is seeded to where it already is. on_activate re-seeds
    // it from the WMX command position, which the stream's hold is built from.
    joints_[i].cmd = (joints_[i].mode == JointMode::Position) ? joints_[i].posState : 0.0;
    joints_[i].lastCmd = std::numeric_limits<double>::quiet_NaN();
  }
}

bool WmxSystemHardware::seedPositionCommands()
{
  for (const size_t i : positionJointIdx_) {
    WmxJoint & joint = joints_[i];
    // The *command* position, not the feedback position: it is what the cyclic
    // buffer's primed hold is built from (see WmxCyclicStream::holdCommands), so
    // seeding from feedback here would make the first streamed command differ
    // from the hold by the servo following error -- a step on every axis at once.
    double commanded = 0.0;
    std::string message;
    if (api_->getPosCommand(joint.axis, commanded, message) != ErrorCode::None) {
      return false;
    }
    joint.cmd = commanded;
    joint.velCmd = 0.0;
  }
  return true;
}

bool WmxSystemHardware::openAndStartStream()
{
  WmxCyclicStream::Config cfg;
  cfg.axes.reserve(positionJointIdx_.size());
  for (const size_t i : positionJointIdx_) {
    cfg.axes.push_back(joints_[i].axis);
  }
  cfg.buffer_horizon_s = cyclicBufferHorizonS_;
  cfg.target_periods = static_cast<unsigned int>(std::max(1, cyclicTargetPeriods_));
  cfg.max_acc = cyclicMaxAcc_;
  cfg.hermite = cyclicHermite_;

  // [local 2026-10-26] on_deactivate (stopAllAxes) only aborts the buffer and leaves it open, so
  // re-activating the hardware (servo off -> on, alarm clear) failed with "cyclic stream is already
  // open" and dropped the component to unconfigured. Close first; close() is a no-op when not open.
  stream_.close();
  if (!stream_.open(api_->device(), api_->coreMotion(), cfg, cyclicPushPeriodMs_, &streamErr_)) {
    RCLCPP_ERROR(logger_, "Cyclic buffer open failed: %s", streamErr_.c_str());
    return false;
  }
  if (!stream_.start(&streamErr_)) {
    RCLCPP_ERROR(logger_, "Cyclic buffer start failed: %s", streamErr_.c_str());
    stream_.close();
    return false;
  }

  const auto & timing = stream_.timing();
  RCLCPP_INFO(
    logger_,
    "Cyclic buffer streaming %zu axes: engine cycle %.3f ms, seed push period "
    "%.3f ms -> %u cycles per command, depth setpoint %u commands, maxAcc %.3f",
    cfg.axes.size(), timing.cycle_ms, timing.push_period_ms,
    timing.nominalInterval(), stream_.targetDepth(), cfg.max_acc);
  if (cfg.hermite) {
    size_t nFf = 0;
    for (const size_t i : positionJointIdx_) {
      nFf += joints_[i].velFf ? 1 : 0;
    }
    RCLCPP_INFO(
      logger_,
      "Cyclic subsample: hermite -> one-cycle samples over each write() period (~%u), "
      "depth setpoint %u cycles, longest segment %u cycles, velocity interface exported on %zu/%zu "
      "joints (used while a controller claims it, else the chord)",
      timing.nominalInterval(), stream_.targetCycles(), stream_.maxSegmentCycles(), nFf,
      positionJointIdx_.size());
  }
  return true;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string message;
  if (api_->createDevice(message) != ErrorCode::None) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!wmxParamFile_.empty() &&
    api_->importAndSetAll(wmxParamFile_, message) != ErrorCode::None)
  {
    RCLCPP_FATAL(
      logger_, "Refusing to run with default axis parameters: rad and count would not match. %s",
      message.c_str());
    return hardware_interface::CallbackReturn::ERROR;
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

  // [local 2026-10-27] A fault (on_error) or a stop that did not complete leaves the engine in
  // emergency stop, in which no axis can be commanded. Activation is the operator's explicit
  // restart (GUI servo on / alarm clear), so release it here.
  {
    std::string message;
    bool released = false;
    if (api_->releaseEStopIfActive(released, message) != ErrorCode::None) {
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (released) {
      RCLCPP_WARN(logger_, "%s (activation = operator restart)", message.c_str());
    }
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

  // Position joints: seed the command from where the axis is, then open and
  // start the stream. Both must happen after servo-on above -- the cyclic buffer
  // reports ServoOff otherwise -- and seeding must precede start() so the first
  // write() to land, before any controller has claimed the interfaces, is a hold
  // rather than a six-axis move to 0 rad.
  if (!positionJointIdx_.empty()) {
    if (!seedPositionCommands() || !openAndStartStream()) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    streamStarted_ = true;
  }

  RCLCPP_INFO(logger_, "WmxSystemHardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void WmxSystemHardware::stopAllAxes()
{
  if (!api_) {
    return;
  }

  // Abort before anything else: the axes are in DirectControl until the buffer
  // is aborted, and dropping the servo out from under a running stream leaves it
  // in the latching ServoOff state instead of cleanly Stopped.
  if (streamStarted_) {
    stream_.abort();
    streamStarted_ = false;
  }

  std::string message;

  for (const WmxJoint & joint : joints_) {
    if (joint.mode == JointMode::Velocity) {
      api_->startVel(joint.axis, 0.0, message);
    } else {
      api_->stop(joint.axis, message);
    }
  }

  if (!autoServoOn_) {
    return;
  }

  // [local 2026-10-27] Shutdown after a servo off: the servos are already off and the engine node
  // may have stopped communication first (Wait then fails with "Communication is not started").
  // With no communication or no servo on there is nothing to stop or cut, so no wait and no
  // emergency stop below.
  {
    std::vector<WmxSystemHardwareApi::AxisFeedback> feedback;
    bool communicating = false;
    if (api_->getStatus(axes_, feedback, communicating, message) == ErrorCode::None) {
      bool anyServoOn = false;
      for (const auto & axis : feedback) {
        anyServoOn = anyServoOn || axis.servoOn;
      }
      if (!communicating || !anyServoOn) {
        return;
      }
    }
  }

  if (api_->waitForMotionComplete(axes_, stopTimeout_, message) != ErrorCode::None) {
    // [local 2026-10-27] This used to leave the servos ON while the deactivation still reported
    // success (the GUI then showed servo off). A stop that does not complete is an R2 fault:
    // emergency-stop Final, as on_error does.
    RCLCPP_ERROR(logger_, "%s -- emergency stop", message.c_str());
    eStopFinal();
    return;
  }

  for (const WmxJoint & joint : joints_) {
    api_->setServoOn(joint.axis, 0, message);
  }
}

void WmxSystemHardware::eStopFinal()
{
  if (!api_) {
    return;
  }

  // E-stop first (the fastest way to cut every axis), the buffer is cleaned up after.
  std::string message;
  if (api_->execEStopFinal(message) == ErrorCode::None) {
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  } else {
    RCLCPP_ERROR(
      logger_, "%s -- the drives' own watchdog and the hardware E-stop (STO) remain",
      message.c_str());
  }

  if (streamStarted_) {
    stream_.abort();
    streamStarted_ = false;
  }
}

hardware_interface::CallbackReturn WmxSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopAllAxes();

  RCLCPP_INFO(logger_, "WmxSystemHardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // [local 2026-10-27, user decision: an R2 fault is critical, stop at once] A fault (MaxAccError,
  // loop stall, servo or communication loss) leaves the command path untrustworthy and the stop
  // time unknown, so no decelerate-and-wait (stopAllAxes): emergency-stop Final drops the servo on
  // every axis at once and the drive brakes hold the arm. With communication already gone the
  // engine cannot reach the drives; their own watchdog and the hardware E-stop are left.
  eStopFinal();

  RCLCPP_ERROR(logger_, "WmxSystemHardware entered the error state: emergency stop, all servos off");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopAllAxes();

  if (api_) {
    // Deallocates on the engine; must happen while the device is still open.
    stream_.close();
    api_->closeDevice();
  }

  RCLCPP_INFO(logger_, "WmxSystemHardware shut down");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Deallocates on the engine; must happen while the device is still open.
  stream_.close();
  streamStarted_ = false;
  api_->closeDevice();
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
    if (joint.mode == JointMode::Position) {
      command_interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.cmd);
      if (joint.velFf) {
        command_interfaces.emplace_back(
          joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velCmd);
      }
    } else if (joint.mode == JointMode::Velocity) {
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

  if (!communicating_) {
    RCLCPP_ERROR_THROTTLE(
      logger_, clock_, 1000, "WMX engine not Communicating; joint states are stale");
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joints_.size() && i < feedback_.size(); ++i) {
    joints_[i].posState = feedback_[i].actualPos;
    joints_[i].velState = feedback_[i].actualVelocity;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WmxSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // hermite: a velocity interface nobody claims would otherwise feed its seed (0)
  // or a previous owner's last value as the cubic's end slope.
  for (auto & joint : joints_) {
    if (!joint.velFf) {
      continue;
    }
    const std::string name = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
    if (std::find(stop_interfaces.begin(), stop_interfaces.end(), name) != stop_interfaces.end()) {
      joint.velClaimed = false;
      joint.velCmd = 0.0;
    }
    if (std::find(start_interfaces.begin(), start_interfaces.end(), name) !=
      start_interfaces.end())
    {
      joint.velClaimed = true;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WmxSystemHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!communicating_) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 1000, "WMX engine not Communicating; skipping write()");
    return hardware_interface::return_type::OK;
  }

  if (streamStarted_) {
    // The commanded interval is a ratio of the loop period, so track the real
    // one rather than trusting the configured update_rate.
    stream_.observePushPeriod(period.seconds() * 1000.0);

    for (size_t k = 0; k < positionJointIdx_.size(); ++k) {
      const WmxJoint & joint = joints_[positionJointIdx_[k]];
      qCmdScratch_[k] = joint.cmd;
      // Unclaimed velocity -> NaN: the stream then uses the chord, never a stale value or 0
      vCmdScratch_[k] = (joint.velFf && joint.velClaimed) ? joint.velCmd :
        std::numeric_limits<double>::quiet_NaN();
    }
    // Hermite segment length = the loop-top period that ros2_control_node passes to
    // read/update/write alike; JTC advances its trajectory clock by exactly this.
    // The time between write() calls differs from it by the read+update duration
    // jitter (~20 us): at 2 rad/s a 40 urad position/velocity mismatch per segment
    // gives +-3..7 rad/s^2 per-cycle acceleration noise and MaxAccError on single axes.
    const double segmentS = period.seconds();

    // Diagnostic: CR3A_R2_TRACE=<file> records every command write() receives
    // (time, segment, positions, velocities).
    static FILE * trace = nullptr;
    static bool traceInit = false;
    if (!traceInit) {
      traceInit = true;
      const char * tracePath = std::getenv("CR3A_R2_TRACE");
      if (tracePath != nullptr && tracePath[0] != 0) {
        trace = std::fopen(tracePath, "w");
      }
    }
    if (trace != nullptr) {
      std::fprintf(trace, "%.6f %.7f", time.seconds(), segmentS);
      for (size_t k = 0; k < qCmdScratch_.size(); ++k) {
        std::fprintf(trace, " %.9f", qCmdScratch_[k]);
      }
      for (size_t k = 0; k < vCmdScratch_.size(); ++k) {
        std::fprintf(trace, " %.7f", vCmdScratch_[k]);
      }
      std::fprintf(trace, "\n");
    }

    // push() polls the buffer itself, so there is deliberately no refresh() in
    // read(): that would double the engine round-trips per control cycle.
    const bool pushed = stream_.hermite() ?
      stream_.pushSamples(qCmdScratch_, vCmdScratch_, segmentS, &streamErr_) :
      stream_.push(qCmdScratch_, &streamErr_);
    if (!pushed) {
      if (stream_.faulted()) {
        // Latching: another push cannot recover it. Surfacing ERROR lets the
        // controller manager deactivate rather than spin on a dead stream.
        RCLCPP_ERROR_THROTTLE(
          logger_, clock_, 1000,
          "Cyclic buffer faulted (%s): %s", stream_.stateName(), streamErr_.c_str());
        return hardware_interface::return_type::ERROR;
      }
      // Transient: a full buffer or a non-finite command. The axis holds at its
      // last commanded position until a push lands again.
      RCLCPP_WARN_THROTTLE(
        logger_, clock_, 1000,
        "Cyclic push rejected (state=%s depth=%u/%u %s, rejected=%u): %s",
        stream_.stateName(), stream_.depth(),
        stream_.hermite() ? stream_.targetCycles() : stream_.targetDepth(),
        stream_.hermite() ? "cycles" : "commands",
        stream_.rejected(), streamErr_.c_str());
    }
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

    if (!std::isfinite(joint.cmd)) {
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 1000,
        "Axis %d commanded %f; skipping velocity command", joint.axis, joint.cmd);
      continue;
    }

    if (std::isnan(joint.lastCmd) || std::fabs(joint.cmd - joint.lastCmd) > kCmdEpsilon) {
      std::string message;
      if (api_->startVel(joint.axis, joint.cmd, message) != ErrorCode::None) {
        RCLCPP_ERROR_THROTTLE(logger_, clock_, 1000, "%s", message.c_str());
        joint.lastCmd = std::numeric_limits<double>::quiet_NaN();
        continue;
      }
      joint.lastCmd = joint.cmd;
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace wmx_r2_control

PLUGINLIB_EXPORT_CLASS(
  wmx_r2_control::WmxSystemHardware, hardware_interface::SystemInterface)
