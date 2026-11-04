// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_r2_control/wmx_system_hardware.hpp"

#include <algorithm>
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
using wmx3Api::Motion;
using wmx3Api::ProfileType;
using wmx3Api::Velocity;

namespace
{
constexpr double kCmdEpsilon = 1e-9;
constexpr const char * kPassthroughPrefix = "trajectory_passthrough/";
constexpr const char * kSetpointPositionsPrefix = "setpoint_positions_";

std::string errorToString(int err)
{
  char errString[256] = {};
  CoreMotion::ErrorToString(err, errString, sizeof(errString));
  return errString;
}
}  // namespace

WmxSystemHardwareApi::WmxSystemHardwareApi(
  const rclcpp::Logger & logger, const Config & config)
: logger_(logger), config_(config), cm_(&wmx3Lib_), io_(&wmx3Lib_), am_(&wmx3Lib_)
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
  io_ = wmx3Api::IO(&wmx3Lib_);
  am_ = wmx3Api::AdvancedMotion(&wmx3Lib_);

  message = "Attached to WMX3 device as '" + config_.deviceName + "'";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void WmxSystemHardwareApi::closeDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (splineBufferCreated_) {
    am_.advMotion->FreeSplineBuffer(kSplineChannel);
    splineBufferCreated_ = false;
  }

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
    feedback.push_back(
      {raw.actualPos, raw.actualVelocity, raw.servoOn, raw.ampAlarm, raw.posCmd, raw.inPos});
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

int WmxSystemHardwareApi::startLinearIntplPos(
  const std::vector<int> & axes, const std::vector<double> & targets,
  const std::vector<double> & maxVelocity, const std::vector<double> & maxAcc,
  std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  Motion::LinearIntplCommand intpl;
  intpl.axisCount = static_cast<unsigned int>(axes.size());
  for (size_t i = 0; i < axes.size(); ++i) {
    intpl.axis[i] = axes[i];
    intpl.target[i] = targets[i];
    intpl.maxVelocity[i] = maxVelocity[i];
    intpl.maxAcc[i] = maxAcc[i];
    intpl.maxDec[i] = maxAcc[i];
  }
  intpl.profile.type = ProfileType::Trapezoidal;
  intpl.profile.velocity = 0.0;
  intpl.profile.acc = 0.0;
  intpl.profile.dec = 0.0;

  const int err = cm_.motion->StartLinearIntplPos(&intpl);
  if (err != ErrorCode::None) {
    message = "StartLinearIntplPos failed. Error=" + std::to_string(err) + " (" +
      errorToString(err) + ")";
    return err;
  }
  return ErrorCode::None;
}

int WmxSystemHardwareApi::setOutBit(int addr, int bit, bool on)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);
  return io_.SetOutBit(addr, bit, on ? 1 : 0);
}

int WmxSystemHardwareApi::setOutByte(int addr, int data)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);
  return io_.SetOutByte(addr, static_cast<unsigned char>(data));
}

int WmxSystemHardwareApi::createSplineBuffer(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (splineBufferCreated_) {
    return ErrorCode::None;
  }

  const int err = am_.advMotion->CreateSplineBuffer(kSplineChannel, kMaxTrajectoryPoints);
  if (err != ErrorCode::None) {
    message = "Failed to create the spline buffer on channel " +
      std::to_string(kSplineChannel) + ". Error=" + std::to_string(err) +
      " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  splinePoints_.resize(kMaxTrajectoryPoints);
  splineTimesMs_.resize(kMaxTrajectoryPoints);
  splineBufferCreated_ = true;
  return ErrorCode::None;
}

void WmxSystemHardwareApi::freeSplineBuffer()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!splineBufferCreated_) {
    return;
  }
  const int err = am_.advMotion->FreeSplineBuffer(kSplineChannel);
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(
      logger_, "Failed to free the spline buffer. Error=%d (%s)", err, errorToString(err).c_str());
  }
  splineBufferCreated_ = false;
}

int WmxSystemHardwareApi::startCSplinePos(
  const std::vector<int> & axes, const std::vector<double> & positions,
  const std::vector<double> & timesMs, unsigned int pointCount, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!splineBufferCreated_) {
    message = "Cannot start the trajectory. The spline buffer was not created.";
    return ErrorCode::ArgumentOutOfRange;
  }

  if (pointCount == 0 || pointCount > kMaxTrajectoryPoints ||
    positions.size() < static_cast<size_t>(pointCount) * axes.size() ||
    timesMs.size() < pointCount)
  {
    message = "Invalid trajectory point count " + std::to_string(pointCount) +
      ": must be in [1, " + std::to_string(kMaxTrajectoryPoints) + "].";
    return ErrorCode::ArgumentOutOfRange;
  }

  const size_t axisCount = axes.size();
  splineCommand_.dimensionCount = static_cast<unsigned int>(axisCount);
  for (size_t j = 0; j < axisCount; ++j) {
    splineCommand_.axis[j] = axes[j];
  }
  for (size_t i = 0; i < pointCount; ++i) {
    for (size_t j = 0; j < axisCount; ++j) {
      splinePoints_[i].pos[j] = positions[i * axisCount + j];
    }
    splineTimesMs_[i] = timesMs[i];
  }

  const int err = am_.advMotion->StartCSplinePos(
    kSplineChannel, &splineCommand_, pointCount, splinePoints_.data(), splineTimesMs_.data());
  if (err != ErrorCode::None) {
    message = "StartCSplinePos failed. Error=" + std::to_string(err) +
      " (" + errorToString(err) + ")";
    return err;
  }
  return ErrorCode::None;
}

int WmxSystemHardwareApi::stopAxes(const std::vector<int> & axes, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (axes.empty() || axes.size() > static_cast<size_t>(wmx3Api::constants::maxAxes)) {
    message = "Invalid axis count " + std::to_string(axes.size());
    return ErrorCode::ArgumentOutOfRange;
  }

  wmx3Api::AxisSelection axisSel;
  axisSel.axisCount = static_cast<int>(axes.size());
  for (size_t i = 0; i < axes.size(); ++i) {
    axisSel.axis[i] = axes[i];
  }

  const int err = cm_.motion->Stop(&axisSel);
  if (err != ErrorCode::None) {
    message = "Stop failed. Error=" + std::to_string(err) + " (" + errorToString(err) + ")";
    return err;
  }
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
  accelRatio_ = std::stod(getHwParam("accel_ratio", "0.3"));
  minStep_ = std::stod(getHwParam("min_step", "0.001"));
  if (!(accelRatio_ > 0.0) || !(minStep_ >= 0.0)) {
    RCLCPP_FATAL(
      logger_, "accel_ratio must be > 0 and min_step >= 0 (got %f, %f)", accelRatio_, minStep_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  api_ = std::make_unique<WmxSystemHardwareApi>(logger_, config);

  joints_.clear();
  grippers_.clear();
  joints_.reserve(info_.joints.size());
  axes_.clear();
  axes_.reserve(info_.joints.size());
  positionAxes_.reserve(info_.joints.size());
  positionJoints_.reserve(info_.joints.size());
  targets_.reserve(info_.joints.size());
  maxVelocity_.reserve(info_.joints.size());
  maxAcc_.reserve(info_.joints.size());

  for (const auto & j : info_.joints) {
    const auto mimic_it = j.parameters.find("mimic");
    if (mimic_it != j.parameters.end()) {
      auto target = std::find_if(
        grippers_.begin(), grippers_.end(),
        [&](const WmxGripper & g) {return g.name == mimic_it->second;});
      if (target == grippers_.end()) {
        RCLCPP_FATAL(
          logger_, "Joint '%s' mimics '%s', which is not a gripper declared before it",
          j.name.c_str(), mimic_it->second.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (!j.command_interfaces.empty()) {
        RCLCPP_FATAL(
          logger_, "Mimic joint '%s' must not declare command interfaces", j.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      target->mimicJoints.push_back(j.name);
      continue;
    }

    const auto io_byte_it = j.parameters.find("io_byte");
    if (j.parameters.find("axis") == j.parameters.end() && io_byte_it != j.parameters.end()) {
      WmxGripper gripper;
      gripper.name = j.name;
      gripper.ioByte = std::stoi(io_byte_it->second);
      gripper.ioBit = std::stoi(j.parameters.at("io_bit"));
      gripper.closeThreshold = std::stod(j.parameters.at("close_threshold"));
      const auto power_byte_it = j.parameters.find("power_byte");
      const auto power_value_it = j.parameters.find("power_value");
      if (power_byte_it != j.parameters.end() && power_value_it != j.parameters.end()) {
        gripper.hasPower = true;
        gripper.powerByte = std::stoi(power_byte_it->second);
        gripper.powerValue = std::stoi(power_value_it->second);
      }
      if (j.command_interfaces.size() != 1 ||
        j.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          logger_, "Gripper '%s' must declare a single 'position' command_interface",
          j.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(
        logger_, "Gripper '%s' -> WMX out byte %d bit %d, closed at position >= %f",
        gripper.name.c_str(), gripper.ioByte, gripper.ioBit, gripper.closeThreshold);
      grippers_.push_back(gripper);
      continue;
    }

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
    } else if (cmd_ifs.size() == 1 && cmd_ifs[0].name == hardware_interface::HW_IF_POSITION) {
      joint.mode = JointMode::Position;
    } else {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s' must declare either no command_interface (state-only) or a "
        "single 'velocity' or 'position' command_interface", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      logger_, "Joint '%s' -> WMX axis %d, mode=%s",
      joint.name.c_str(), joint.axis,
      joint.mode == JointMode::Velocity ? "velocity" :
      joint.mode == JointMode::Position ? "position" : "state-only");

    axes_.push_back(joint.axis);
    joints_.push_back(joint);
  }

  ptAxes_.clear();
  ptJoints_.clear();
  armPositionInterfaces_.clear();
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].mode == JointMode::Position) {
      ptAxes_.push_back(joints_[i].axis);
      ptJoints_.push_back(i);
      armPositionInterfaces_.push_back(joints_[i].name + "/" + hardware_interface::HW_IF_POSITION);
    }
  }
  ptSetpointPositions_.assign(ptAxes_.size(), std::numeric_limits<double>::quiet_NaN());

  hasPassthroughGpio_ = false;
  for (const auto & gpio : info_.gpios) {
    if (gpio.name != kPassthroughGpio) {
      continue;
    }
    std::vector<std::string> expected;
    for (size_t k = 0; k < ptAxes_.size(); ++k) {
      expected.push_back(kSetpointPositionsPrefix + std::to_string(k));
    }
    expected.insert(
      expected.end(), {"time_from_start", "transfer_state", "abort", "trajectory_size"});
    for (const auto & name : expected) {
      bool found = false;
      for (const auto & ci : gpio.command_interfaces) {
        found |= ci.name == name;
      }
      if (!found) {
        RCLCPP_FATAL(
          logger_, "GPIO '%s' is missing command_interface '%s'", kPassthroughGpio, name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (ptAxes_.empty() ||
      ptAxes_.size() > static_cast<size_t>(wmx3Api::constants::maxSplineDimensions))
    {
      RCLCPP_FATAL(
        logger_, "GPIO '%s' needs 1..%d position joints, got %zu", kPassthroughGpio,
        wmx3Api::constants::maxSplineDimensions, ptAxes_.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    hasPassthroughGpio_ = true;
    RCLCPP_INFO(
      logger_, "GPIO '%s' enabled for %zu position joints (CSpline channel %d)",
      kPassthroughGpio, ptAxes_.size(), WmxSystemHardwareApi::kSplineChannel);
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

bool WmxSystemHardware::seedJointStates()
{
  std::string message;
  if (api_->getStatus(axes_, feedback_, communicating_, message) != ErrorCode::None) {
    RCLCPP_WARN(logger_, "Could not seed joint states: %s", message.c_str());
    return false;
  }

  for (size_t i = 0; i < joints_.size() && i < feedback_.size(); ++i) {
    joints_[i].posState = feedback_[i].actualPos;
    joints_[i].velState = feedback_[i].actualVelocity;
    // A position command of 0 would jump the arm; hold where it is until commanded.
    joints_[i].cmd = joints_[i].mode == JointMode::Position ? feedback_[i].actualPos : 0.0;
    joints_[i].lastCmd = std::numeric_limits<double>::quiet_NaN();
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

  if (!seedJointStates()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string message;
  for (WmxGripper & gripper : grippers_) {
    // Report open; the controller's first hold command (0.0) writes the open bit.
    gripper.cmd = gripper.posState;
    gripper.lastClosed = -1;
    if (gripper.hasPower) {
      const int err = api_->setOutByte(gripper.powerByte, gripper.powerValue);
      if (err != ErrorCode::None) {
        RCLCPP_ERROR(
          logger_, "Gripper '%s': SetOutByte(%d, %d) failed. Error=%d (%s)",
          gripper.name.c_str(), gripper.powerByte, gripper.powerValue, err,
          errorToString(err).c_str());
      }
    }
  }

  armMode_ = ArmCommandMode::Position;
  ptClaimed_ = 0;
  posClaimed_ = 0;
  ptTransferState_ = passthrough_state::kIdle;
  ptAbort_ = 0.0;
  ptStopping_ = false;
  if (hasPassthroughGpio_) {
    // Reserve the whole trajectory now so write() never allocates.
    const size_t maxPoints = WmxSystemHardwareApi::kMaxTrajectoryPoints;
    ptPositions_.assign(maxPoints * ptAxes_.size(), 0.0);
    ptTimesMs_.assign(maxPoints, 0.0);
    if (api_->createSplineBuffer(message) != ErrorCode::None) {
      RCLCPP_ERROR(
        logger_, "trajectory_passthrough unavailable: %s", message.c_str());
    }
  }

  RCLCPP_INFO(logger_, "WmxSystemHardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void WmxSystemHardware::stopAllAxes()
{
  if (!api_) {
    return;
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

  if (api_->waitForMotionComplete(axes_, stopTimeout_, message) != ErrorCode::None) {
    RCLCPP_ERROR(
      logger_,
      "Leaving the servos on: %s Cutting the torque while the axes are still moving "
      "would let them coast.", message.c_str());
    return;
  }

  for (const WmxJoint & joint : joints_) {
    api_->setServoOn(joint.axis, 0, message);
  }
}

hardware_interface::CallbackReturn WmxSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopAllAxes();
  if (api_) {
    api_->freeSplineBuffer();
  }

  RCLCPP_INFO(logger_, "WmxSystemHardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopAllAxes();

  RCLCPP_ERROR(logger_, "WmxSystemHardware entered the error state, all axes stopped");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopAllAxes();

  if (api_) {
    api_->closeDevice();
  }

  RCLCPP_INFO(logger_, "WmxSystemHardware shut down");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
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
  for (auto & gripper : grippers_) {
    state_interfaces.emplace_back(
      gripper.name, hardware_interface::HW_IF_POSITION, &gripper.posState);
    state_interfaces.emplace_back(
      gripper.name, hardware_interface::HW_IF_VELOCITY, &gripper.velState);
    for (const std::string & mimic : gripper.mimicJoints) {
      state_interfaces.emplace_back(
        mimic, hardware_interface::HW_IF_POSITION, &gripper.posState);
      state_interfaces.emplace_back(
        mimic, hardware_interface::HW_IF_VELOCITY, &gripper.velState);
    }
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
    } else if (joint.mode == JointMode::Position) {
      command_interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.cmd);
    }
  }
  for (auto & gripper : grippers_) {
    command_interfaces.emplace_back(
      gripper.name, hardware_interface::HW_IF_POSITION, &gripper.cmd);
  }
  if (hasPassthroughGpio_) {
    for (size_t k = 0; k < ptSetpointPositions_.size(); ++k) {
      command_interfaces.emplace_back(
        kPassthroughGpio, kSetpointPositionsPrefix + std::to_string(k), &ptSetpointPositions_[k]);
    }
    command_interfaces.emplace_back(kPassthroughGpio, "time_from_start", &ptTimeFromStart_);
    command_interfaces.emplace_back(kPassthroughGpio, "transfer_state", &ptTransferState_);
    command_interfaces.emplace_back(kPassthroughGpio, "abort", &ptAbort_);
    command_interfaces.emplace_back(kPassthroughGpio, "trajectory_size", &ptTrajectorySize_);
  }
  return command_interfaces;
}

bool WmxSystemHardware::isPassthroughInterface(const std::string & name) const
{
  return hasPassthroughGpio_ && name.rfind(kPassthroughPrefix, 0) == 0;
}

bool WmxSystemHardware::isArmPositionInterface(const std::string & name) const
{
  for (const std::string & armIf : armPositionInterfaces_) {
    if (name == armIf) {
      return true;
    }
  }
  return false;
}

hardware_interface::return_type WmxSystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  int ptAfter = ptClaimed_;
  int posAfter = posClaimed_;
  for (const std::string & name : stop_interfaces) {
    ptAfter -= isPassthroughInterface(name) ? 1 : 0;
    posAfter -= isArmPositionInterface(name) ? 1 : 0;
  }
  bool startsPassthrough = false;
  for (const std::string & name : start_interfaces) {
    if (isPassthroughInterface(name)) {
      startsPassthrough = true;
      ++ptAfter;
    }
    posAfter += isArmPositionInterface(name) ? 1 : 0;
  }

  if (ptAfter > 0 && posAfter > 0) {
    RCLCPP_ERROR(
      logger_, "Refusing the switch: the arm position interfaces and %s cannot be claimed together",
      kPassthroughGpio);
    return hardware_interface::return_type::ERROR;
  }
  if (startsPassthrough && !api_->splineBufferCreated()) {
    RCLCPP_ERROR(
      logger_, "Refusing the switch: %s needs the WMX spline buffer, which was not created",
      kPassthroughGpio);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WmxSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (const std::string & name : stop_interfaces) {
    ptClaimed_ -= isPassthroughInterface(name) ? 1 : 0;
    posClaimed_ -= isArmPositionInterface(name) ? 1 : 0;
  }
  for (const std::string & name : start_interfaces) {
    ptClaimed_ += isPassthroughInterface(name) ? 1 : 0;
    posClaimed_ += isArmPositionInterface(name) ? 1 : 0;
  }

  const ArmCommandMode newMode =
    ptClaimed_ > 0 ? ArmCommandMode::Passthrough : ArmCommandMode::Position;
  if (newMode == armMode_) {
    return hardware_interface::return_type::OK;
  }

  if (newMode == ArmCommandMode::Passthrough) {
    ptTransferState_ = passthrough_state::kIdle;
    ptAbort_ = 0.0;
    ptStopping_ = false;
    ptExpected_ = 0;
    ptReceived_ = 0;
    RCLCPP_INFO(logger_, "Arm command mode: PASSTHROUGH (StartCSplinePos)");
  } else {
    // Controller left mid-trajectory: do not let the spline run on unattended. Stop only (no
    // Wait in the RT loop); the axes are still decelerating when POSITION takes over, so a mode
    // switch mid-motion is not supported for comparison runs.
    if (ptTransferState_ == passthrough_state::kInMotion) {
      stopPassthroughAxes();
    }
    resyncPositionCommands();
    ptTransferState_ = passthrough_state::kIdle;
    ptAbort_ = 0.0;
    ptStopping_ = false;
    RCLCPP_INFO(logger_, "Arm command mode: POSITION (StartLinearIntplPos)");
  }
  armMode_ = newMode;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WmxSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // No feedback sensor: mirror the last commanded position.
  for (WmxGripper & gripper : grippers_) {
    if (std::isfinite(gripper.cmd)) {
      gripper.posState = gripper.cmd;
    }
    gripper.velState = 0.0;
  }

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

  if (armMode_ == ArmCommandMode::Passthrough &&
    ptTransferState_ == passthrough_state::kInMotion)
  {
    ptElapsed_ += period.seconds();
    bool allInPos = true;
    bool faulted = false;
    for (const size_t i : ptJoints_) {
      if (i >= feedback_.size()) {
        continue;
      }
      allInPos &= feedback_[i].inPos;
      faulted |= !feedback_[i].servoOn || feedback_[i].ampAlarm;
    }
    if (faulted) {
      // Servo off / amp alarm: no inPos will come, report DONE right away.
      RCLCPP_ERROR(logger_, "Axis fault during the passthrough trajectory; stopping");
      stopPassthroughAxes();
      finishPassthrough(true);
    } else if (ptStopping_) {
      // Skip the first samples after Stop: inPos may still be stale from before the spline started.
      if (allInPos && ++ptStopCycles_ > kStopSettleCycles) {
        // Deceleration after the abort Stop has finished.
        finishPassthrough(true);
      }
    } else if (!allInPos) {
      ptSawMotion_ = true;
    } else if (ptSawMotion_ || ptElapsed_ >= ptDurationS_) {
      // Same completion flag the native controller polls (CoreMotionAxisStatus::inPos).
      finishPassthrough(false);
    }
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

  writeGrippers();

  if (armMode_ == ArmCommandMode::Passthrough) {
    writePassthrough();
  }

  positionAxes_.clear();
  positionJoints_.clear();
  bool positionsReady = true;
  for (size_t i = 0; i < joints_.size() && i < feedback_.size(); ++i) {
    WmxJoint & joint = joints_[i];
    if (joint.mode == JointMode::StateOnly) {
      continue;
    }

    const WmxSystemHardwareApi::AxisFeedback & axisStatus = feedback_[i];
    if (!axisStatus.servoOn || axisStatus.ampAlarm) {
      RCLCPP_WARN_THROTTLE(
        logger_, clock_, 2000,
        "Axis %d not ready (servoOn=%d, ampAlarm=%d); skipping command",
        joint.axis, axisStatus.servoOn, axisStatus.ampAlarm);
      joint.lastCmd = std::numeric_limits<double>::quiet_NaN();
      positionsReady &= joint.mode != JointMode::Position;
      continue;
    }

    if (!std::isfinite(joint.cmd)) {
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 1000,
        "Axis %d commanded %f; skipping command", joint.axis, joint.cmd);
      positionsReady &= joint.mode != JointMode::Position;
      continue;
    }

    if (joint.mode == JointMode::Position) {
      positionAxes_.push_back(joint.axis);
      positionJoints_.push_back(i);
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

  // The position joints move as one group: if any of them is unusable, none moves.
  // In PASSTHROUGH mode the spline owns the arm: never stream StartLinearIntplPos.
  if (positionsReady && armMode_ == ArmCommandMode::Position) {
    writePositions();
  }
  return hardware_interface::return_type::OK;
}

void WmxSystemHardware::resyncPositionCommands()
{
  for (const size_t i : ptJoints_) {
    if (i < feedback_.size()) {
      joints_[i].cmd = feedback_[i].posCmd;
    }
    joints_[i].lastCmd = std::numeric_limits<double>::quiet_NaN();
  }
}

// Stop only: never Wait in the RT loop. Completion is detected from inPos in read().
void WmxSystemHardware::stopPassthroughAxes()
{
  std::string message;
  if (api_->stopAxes(ptAxes_, message) != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  }
}

void WmxSystemHardware::finishPassthrough(bool aborted)
{
  ptAbort_ = aborted ? 1.0 : 0.0;
  ptTransferState_ = passthrough_state::kDone;
  ptStopping_ = false;
  resyncPositionCommands();
}

// Handshake in the ros2 passthrough-trajectory controller format, adapted to CSpline: collect
// every point, then one StartCSplinePos.
void WmxSystemHardware::writePassthrough()
{
  namespace ps = passthrough_state;

  if (ptAbort_ == 1.0 && ptTransferState_ != ps::kIdle) {
    if (ptTransferState_ == ps::kInMotion) {
      // Stay IN_MOTION until read() sees every spline axis inPos, then DONE (abort stays 1).
      if (!ptStopping_) {
        RCLCPP_WARN(logger_, "Passthrough trajectory aborted; stopping the arm");
        stopPassthroughAxes();
        ptStopping_ = true;
        ptStopCycles_ = 0;
      }
    } else if (ptTransferState_ != ps::kDone) {
      // Still transferring: nothing was started on WMX.
      finishPassthrough(true);
    }
    return;
  }

  if (ptTransferState_ == ps::kNewTrajectory) {
    const double size = ptTrajectorySize_;
    if (!(size >= 1.0) || size > WmxSystemHardwareApi::kMaxTrajectoryPoints ||
      size != std::floor(size))
    {
      RCLCPP_ERROR(
        logger_, "Invalid trajectory_size %f: must be an integer in [1, %u]",
        size, WmxSystemHardwareApi::kMaxTrajectoryPoints);
      finishPassthrough(true);
      return;
    }
    ptExpected_ = static_cast<size_t>(size);
    ptReceived_ = 0;
    ptElapsed_ = 0.0;
    ptSawMotion_ = false;
    ptTransferState_ = ps::kWaitingForPoint;
  } else if (ptTransferState_ == ps::kTransferring) {
    const size_t axisCount = ptAxes_.size();
    if (ptReceived_ >= ptExpected_) {
      RCLCPP_ERROR(
        logger_, "Passthrough point %zu exceeds trajectory_size %zu", ptReceived_, ptExpected_);
      finishPassthrough(true);
      return;
    }
    for (size_t j = 0; j < axisCount; ++j) {
      ptPositions_[ptReceived_ * axisCount + j] = ptSetpointPositions_[j];
    }
    ptTimesMs_[ptReceived_] = ptTimeFromStart_ * 1000.0;
    ++ptReceived_;
    ptAbort_ = 0.0;
    ptTransferState_ = ps::kWaitingForPoint;
  } else if (ptTransferState_ == ps::kTransferDone) {
    ptAbort_ = 0.0;
    if (ptReceived_ == 0 || ptReceived_ != ptExpected_) {
      RCLCPP_ERROR(
        logger_, "Passthrough transfer incomplete: %zu of %zu points", ptReceived_, ptExpected_);
      finishPassthrough(true);
      return;
    }

    // Same as native buildSplineInput: drop a last point < 1 ms after the previous one,
    // absolute times in ms, first time forced to 0.
    size_t pointCount = ptReceived_;
    if (pointCount > 1 && ptTimesMs_[pointCount - 1] - ptTimesMs_[pointCount - 2] < 1.0) {
      pointCount -= 1;
    }
    ptTimesMs_[0] = 0.0;

    std::string message;
    if (api_->startCSplinePos(
        ptAxes_, ptPositions_, ptTimesMs_, static_cast<unsigned int>(pointCount),
        message) != ErrorCode::None)
    {
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      finishPassthrough(true);
      return;
    }
    ptDurationS_ = ptTimesMs_[pointCount - 1] / 1000.0;
    ptElapsed_ = 0.0;
    ptSawMotion_ = false;
    ptStopping_ = false;
    ptTransferState_ = ps::kInMotion;
  }
}

void WmxSystemHardware::writeGrippers()
{
  for (WmxGripper & gripper : grippers_) {
    if (std::isnan(gripper.cmd)) {
      continue;
    }
    const int closed = gripper.cmd >= gripper.closeThreshold ? 1 : 0;
    if (closed == gripper.lastClosed) {
      continue;
    }
    const int err = api_->setOutBit(gripper.ioByte, gripper.ioBit, closed != 0);
    if (err != ErrorCode::None) {
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 1000, "Gripper '%s': SetOutBit(%d, %d, %d) failed. Error=%d (%s)",
        gripper.name.c_str(), gripper.ioByte, gripper.ioBit, closed, err,
        errorToString(err).c_str());
      continue;
    }
    gripper.lastClosed = closed;
  }
}

void WmxSystemHardware::writePositions()
{
  if (positionAxes_.empty()) {
    return;
  }

  // Same velocity/acceleration derivation as wmx_r2_package joint_position_controller, with
  // the nominal control period as the time to reach the target.
  const double dt = 1.0 / info_.rw_rate;
  targets_.clear();
  maxVelocity_.clear();
  maxAcc_.clear();
  double largestStep = 0.0;
  double largestVelocity = 0.0;
  for (const size_t i : positionJoints_) {
    const double step = std::fabs(joints_[i].cmd - feedback_[i].posCmd);
    largestStep = std::fmax(largestStep, step);
    const double velocity = step / dt;
    largestVelocity = std::fmax(largestVelocity, velocity);
    targets_.push_back(joints_[i].cmd);
    maxVelocity_.push_back(velocity);
    maxAcc_.push_back(velocity / (accelRatio_ * dt));
  }

  if (largestStep < minStep_ || largestVelocity <= 0.0) {
    return;
  }

  for (size_t k = 0; k < positionAxes_.size(); ++k) {
    if (maxVelocity_[k] <= 0.0) {
      maxVelocity_[k] = largestVelocity;
      maxAcc_[k] = largestVelocity / (accelRatio_ * dt);
    }
  }

  // Like the native joint_position_controller: log and drop, do not stop.
  std::string message;
  if (api_->startLinearIntplPos(
      positionAxes_, targets_, maxVelocity_, maxAcc_, message) != ErrorCode::None)
  {
    RCLCPP_WARN_THROTTLE(logger_, clock_, 1000, "%s", message.c_str());
  }
}

}  // namespace wmx_r2_control

PLUGINLIB_EXPORT_CLASS(
  wmx_r2_control::WmxSystemHardware, hardware_interface::SystemInterface)
