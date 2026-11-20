// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "joint_position_streamer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

using std::placeholders::_1;
using namespace std::chrono_literals;

using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::CyclicBuffer;
using wmx3Api::CyclicBufferCommandType;
using wmx3Api::CyclicBufferMultiAxisCommands;
using wmx3Api::CyclicBufferMultiAxisStatus;
using wmx3Api::CyclicBufferState;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;

namespace
{
std::string errorToString(int err)
{
  char errString[256] = {};
  if (err >= wmx3Api::CyclicBufferErrorCode::CreateBufferFailed &&
    err < wmx3Api::CyclicBufferErrorCode::CreateBufferFailed + 0x1000)
  {
    CyclicBuffer::ErrorToString(err, errString, sizeof(errString));
  } else {
    CoreMotion::ErrorToString(err, errString, sizeof(errString));
  }
  return errString;
}

std::string failure(const char * what, int err)
{
  return std::string(what) + " failed. Error=" + std::to_string(err) + " (" +
         errorToString(err) + ")";
}

const char * stateToString(CyclicBufferState::T state)
{
  switch (state) {
    case CyclicBufferState::Stopped: return "Stopped";
    case CyclicBufferState::ExecutingCommand: return "ExecutingCommand";
    case CyclicBufferState::WaitingForCommand: return "WaitingForCommand";
    case CyclicBufferState::MaxAccError: return "MaxAccError";
    case CyclicBufferState::CommandModeMismatchError: return "CommandModeMismatchError";
    case CyclicBufferState::Busy: return "Busy";
    case CyclicBufferState::ServoOff: return "ServoOff";
  }
  return "Unknown";
}

}  // namespace

JointPositionStreamerApi::JointPositionStreamerApi(
  const rclcpp::Logger & logger, const std::string & deviceName)
: logger_(logger), deviceName_(deviceName), cm_(&wmx3Lib_), cyc_(&wmx3Lib_)
{
}

JointPositionStreamerApi::~JointPositionStreamerApi()
{
  closeDevice();
}

int JointPositionStreamerApi::createDevice(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

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

  err = wmx3Lib_.SetDeviceName(deviceName_.c_str());
  if (err != ErrorCode::None) {
    message = "Failed to name the device '" + deviceName_ + "'. Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    wmx3Lib_.CloseDevice();
    return err;
  }

  cm_ = CoreMotion(&wmx3Lib_);
  cyc_ = CyclicBuffer(&wmx3Lib_);

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void JointPositionStreamerApi::closeDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  std::string message;
  closeCyclicBufferLocked(message);

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorToString(err).c_str());
    return;
  }

  RCLCPP_INFO(logger_, "Device closed");
}

int JointPositionStreamerApi::setAxisSelection(
  const std::vector<int64_t> & axes, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (axes.empty()) {
    message = "No axes configured: 'joint_axes' is empty.";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return ErrorCode::ArgumentOutOfRange;
  }

  if (axes.size() > static_cast<size_t>(wmx3Api::constants::maxAxes)) {
    message = "Too many axes: " + std::to_string(axes.size()) + " configured, the SDK allows " +
      std::to_string(wmx3Api::constants::maxAxes) + ".";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return ErrorCode::ArgumentOutOfRange;
  }

  for (const int64_t axis : axes) {
    if (axis < 0 || axis >= wmx3Api::constants::maxAxes) {
      message = "Invalid axis " + std::to_string(axis) + ": must be in [0, " +
        std::to_string(wmx3Api::constants::maxAxes) + ").";
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      return ErrorCode::ArgumentOutOfRange;
    }
  }

  axisSel_.axisCount = static_cast<int>(axes.size());
  for (size_t i = 0; i < axes.size(); ++i) {
    axisSel_.axis[i] = static_cast<int>(axes[i]);
  }

  message = "Driving " + std::to_string(axes.size()) + " axes";
  return ErrorCode::None;
}

int JointPositionStreamerApi::getStatus(
  const std::vector<int> & axes, std::vector<double> & posCmd, double & cycleTimeMs,
  std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  CoreMotionStatus status;
  const int err = cm_.GetStatus(&status);
  if (err != ErrorCode::None) {
    message = failure("GetStatus", err);
    return err;
  }

  cycleTimeMs = status.cycleTimeMilliseconds[0];

  posCmd.clear();
  posCmd.reserve(axes.size());
  for (const int axis : axes) {
    if (!status.axesStatus[axis].servoOn || status.axesStatus[axis].ampAlarm) {
      message = "Axis " + std::to_string(axis) + " not ready (servoOn=" +
        std::to_string(status.axesStatus[axis].servoOn) + ", ampAlarm=" +
        std::to_string(status.axesStatus[axis].ampAlarm) + ")";
      return ErrorCode::ArgumentOutOfRange;
    }
    if (status.axesStatus[axis].opState != wmx3Api::OperationState::Idle) {
      message = "Axis " + std::to_string(axis) + " is busy with another motion (opState=" +
        std::to_string(static_cast<int>(status.axesStatus[axis].opState)) + ")";
      return ErrorCode::ArgumentOutOfRange;
    }
    posCmd.push_back(status.axesStatus[axis].posCmd);
  }

  return ErrorCode::None;
}

int JointPositionStreamerApi::openCyclicBuffer(unsigned int numOfCycles, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (bufferOpen_) {
    return ErrorCode::None;
  }

  const int err = cyc_.OpenCyclicBuffer(&axisSel_, numOfCycles);
  if (err != ErrorCode::None) {
    message = failure("OpenCyclicBuffer", err);
    return err;
  }

  bufferOpen_ = true;
  return ErrorCode::None;
}

int JointPositionStreamerApi::execute(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = cyc_.Execute(&axisSel_);
  if (err != ErrorCode::None) {
    message = failure("Execute", err);
    return err;
  }
  return ErrorCode::None;
}

int JointPositionStreamerApi::addCommand(
  const std::vector<int> & axes, const std::vector<std::vector<double>> & samples,
  unsigned int intervalCycles, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (samples.empty()) {
    return ErrorCode::None;
  }

  std::vector<CyclicBufferMultiAxisCommands> cmds(samples.size());
  for (size_t k = 0; k < samples.size(); ++k) {
    for (size_t i = 0; i < axes.size(); ++i) {
      auto & cmd = cmds[k].cmd[axes[i]];
      cmd.type = CyclicBufferCommandType::AbsolutePos;
      cmd.command = samples[k][i];
      cmd.intervalCycles = intervalCycles;
    }
  }

  const int err = cyc_.AddCommand(&axisSel_, cmds.data(), static_cast<int>(cmds.size()));
  if (err != ErrorCode::None) {
    message = failure("AddCommand", err);
    return err;
  }
  return ErrorCode::None;
}

int JointPositionStreamerApi::getCyclicBufferStatus(
  const std::vector<int> & axes, unsigned int & remainCount, unsigned int & availableCount,
  CyclicBufferState::T & state, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  CyclicBufferMultiAxisStatus status;
  const int err = cyc_.GetStatus(&axisSel_, &status);
  if (err != ErrorCode::None) {
    message = failure("GetStatus", err);
    return err;
  }

  remainCount = 0;
  availableCount = static_cast<unsigned int>(-1);
  state = CyclicBufferState::WaitingForCommand;
  for (const int axis : axes) {
    const auto & s = status.status[axis];
    remainCount = std::max(remainCount, s.remainCount);
    availableCount = std::min(availableCount, s.availableCount);
    const bool healthy = s.state == CyclicBufferState::ExecutingCommand ||
      s.state == CyclicBufferState::WaitingForCommand;
    const bool upgrade = s.state == CyclicBufferState::ExecutingCommand &&
      state == CyclicBufferState::WaitingForCommand;
    if (!healthy || upgrade) {
      state = s.state;
    }
  }
  return ErrorCode::None;
}

int JointPositionStreamerApi::abort(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!bufferOpen_) {
    return ErrorCode::None;
  }

  const int err = cyc_.Abort(&axisSel_);
  if (err != ErrorCode::None) {
    message = failure("Abort", err);
    return err;
  }
  return ErrorCode::None;
}

int JointPositionStreamerApi::closeCyclicBuffer(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);
  return closeCyclicBufferLocked(message);
}

int JointPositionStreamerApi::closeCyclicBufferLocked(std::string & message)
{
  if (!bufferOpen_) {
    return ErrorCode::None;
  }

  const int err = cyc_.CloseCyclicBuffer(&axisSel_);
  bufferOpen_ = false;
  if (err != ErrorCode::None) {
    message = failure("CloseCyclicBuffer", err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }
  return ErrorCode::None;
}

JointPositionStreamer::JointPositionStreamer(
  const std::string & nodeName, const std::string & deviceName,
  const std::vector<std::string> & defaultBlockTopics)
: LifecycleNode(nodeName), nodeName_(nodeName)
{
  setRosParameter(defaultBlockTopics);

  api_ = std::make_unique<JointPositionStreamerApi>(this->get_logger(), deviceName);

  RCLCPP_INFO(
    this->get_logger(), "%s is unconfigured, waiting for configure...", nodeName_.c_str());
}

JointPositionStreamer::~JointPositionStreamer()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "%s stopped", nodeName_.c_str());
}

void JointPositionStreamer::setRosParameter(
  const std::vector<std::string> & defaultBlockTopics)
{
  jointAxes_ = this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  jointNames_ = this->declare_parameter<std::vector<std::string>>(
    "joint_name", {"j1", "j2", "j3", "j4", "j5", "j6"});
  bufferCycles_ = this->declare_parameter<int>("buffer_cycles", 2048);
  cycleTime_ = this->declare_parameter<double>("cycle_time", 0.001);
  sampleCycles_ = this->declare_parameter<int>("sample_cycles", 1);
  leadTime_ = this->declare_parameter<double>("lead_time", 0.025);
  streamTimeout_ = this->declare_parameter<double>("stream_timeout", 0.1);
  stopDeceleration_ = this->declare_parameter<double>("stop_deceleration", 2.0);
  interpolationName_ = this->declare_parameter<std::string>("interpolation", "cubic");
  maxVelocity_ = this->declare_parameter<std::vector<double>>(
    "max_velocity", std::vector<double>{2.0});
  maxAcceleration_ = this->declare_parameter<std::vector<double>>(
    "max_acceleration", std::vector<double>{2.0});
  blockTopics_ = defaultBlockTopics;

  if (bufferCycles_ < 64) {
    RCLCPP_WARN(this->get_logger(), "buffer_cycles %d too small, using 2048", bufferCycles_);
    bufferCycles_ = 2048;
  }
  if (!std::isfinite(cycleTime_) || cycleTime_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "cycle_time %f invalid, using 0.001", cycleTime_);
    cycleTime_ = 0.001;
  }
  if (sampleCycles_ < 1) {
    RCLCPP_WARN(this->get_logger(), "sample_cycles %d must be >= 1, using 1", sampleCycles_);
    sampleCycles_ = 1;
  }
  if (!std::isfinite(leadTime_) || leadTime_ < 0.0) {
    RCLCPP_WARN(this->get_logger(), "lead_time %f invalid, using 0.025", leadTime_);
    leadTime_ = 0.025;
  }
  if (!std::isfinite(streamTimeout_) || streamTimeout_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "stream_timeout %f invalid, using 0.1", streamTimeout_);
    streamTimeout_ = 0.1;
  }
  if (!std::isfinite(stopDeceleration_) || stopDeceleration_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(), "stop_deceleration %f invalid, using 2.0", stopDeceleration_);
    stopDeceleration_ = 2.0;
  }

  if (interpolationName_ == "linear") {
    interpolation_ = Interpolation::LINEAR;
  } else if (interpolationName_ == "quintic") {
    interpolation_ = Interpolation::QUINTIC;
  } else {
    if (interpolationName_ != "cubic") {
      RCLCPP_WARN(
        this->get_logger(), "interpolation '%s' unknown, using cubic",
        interpolationName_.c_str());
    }
    interpolationName_ = "cubic";
    interpolation_ = Interpolation::CUBIC;
  }

  if (maxVelocity_.size() == 1) {
    maxVelocity_.assign(jointAxes_.size(), maxVelocity_.front());
  }
  if (maxVelocity_.size() != jointAxes_.size()) {
    RCLCPP_WARN(
      this->get_logger(), "max_velocity has %zu values for %zu axes, disabling the limit",
      maxVelocity_.size(), jointAxes_.size());
    maxVelocity_.assign(jointAxes_.size(), 0.0);
  }
  if (maxAcceleration_.size() == 1) {
    maxAcceleration_.assign(jointAxes_.size(), maxAcceleration_.front());
  }
  if (maxAcceleration_.size() != jointAxes_.size()) {
    RCLCPP_WARN(
      this->get_logger(), "max_acceleration has %zu values for %zu axes, disabling the limit",
      maxAcceleration_.size(), jointAxes_.size());
    maxAcceleration_.assign(jointAxes_.size(), 0.0);
  }

  axes_.clear();
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    axes_.push_back(static_cast<int>(jointAxes_[i]));
  }
  for (size_t i = 0; i < jointNames_.size() && i < jointAxes_.size(); ++i) {
    indexByName_[jointNames_[i]] = i;
  }

  std::string jointAxesText;
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    if (i > 0) {jointAxesText += ", ";}
    jointAxesText += std::to_string(jointAxes_[i]);
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", jointAxesText.c_str());
  RCLCPP_INFO(this->get_logger(), "buffer_cycles: %d", bufferCycles_);
  RCLCPP_INFO(this->get_logger(), "cycle_time: %f", cycleTime_);
  RCLCPP_INFO(this->get_logger(), "sample_cycles: %d", sampleCycles_);
  RCLCPP_INFO(this->get_logger(), "lead_time: %f", leadTime_);
  RCLCPP_INFO(this->get_logger(), "stream_timeout: %f", streamTimeout_);
  RCLCPP_INFO(this->get_logger(), "stop_deceleration: %f", stopDeceleration_);
  RCLCPP_INFO(this->get_logger(), "interpolation: %s", interpolationName_.c_str());
  std::string accText;
  std::string velText;
  for (size_t i = 0; i < maxVelocity_.size(); ++i) {
    if (i > 0) {velText += ", ";}
    velText += std::to_string(maxVelocity_[i]);
  }
  RCLCPP_INFO(this->get_logger(), "max_velocity: [%s]", velText.c_str());
  for (size_t i = 0; i < maxAcceleration_.size(); ++i) {
    if (i > 0) {accText += ", ";}
    accText += std::to_string(maxAcceleration_[i]);
  }
  RCLCPP_INFO(this->get_logger(), "max_acceleration: [%s]", accText.c_str());
  RCLCPP_INFO(this->get_logger(), "===========================");
}

JointPositionStreamer::CallbackReturn JointPositionStreamer::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring %s...", nodeName_.c_str());

  std::string message;
  if (api_->createDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  if (api_->setAxisSelection(jointAxes_, message) != ErrorCode::None) {
    api_->closeDevice();
    return CallbackReturn::FAILURE;
  }

  std::vector<double> posCmd;
  double cycleTimeMs = 0.0;
  api_->getStatus({}, posCmd, cycleTimeMs, message);
  const double engineCycleTime = cycleTimeMs * 1e-3;
  if (engineCycleTime <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(), "Engine cycle time unavailable, using cycle_time %.3f ms",
      cycleTime_ * 1e3);
  } else if (std::fabs(engineCycleTime - cycleTime_) > 1e-6) {
    RCLCPP_WARN(
      this->get_logger(),
      "cycle_time %.3f ms differs from the engine's %.3f ms, using the engine's",
      cycleTime_ * 1e3, engineCycleTime * 1e3);
    cycleTime_ = engineCycleTime;
  }
  RCLCPP_INFO(this->get_logger(), "Engine cycle time: %.3f ms", cycleTime_ * 1e3);

  streamActivePub_ = this->create_publisher<std_msgs::msg::Bool>(
    "~/stream_active", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(this->get_logger(), "%s is configured", nodeName_.c_str());
  return CallbackReturn::SUCCESS;
}

JointPositionStreamer::CallbackReturn JointPositionStreamer::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  inExecution_ = false;
  blockFlags_.assign(blockTopics_.size(), false);
  for (size_t i = 0; i < blockTopics_.size(); ++i) {
    blockSubs_.push_back(
      this->create_subscription<std_msgs::msg::Bool>(
        blockTopics_[i], rclcpp::QoS(1).transient_local(),
        [this, i](const std_msgs::msg::Bool::SharedPtr msg) {blockCallback(i, msg);}));
  }

  watchdogTimer_ = this->create_wall_timer(
    10ms, std::bind(&JointPositionStreamer::watchdogCallback, this));

  LifecycleNode::on_activate(previous_state);
  publishStreamActive(false);

  RCLCPP_INFO(this->get_logger(), "%s is active", nodeName_.c_str());
  return CallbackReturn::SUCCESS;
}

JointPositionStreamer::CallbackReturn JointPositionStreamer::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  watchdogTimer_.reset();
  blockSubs_.clear();

  stopStream("deactivating");
  publishStreamActive(false);

  LifecycleNode::on_deactivate(previous_state);

  RCLCPP_INFO(this->get_logger(), "%s is inactive", nodeName_.c_str());
  return CallbackReturn::SUCCESS;
}

JointPositionStreamer::CallbackReturn JointPositionStreamer::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  api_->closeDevice();
  streamActivePub_.reset();

  RCLCPP_INFO(this->get_logger(), "%s is cleaned up", nodeName_.c_str());
  return CallbackReturn::SUCCESS;
}

JointPositionStreamer::CallbackReturn JointPositionStreamer::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(previous_state);
  }

  return on_cleanup(previous_state);
}

void JointPositionStreamer::publishStreamActive(bool active)
{
  if (!streamActivePub_ || !streamActivePub_->is_activated()) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = active;
  streamActivePub_->publish(msg);
}

void JointPositionStreamer::blockCallback(
  size_t index, const std_msgs::msg::Bool::SharedPtr msg)
{
  if (index >= blockFlags_.size() || blockFlags_[index] == msg->data) {
    return;
  }
  blockFlags_[index] = msg->data;

  bool any = false;
  for (const bool flag : blockFlags_) {
    any = any || flag;
  }

  if (inExecution_.exchange(any) == any) {
    return;
  }

  if (any) {
    stopStream(("blocked by " + blockTopics_[index]).c_str());
  }

  RCLCPP_INFO(
    this->get_logger(), "Stream commands %s (%s = %s)",
    any ? "blocked" : "allowed", blockTopics_[index].c_str(), msg->data ? "true" : "false");
}

bool JointPositionStreamer::startStream()
{
  std::string message;
  std::vector<double> posCmd;
  double cycleTimeMs = 0.0;

  if (draining_) {
    draining_ = false;
    streaming_ = true;
    return true;
  }

  if (api_->getStatus(axes_, posCmd, cycleTimeMs, message) != ErrorCode::None) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Stream not started: %s", message.c_str());
    return false;
  }

  if (api_->openCyclicBuffer(static_cast<unsigned int>(bufferCycles_), message) !=
    ErrorCode::None ||
    api_->execute(message) != ErrorCode::None)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Stream not started: %s", message.c_str());
    api_->closeCyclicBuffer(message);
    return false;
  }

  last_.position = posCmd;
  last_.velocity.assign(axes_.size(), 0.0);
  last_.acceleration.assign(axes_.size(), 0.0);
  refLast_ = posCmd;
  outPos_ = posCmd;
  outVel_.assign(axes_.size(), 0.0);

  const auto holdCount = static_cast<size_t>(
    std::ceil(leadTime_ / (cycleTime_ * sampleCycles_)));
  if (holdCount > 0) {
    std::vector<std::vector<double>> hold(holdCount, posCmd);
    if (api_->addCommand(axes_, hold, static_cast<unsigned int>(sampleCycles_), message) !=
      ErrorCode::None)
    {
      RCLCPP_WARN(this->get_logger(), "Stream not started: %s", message.c_str());
      api_->closeCyclicBuffer(message);
      return false;
    }
  }

  streaming_ = true;
  draining_ = false;
  publishStreamActive(true);
  RCLCPP_INFO(this->get_logger(), "Servo stream started");
  return true;
}

void JointPositionStreamer::stopStream(const char * reason)
{
  if (!streaming_ && !draining_) {
    return;
  }

  std::string message;
  if (api_->abort(message) != ErrorCode::None) {
    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
  }
  api_->closeCyclicBuffer(message);

  streaming_ = false;
  draining_ = false;
  publishStreamActive(false);
  RCLCPP_INFO(this->get_logger(), "Servo stream stopped: %s", reason);
  onStreamStopped(reason);
}

bool JointPositionStreamer::queueSegment(const JointSample & target, double duration)
{
  size_t count = 0;
  return planSegment(duration, count) && appendSegment(target, count);
}

bool JointPositionStreamer::planSegment(double duration, size_t & count)
{
  std::string message;
  unsigned int remainCount = 0;
  unsigned int availableCount = 0;
  CyclicBufferState::T state = CyclicBufferState::Stopped;

  if (api_->getCyclicBufferStatus(axes_, remainCount, availableCount, state, message) !=
    ErrorCode::None)
  {
    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
    return false;
  }

  if (state != CyclicBufferState::ExecutingCommand &&
    state != CyclicBufferState::WaitingForCommand)
  {
    RCLCPP_ERROR(this->get_logger(), "Cyclic buffer in state %s", stateToString(state));
    stopStream(stateToString(state));
    return false;
  }

  const double depth = static_cast<double>(remainCount) * sampleCycles_ * cycleTime_;
  const double corrected = std::clamp(
    duration - 0.5 * (depth - leadTime_), 0.5 * duration, 1.5 * duration);

  const double sampleTime = this->sampleTime();
  count = static_cast<size_t>(std::max(1.0, std::round(corrected / sampleTime)));

  if (streaming_ && remainCount == 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Cyclic buffer ran empty, the axes held until this segment");
  }
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "buffer depth %.1f ms, segment %.1f ms (%zu commands)",
    depth * 1e3, static_cast<double>(count) * sampleTime * 1e3, count);

  if (count > availableCount) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Cyclic buffer full (%u free, %zu needed), dropping segment", availableCount, count);
    return false;
  }
  return true;
}

bool JointPositionStreamer::appendSegment(const JointSample & target, size_t count)
{
  std::string message;
  const double T = static_cast<double>(count) * sampleTime();
  const size_t joints = axes_.size();
  std::vector<std::vector<double>> samples(count, std::vector<double>(joints, 0.0));

  for (size_t i = 0; i < joints; ++i) {
    const double p0 = last_.position[i];
    const double p1 = target.position[i];
    const double d = p1 - p0;
    const double v0 = last_.velocity[i] * T;
    const double v1 = target.velocity[i] * T;
    const double a0 = last_.acceleration[i] * T * T;
    const double a1 = target.acceleration[i] * T * T;

    double c1 = 0.0, c2 = 0.0, c3 = 0.0, c4 = 0.0, c5 = 0.0;
    switch (interpolation_) {
      case Interpolation::LINEAR:
        c1 = d;
        break;
      case Interpolation::CUBIC:
        c1 = v0;
        c2 = 3.0 * d - 2.0 * v0 - v1;
        c3 = -2.0 * d + v0 + v1;
        break;
      case Interpolation::QUINTIC:
        c1 = v0;
        c2 = 0.5 * a0;
        c3 = 10.0 * d - 6.0 * v0 - 4.0 * v1 - 1.5 * a0 + 0.5 * a1;
        c4 = -15.0 * d + 8.0 * v0 + 7.0 * v1 + 1.5 * a0 - a1;
        c5 = 6.0 * d - 3.0 * v0 - 3.0 * v1 - 0.5 * a0 + 0.5 * a1;
        break;
    }

    for (size_t k = 0; k < count; ++k) {
      const double s = static_cast<double>(k + 1) / static_cast<double>(count);
      samples[k][i] = p0 + s * (c1 + s * (c2 + s * (c3 + s * (c4 + s * c5))));
    }
  }

  const auto refLast = refLast_;
  const auto outPos = outPos_;
  const auto outVel = outVel_;
  limitAcceleration(samples);

  if (api_->addCommand(axes_, samples, static_cast<unsigned int>(sampleCycles_), message) !=
    ErrorCode::None)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", message.c_str());
    refLast_ = refLast;
    outPos_ = outPos;
    outVel_ = outVel;
    return false;
  }

  last_ = target;
  return true;
}

void JointPositionStreamer::limitAcceleration(std::vector<std::vector<double>> & samples)
{
  const double h = cycleTime_ * sampleCycles_;
  const double tau = 5.0 * h;
  for (auto & sample : samples) {
    for (size_t i = 0; i < sample.size(); ++i) {
      const double ref = sample[i];
      const double vRef = (ref - refLast_[i]) / h;
      const double e = refLast_[i] - outPos_[i];
      refLast_[i] = ref;
      const double a = maxAcceleration_[i];
      if (a <= 0.0) {
        outPos_[i] = ref;
        outVel_[i] = vRef;
        continue;
      }
      const double corr = std::copysign(
        std::min(std::fabs(e) / tau, std::sqrt(2.0 * a * std::fabs(e))), e);
      double bound = std::sqrt(vRef * vRef + 2.0 * a * std::fabs(e));
      if (maxVelocity_[i] > 0.0) {
        bound = std::min(bound, maxVelocity_[i]);
      }
      const double dv = std::clamp(
        std::clamp(vRef + corr, -bound, bound) - outVel_[i], -a * h, a * h);
      outVel_[i] += dv;
      outPos_[i] += outVel_[i] * h;
      sample[i] = outPos_[i];
    }
  }
}

bool JointPositionStreamer::settled() const
{
  for (size_t i = 0; i < outPos_.size(); ++i) {
    if (std::fabs(refLast_[i] - outPos_[i]) > 1e-6 || std::fabs(outVel_[i]) > 1e-5) {
      return false;
    }
  }
  return true;
}

void JointPositionStreamer::queueSettle()
{
  std::vector<std::vector<double>> samples(50, refLast_);
  limitAcceleration(samples);
  std::string message;
  if (api_->addCommand(axes_, samples, static_cast<unsigned int>(sampleCycles_), message) !=
    ErrorCode::None)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", message.c_str());
  }
}

void JointPositionStreamer::queueStop()
{
  double T = 0.05;
  for (const double v : last_.velocity) {
    T = std::max(T, 2.0 * std::fabs(v) / stopDeceleration_);
  }

  JointSample rest;
  rest.position = last_.position;
  rest.velocity.assign(axes_.size(), 0.0);
  rest.acceleration.assign(axes_.size(), 0.0);
  for (size_t i = 0; i < axes_.size(); ++i) {
    rest.position[i] += 0.5 * last_.velocity[i] * T;
  }

  queueSegment(rest, T);
  streaming_ = false;
  draining_ = true;
}

bool JointPositionStreamer::readPoint(
  const trajectory_msgs::msg::JointTrajectory & traj, size_t index, JointSample & sample)
{
  const auto & pt = traj.points[index];
  const size_t count = pt.positions.size();

  if (count != axes_.size() || traj.joint_names.size() != count) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Dropped trajectory: %zu positions, %zu names for %zu axes",
      count, traj.joint_names.size(), axes_.size());
    return false;
  }

  const bool hasVelocity = pt.velocities.size() == count;
  const bool hasAcceleration = pt.accelerations.size() == count;

  sample.position.assign(count, 0.0);
  sample.velocity.assign(count, 0.0);
  sample.acceleration.assign(count, 0.0);

  for (size_t j = 0; j < count; ++j) {
    const auto it = indexByName_.find(traj.joint_names[j]);
    if (it == indexByName_.end()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dropped trajectory: joint '%s' is not in joint_name", traj.joint_names[j].c_str());
      return false;
    }

    const size_t i = it->second;
    sample.position[i] = pt.positions[j];
    sample.velocity[i] = hasVelocity ? pt.velocities[j] : 0.0;
    sample.acceleration[i] = hasAcceleration ? pt.accelerations[j] : 0.0;

    if (!std::isfinite(sample.position[i]) || !std::isfinite(sample.velocity[i]) ||
      !std::isfinite(sample.acceleration[i]))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dropped trajectory: joint '%s' is not finite", traj.joint_names[j].c_str());
      return false;
    }
  }

  return true;
}

void JointPositionStreamer::watchdogCallback()
{
  if (streaming_) {
    const double idle = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - lastReceived_).count();
    if (idle > streamTimeout_) {
      queueStop();
    }
    return;
  }

  if (!draining_) {
    return;
  }

  std::string message;
  unsigned int remainCount = 0;
  unsigned int availableCount = 0;
  CyclicBufferState::T state = CyclicBufferState::Stopped;
  if (api_->getCyclicBufferStatus(axes_, remainCount, availableCount, state, message) !=
    ErrorCode::None)
  {
    stopStream(message.c_str());
    return;
  }

  if (!settled()) {
    if (remainCount < 100) {
      queueSettle();
    }
    return;
  }

  if (remainCount == 0) {
    stopStream("stream idle");
  }
}

JointPositionController::JointPositionController()
: JointPositionStreamer(
    "joint_position_controller", "joint_position_controller",
    {"/moveit2_trajectory/execution_active", "/dynamic_trajectory_controller/stream_active"})
{
  jointTrajectoryTopic_ = this->declare_parameter<std::string>(
    "joint_trajectory_topic", "/joint_trajectory_topic/no_param");
  publishPeriod_ = this->declare_parameter<double>("publish_period", 0.025);
  if (!std::isfinite(publishPeriod_) || publishPeriod_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "publish_period %f invalid, using 0.025", publishPeriod_);
    publishPeriod_ = 0.025;
  }
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_topic: %s", jointTrajectoryTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "publish_period: %f", publishPeriod_);
}

JointPositionController::CallbackReturn JointPositionController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = JointPositionStreamer::on_activate(previous_state);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  jointTrajectorySub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    jointTrajectoryTopic_, 10,
    std::bind(&JointPositionController::jointTrajectoryCallback, this, _1));
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  jointTrajectorySub_.reset();
  return JointPositionStreamer::on_deactivate(previous_state);
}

void JointPositionController::jointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->points.empty() || blocked()) {
    return;
  }

  JointSample sample;
  if (!readPoint(*msg, msg->points.size() - 1, sample)) {
    return;
  }

  lastReceived_ = std::chrono::steady_clock::now();

  if (!streaming_ && !startStream()) {
    return;
  }

  queueSegment(sample, publishPeriod_);
}

DynamicTrajectoryController::DynamicTrajectoryController()
: JointPositionStreamer(
    "dynamic_trajectory_controller", "dynamic_trajectory_controller",
    {"/moveit2_trajectory/execution_active", "/joint_position_controller/stream_active"})
{
  jointTrajectoryTopic_ = this->declare_parameter<std::string>(
    "joint_trajectory_topic", "/joint_trajectory_topic/no_param");
  slicePeriod_ = this->declare_parameter<double>("slice_period", 0.01);
  if (!std::isfinite(slicePeriod_) || slicePeriod_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "slice_period %f invalid, using 0.01", slicePeriod_);
    slicePeriod_ = 0.01;
  }
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_topic: %s", jointTrajectoryTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "slice_period: %f", slicePeriod_);
}

DynamicTrajectoryController::CallbackReturn DynamicTrajectoryController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = JointPositionStreamer::on_activate(previous_state);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  jointTrajectorySub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    jointTrajectoryTopic_, 10,
    std::bind(&DynamicTrajectoryController::jointTrajectoryCallback, this, _1));
  sliceTimer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(slicePeriod_)),
    std::bind(&DynamicTrajectoryController::sliceCallback, this));
  return CallbackReturn::SUCCESS;
}

DynamicTrajectoryController::CallbackReturn DynamicTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  sliceTimer_.reset();
  jointTrajectorySub_.reset();
  const auto ret = JointPositionStreamer::on_deactivate(previous_state);
  clearTrajectory();
  return ret;
}

void DynamicTrajectoryController::onStreamStopped(const char *)
{
  clearTrajectory();
}

void DynamicTrajectoryController::jointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->points.empty() || blocked()) {
    return;
  }

  std::vector<Waypoint> points;
  points.reserve(msg->points.size());
  for (size_t k = 0; k < msg->points.size(); ++k) {
    Waypoint wp;
    if (!readPoint(*msg, k, wp.state)) {
      return;
    }
    wp.time = rclcpp::Duration(msg->points[k].time_from_start).seconds();
    wp.hasVelocity = msg->points[k].velocities.size() == axes_.size();
    if (k > 0 && !(wp.time > points.back().time)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dropped trajectory: time_from_start of point %zu is not increasing", k);
      return;
    }
    points.push_back(std::move(wp));
  }

  if (!streaming_ && !startStream()) {
    return;
  }

  double tau0 = 0.0;
  const rclcpp::Time stamp(msg->header.stamp, this->get_clock()->get_clock_type());
  if (stamp.nanoseconds() != 0) {
    tau0 = (this->now() - stamp).seconds() + leadTime_;
  }

  Waypoint start;
  start.time = tau0;
  start.state = last_;
  start.hasVelocity = true;

  std::vector<Waypoint> trajectory{start};
  for (auto & wp : points) {
    if (wp.time > tau0 + sampleTime()) {
      trajectory.push_back(std::move(wp));
    }
  }
  if (trajectory.size() < 2) {
    points.back().time = tau0 + slicePeriod_;
    trajectory.push_back(std::move(points.back()));
  }

  trajectory_ = std::move(trajectory);
  segment_ = 0;
  tau_ = tau0;
  running_ = true;
}

DynamicTrajectoryController::JointSample DynamicTrajectoryController::sampleAt(double t)
{
  const Waypoint & last = trajectory_.back();
  if (t >= last.time) {
    JointSample hold = last.state;
    hold.velocity.assign(axes_.size(), 0.0);
    hold.acceleration.assign(axes_.size(), 0.0);
    return hold;
  }

  while (trajectory_[segment_ + 1].time <= t) {
    ++segment_;
  }
  const Waypoint & a = trajectory_[segment_];
  const Waypoint & b = trajectory_[segment_ + 1];
  const double h = b.time - a.time;
  const double s = (t - a.time) / h;

  JointSample out;
  out.position.resize(axes_.size());
  out.velocity.resize(axes_.size());
  out.acceleration.assign(axes_.size(), 0.0);
  for (size_t i = 0; i < axes_.size(); ++i) {
    const double p0 = a.state.position[i];
    const double p1 = b.state.position[i];
    out.position[i] = p0 + s * (p1 - p0);
    out.velocity[i] = (a.hasVelocity && b.hasVelocity) ?
      a.state.velocity[i] + s * (b.state.velocity[i] - a.state.velocity[i]) :
      (p1 - p0) / h;
  }
  return out;
}

void DynamicTrajectoryController::sliceCallback()
{
  if (!running_) {
    return;
  }
  lastReceived_ = std::chrono::steady_clock::now();

  size_t count = 0;
  if (!planSegment(slicePeriod_, count)) {
    return;
  }

  const double T = static_cast<double>(count) * sampleTime();
  if (!appendSegment(sampleAt(tau_ + T), count)) {
    return;
  }
  tau_ += T;

  if (tau_ >= trajectory_.back().time) {
    queueStop();
    clearTrajectory();
    RCLCPP_INFO(this->get_logger(), "Trajectory finished, draining");
  }
}

void DynamicTrajectoryController::clearTrajectory()
{
  trajectory_.clear();
  segment_ = 0;
  tau_ = 0.0;
  running_ = false;
}
