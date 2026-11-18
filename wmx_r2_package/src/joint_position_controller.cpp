// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "joint_position_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

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

JointPositionControllerApi::JointPositionControllerApi(const rclcpp::Logger & logger)
: logger_(logger), cm_(&wmx3Lib_), cyc_(&wmx3Lib_)
{
}

JointPositionControllerApi::~JointPositionControllerApi()
{
  closeDevice();
}

int JointPositionControllerApi::createDevice(std::string & message)
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

  err = wmx3Lib_.SetDeviceName(deviceName_);
  if (err != ErrorCode::None) {
    message = "Failed to name the device '" + std::string(deviceName_) + "'. Error=" +
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

void JointPositionControllerApi::closeDevice()
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

int JointPositionControllerApi::setAxisSelection(
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

int JointPositionControllerApi::getStatus(
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
    posCmd.push_back(status.axesStatus[axis].posCmd);
  }

  return ErrorCode::None;
}

int JointPositionControllerApi::openCyclicBuffer(unsigned int numOfCycles, std::string & message)
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

int JointPositionControllerApi::execute(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = cyc_.Execute(&axisSel_);
  if (err != ErrorCode::None) {
    message = failure("Execute", err);
    return err;
  }
  return ErrorCode::None;
}

int JointPositionControllerApi::addCommand(
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

int JointPositionControllerApi::getCyclicBufferStatus(
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

int JointPositionControllerApi::abort(std::string & message)
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

int JointPositionControllerApi::closeCyclicBuffer(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);
  return closeCyclicBufferLocked(message);
}

int JointPositionControllerApi::closeCyclicBufferLocked(std::string & message)
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

JointPositionController::JointPositionController()
: LifecycleNode("joint_position_controller")
{
  setRosParameter();

  api_ = std::make_unique<JointPositionControllerApi>(this->get_logger());

  RCLCPP_INFO(
    this->get_logger(), "joint_position_controller is unconfigured, waiting for configure...");
}

JointPositionController::~JointPositionController()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "joint_position_controller stopped");
}

void JointPositionController::setRosParameter()
{
  jointAxes_ = this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  jointNames_ = this->declare_parameter<std::vector<std::string>>(
    "joint_name", {"j1", "j2", "j3", "j4", "j5", "j6"});
  jointTrajectoryTopic_ = this->declare_parameter<std::string>(
    "joint_trajectory_topic", "/joint_trajectory_topic/no_param");
  bufferCycles_ = this->declare_parameter<int>("buffer_cycles", 2048);
  cycleTime_ = this->declare_parameter<double>("cycle_time", 0.001);
  sampleCycles_ = this->declare_parameter<int>("sample_cycles", 1);
  publishPeriod_ = this->declare_parameter<double>("publish_period", 0.025);
  leadTime_ = this->declare_parameter<double>("lead_time", 0.025);
  streamTimeout_ = this->declare_parameter<double>("stream_timeout", 0.1);
  stopDeceleration_ = this->declare_parameter<double>("stop_deceleration", 2.0);
  interpolationName_ = this->declare_parameter<std::string>("interpolation", "cubic");
  maxAcceleration_ = this->declare_parameter<std::vector<double>>(
    "max_acceleration", std::vector<double>{2.0});

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
  if (!std::isfinite(publishPeriod_) || publishPeriod_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "publish_period %f invalid, using 0.025", publishPeriod_);
    publishPeriod_ = 0.025;
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
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_topic: %s", jointTrajectoryTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "buffer_cycles: %d", bufferCycles_);
  RCLCPP_INFO(this->get_logger(), "cycle_time: %f", cycleTime_);
  RCLCPP_INFO(this->get_logger(), "sample_cycles: %d", sampleCycles_);
  RCLCPP_INFO(this->get_logger(), "publish_period: %f", publishPeriod_);
  RCLCPP_INFO(this->get_logger(), "lead_time: %f", leadTime_);
  RCLCPP_INFO(this->get_logger(), "stream_timeout: %f", streamTimeout_);
  RCLCPP_INFO(this->get_logger(), "stop_deceleration: %f", stopDeceleration_);
  RCLCPP_INFO(this->get_logger(), "interpolation: %s", interpolationName_.c_str());
  std::string accText;
  for (size_t i = 0; i < maxAcceleration_.size(); ++i) {
    if (i > 0) {accText += ", ";}
    accText += std::to_string(maxAcceleration_[i]);
  }
  RCLCPP_INFO(this->get_logger(), "max_acceleration: [%s]", accText.c_str());
  RCLCPP_INFO(this->get_logger(), "===========================");
}

JointPositionController::CallbackReturn JointPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring joint_position_controller...");

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

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is configured");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  execActiveSub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local(),
    std::bind(&JointPositionController::execActiveCallback, this, _1));

  jointTrajectorySub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    jointTrajectoryTopic_, 10,
    std::bind(&JointPositionController::jointTrajectoryCallback, this, _1));

  watchdogTimer_ = this->create_wall_timer(
    10ms, std::bind(&JointPositionController::watchdogCallback, this));

  LifecycleNode::on_activate(previous_state);

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is active");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  watchdogTimer_.reset();
  jointTrajectorySub_.reset();
  execActiveSub_.reset();

  stopStream("deactivating");

  LifecycleNode::on_deactivate(previous_state);

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is inactive");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  api_->closeDevice();

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is cleaned up");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(previous_state);
  }

  return on_cleanup(previous_state);
}

void JointPositionController::execActiveCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (inExecution_.exchange(msg->data) == msg->data) {
    return;
  }

  if (msg->data) {
    stopStream("move_group execution started");
  }

  RCLCPP_INFO(
    this->get_logger(), "Servo commands %s (move_group execution %s)",
    msg->data ? "blocked" : "allowed", msg->data ? "started" : "finished");
}

bool JointPositionController::startStream()
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
  RCLCPP_INFO(this->get_logger(), "Servo stream started");
  return true;
}

void JointPositionController::stopStream(const char * reason)
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
  RCLCPP_INFO(this->get_logger(), "Servo stream stopped: %s", reason);
}

bool JointPositionController::queueSegment(const JointSample & target, double duration)
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

  const double sampleTime = cycleTime_ * sampleCycles_;
  const auto count = static_cast<size_t>(std::max(1.0, std::round(corrected / sampleTime)));

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

  const double T = static_cast<double>(count) * sampleTime;
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

void JointPositionController::limitAcceleration(std::vector<std::vector<double>> & samples)
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
      const double bound = std::sqrt(vRef * vRef + 2.0 * a * std::fabs(e));
      const double dv = std::clamp(
        std::clamp(vRef + corr, -bound, bound) - outVel_[i], -a * h, a * h);
      outVel_[i] += dv;
      outPos_[i] += outVel_[i] * h;
      sample[i] = outPos_[i];
    }
  }
}

bool JointPositionController::settled() const
{
  for (size_t i = 0; i < outPos_.size(); ++i) {
    if (std::fabs(refLast_[i] - outPos_[i]) > 1e-6 || std::fabs(outVel_[i]) > 1e-5) {
      return false;
    }
  }
  return true;
}

// After the stop segment, holds the reference until the limited output has caught up.
void JointPositionController::queueSettle()
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

void JointPositionController::queueStop()
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

bool JointPositionController::readPoint(
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

void JointPositionController::jointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->points.empty() || inExecution_.load()) {
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

void JointPositionController::watchdogCallback()
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointPositionController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
