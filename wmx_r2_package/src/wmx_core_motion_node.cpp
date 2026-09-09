// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_core_motion_node.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

#include "wmx_qos_compat.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using wmx3Api::AxisCommandMode;
using wmx3Api::Config;
using wmx3Api::CoreMotion;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;

namespace
{
std::string valueText(double value)
{
  std::ostringstream stream;
  stream << std::setprecision(15) << value;
  return stream.str();
}

std::chrono::nanoseconds periodFromRate(int rate)
{
  return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / static_cast<double>(rate)));
}
}  // namespace

WmxCoreMotionNodeApi::WmxCoreMotionNodeApi(
  const rclcpp::Logger & logger, const Config & config)
: logger_(logger), config_(config), cm_(&wmx3Lib_)
{
}

WmxCoreMotionNodeApi::~WmxCoreMotionNodeApi()
{
  closeDevice();
}

std::string WmxCoreMotionNodeApi::errorToString(int err)
{
  char errString[256] = {};
  wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
  return errString;
}

int WmxCoreMotionNodeApi::createDevice(std::string & message)
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

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void WmxCoreMotionNodeApi::closeDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorToString(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "Device closed");
  }
}

int WmxCoreMotionNodeApi::getNumOfAxes()
{
  wmx3Api::EngineStatus engineStatus;
  wmx3Lib_.GetEngineStatus(&engineStatus);

  int axisCount = 0;
  for (int i = 0; i < engineStatus.numOfInterrupts; ++i) {
    axisCount += engineStatus.interrupts[i].numOfAxes;
  }
  return axisCount;
}

int WmxCoreMotionNodeApi::getStatus(wmx3Api::CoreMotionStatus & status)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  return cm_.GetStatus(&status);
}

int WmxCoreMotionNodeApi::startPos(
  int axis, double target, double velocity, double acc, double dec, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  wmx3Api::Motion::PosCommand position;
  position.axis = axis;
  position.target = target;
  position.profile.velocity = velocity;
  position.profile.type = ProfileType::T::Trapezoidal;
  position.profile.acc = acc;
  position.profile.dec = dec;

  const int err = cm_.motion->StartPos(&position);
  if (err != ErrorCode::None) {
    message = "Failed to move position motor " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Moving axis " + std::to_string(axis) + " to " + std::to_string(target);
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::startMov(
  int axis, double target, double velocity, double acc, double dec, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  wmx3Api::Motion::PosCommand position;
  position.axis = axis;
  position.target = target;
  position.profile.velocity = velocity;
  position.profile.type = ProfileType::T::Trapezoidal;
  position.profile.acc = acc;
  position.profile.dec = dec;

  const int err = cm_.motion->StartMov(&position);
  if (err != ErrorCode::None) {
    message = "Failed to move relative motor " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Moving axis " + std::to_string(axis) + " by " + std::to_string(target);
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::startVel(
  int axis, double velocity, double acc, double dec, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  wmx3Api::Velocity::VelCommand command;
  command.axis = axis;
  command.profile.velocity = velocity;
  command.profile.type = ProfileType::T::Trapezoidal;
  command.profile.acc = acc;
  command.profile.dec = dec;

  const int err = cm_.velocity->StartVel(&command);
  if (err != ErrorCode::None) {
    message = "Failed to move velocity motor " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Axis " + std::to_string(axis) + " running at " + std::to_string(velocity);
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::startJog(
  int axis, double velocity, double acc, double dec, const rclcpp::Time & now,
  std::string & message)
{
  const rclcpp::Time deadline =
    now + rclcpp::Duration::from_nanoseconds(
    static_cast<int64_t>(config_.jogTimeoutMs * 1e6));

  if (velocity == 0.0) {
    bool wasJogging = false;
    {
      std::lock_guard<std::mutex> jogLock(jogMutex_);
      wasJogging = jogState_.erase(axis) > 0;
    }
    if (wasJogging) {
      std::string stopMessage;
      stop(axis, stopMessage);
    }
    message = "Stopped jog on axis " + std::to_string(axis);
    return ErrorCode::None;
  }

  {
    std::lock_guard<std::mutex> jogLock(jogMutex_);
    auto it = jogState_.find(axis);
    if (it != jogState_.end() && it->second.velocity == velocity) {
      it->second.deadline = deadline;
      message = "Refreshed jog on axis " + std::to_string(axis);
      return ErrorCode::None;
    }
  }

  const double accTimeMs = (acc > 0.0) ? std::abs(velocity) / acc * 1000.0 : 0.0;
  const double decTimeMs = (dec > 0.0) ? std::abs(velocity) / dec * 1000.0 : 0.0;

  {
    std::lock_guard<std::mutex> lock(deviceMutex_);

    wmx3Api::Motion::JogCommand jogCommand = wmx3Api::Motion::JogCommand();
    jogCommand.axis = axis;
    jogCommand.profile.type = ProfileType::T::TimeAccJerkRatio;
    jogCommand.profile.velocity = velocity;
    jogCommand.profile.acc = 0;
    jogCommand.profile.dec = 0;
    jogCommand.profile.jerkAcc = 0;
    jogCommand.profile.jerkDec = 0;
    jogCommand.profile.jerkAccRatio = config_.jogJerkRatio;
    jogCommand.profile.jerkDecRatio = config_.jogJerkRatio;
    jogCommand.profile.accTimeMilliseconds = accTimeMs;
    jogCommand.profile.decTimeMilliseconds = decTimeMs;
    jogCommand.profile.startingVelocity = 0;
    jogCommand.profile.endVelocity = 0;
    jogCommand.profile.secondVelocity = 0;
    jogCommand.profile.movingAverageTimeMilliseconds = 0;
    jogCommand.SetRunTime(config_.jogRunTimeMs);

    const int err = cm_.motion->StartJog(&jogCommand);
    if (err != ErrorCode::None) {
      message = "Failed to jog motor " + std::to_string(axis) + ". Error=" +
        std::to_string(err) + " (" + errorToString(err) + ")";
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      return err;
    }
  }

  std::lock_guard<std::mutex> jogLock(jogMutex_);
  jogState_.insert_or_assign(axis, JogState{velocity, deadline});
  message = "Jogging axis " + std::to_string(axis);
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::stopExpiredJogs(const rclcpp::Time & now, std::string & message)
{
  std::vector<int> expired;

  {
    std::lock_guard<std::mutex> lock(jogMutex_);
    for (auto it = jogState_.begin(); it != jogState_.end(); ) {
      if (now >= it->second.deadline) {
        expired.push_back(it->first);
        it = jogState_.erase(it);
      } else {
        ++it;
      }
    }
  }

  if (expired.empty()) {
    message = "No expired jog";
    return ErrorCode::None;
  }

  int firstError = ErrorCode::None;
  std::stringstream stream;

  for (const int axis : expired) {
    std::string stopMessage;
    const int err = stop(axis, stopMessage);
    if (err != ErrorCode::None && firstError == ErrorCode::None) {
      firstError = err;
    }
    stream << stopMessage << "; ";
  }

  message = stream.str();
  return firstError;
}

int WmxCoreMotionNodeApi::clearJog(int axis, std::string & message)
{
  std::lock_guard<std::mutex> lock(jogMutex_);

  const bool cleared = jogState_.erase(axis) > 0;
  message = cleared ?
    "Cleared jog on axis " + std::to_string(axis) :
    "No jog to clear on axis " + std::to_string(axis);
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::stopAllJogs(std::string & message)
{
  std::vector<int> jogging;

  {
    std::lock_guard<std::mutex> lock(jogMutex_);
    for (const auto & entry : jogState_) {
      jogging.push_back(entry.first);
    }
    jogState_.clear();
  }

  if (jogging.empty()) {
    message = "No jog to stop";
    return ErrorCode::None;
  }

  int firstError = ErrorCode::None;
  std::stringstream stream;

  for (const int axis : jogging) {
    std::string stopMessage;
    const int err = stop(axis, stopMessage);
    if (err != ErrorCode::None && firstError == ErrorCode::None) {
      firstError = err;
    }
    stream << stopMessage << "; ";
  }

  message = stream.str();
  return firstError;
}

int WmxCoreMotionNodeApi::stop(int axis, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = cm_.motion->Stop(axis);
  if (err != ErrorCode::None) {
    message = "Failed to stop axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Stopped axis " + std::to_string(axis);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::setServoOn(int axis, int newStatus, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = cm_.axisControl->SetServoOn(axis, newStatus, servoOnTimeout_);
  const std::string onOff = newStatus ? "on" : "off";

  if (err != ErrorCode::None) {
    message = "Failed to set axis " + std::to_string(axis) + " " + onOff + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set axis " + std::to_string(axis) + " " + onOff;
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::setAxisCommandMode(int axis, int mode, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (mode != 0 && mode != 1) {
    message = "Invalid mode " + std::to_string(mode) + " for axis " + std::to_string(axis);
    RCLCPP_WARN(logger_, "%s", message.c_str());
    return ErrorCode::ArgumentOutOfRange;
  }

  const AxisCommandMode::T commandMode =
    (mode == 0) ? AxisCommandMode::Position : AxisCommandMode::Velocity;

  const int err = cm_.axisControl->SetAxisCommandMode(axis, commandMode);
  if (err != ErrorCode::None) {
    message = "Failed to set axis " + std::to_string(axis) + " mode " + std::to_string(mode) +
      ". Error=" + std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set axis " + std::to_string(axis) + " in " +
    ((mode == 0) ? "Position" : "Velocity") + " mode";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::clearAmpAlarm(int axis, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = cm_.axisControl->ClearAmpAlarm(axis);
  if (err != ErrorCode::None) {
    message = "Failed to clear alarm axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Cleared alarm axis " + std::to_string(axis);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::setAxisPolarity(int axis, int polarity, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (polarity != 1 && polarity != -1) {
    message = "Invalid polarity value for axis " + std::to_string(axis) + ": " +
      std::to_string(polarity);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return ErrorCode::ArgumentOutOfRange;
  }

  const int err = cm_.config->SetAxisPolarity(axis, polarity);
  if (err != ErrorCode::None) {
    message = "Failed to set polarity on axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set polarity on axis " + std::to_string(axis);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::setGearRatio(
  int axis, double numerator, double denominator, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = cm_.config->SetGearRatio(axis, numerator, denominator);
  if (err != ErrorCode::None) {
    message = "Failed to set gear ratio on axis " + std::to_string(axis) + " to " +
      valueText(numerator) + " / " + valueText(denominator) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set gear ratio on axis " + std::to_string(axis) + " to " +
    valueText(numerator) + " / " + valueText(denominator);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::startHome(int axis, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  wmx3Api::Config::HomeParam homeParam;

  int err = cm_.config->GetHomeParam(axis, &homeParam);
  if (err != ErrorCode::None) {
    message = "Failed to read home param for axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  homeParam.homeType = wmx3Api::Config::HomeType::CurrentPos;

  err = cm_.config->SetHomeParam(axis, &homeParam);
  if (err != ErrorCode::None) {
    message = "Failed to set home param for axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = cm_.home->StartHome(axis);
  if (err != ErrorCode::None) {
    message = "Failed to start homing axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Homing started on axis " + std::to_string(axis);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

WmxCoreMotionNode::WmxCoreMotionNode()
: LifecycleNode("wmx_core_motion_node")
{
  WmxCoreMotionNodeApi::Config config;
  config.jogTimeoutMs = this->declare_parameter("jog_timeout_ms", 200.0);
  config.jogRunTimeMs = this->declare_parameter("jog_run_time_ms", 2000.0);
  config.jogJerkRatio = this->declare_parameter("jog_jerk_ratio", 0.75);

  api_ = std::make_unique<WmxCoreMotionNodeApi>(this->get_logger(), config);

  rate_ = this->declare_parameter("axes_status_rate", 100);
  if (rate_ <= 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "axes_status_rate must be > 0, got %d. Falling back to %d Hz.", rate_, 100);
    rate_ = 100;
  }

  motionControllers_ = this->declare_parameter<std::vector<std::string>>(
    "motion_controllers",
    std::vector<std::string>{
    "joint_trajectory_controller",
    "differential_drive_controller",
    "joint_position_controller"});

  controllerResyncPeriod_ = this->declare_parameter("controller_resync_period", 0.2);
  if (controllerResyncPeriod_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "controller_resync_period must be > 0, got %f. Falling back to 0.2 s.",
      controllerResyncPeriod_);
    controllerResyncPeriod_ = 0.2;
  }

  homing_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  clientCbGroup_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  RCLCPP_INFO(
    this->get_logger(), "wmx_core_motion_node is unconfigured, waiting for configure...");
}

WmxCoreMotionNode::~WmxCoreMotionNode()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node stopped");
}

bool WmxCoreMotionNode::isMotionBlocked() const
{
  std::lock_guard<std::mutex> lock(controllerMutex_);

  for (const auto & entry : controllerActive_) {
    if (entry.second) {
      return true;
    }
  }
  return false;
}

void WmxCoreMotionNode::setControllerActive(const std::string & controller, bool active)
{
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(controllerMutex_);
    changed = controllerActive_[controller] != active;
    controllerActive_[controller] = active;
  }

  if (!changed) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "%s is now %s. Motion commands are %s.",
    controller.c_str(), active ? "active" : "inactive",
    isMotionBlocked() ? "blocked" : "allowed");
}

void WmxCoreMotionNode::transitionEventCallback(
  const std::string & controller,
  const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
  switch (msg->goal_state.id) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      setControllerActive(controller, true);
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
      setControllerActive(controller, false);
      break;
    default:
      break;
  }
}

void WmxCoreMotionNode::resyncControllerStates()
{
  for (const auto & entry : getStateClients_) {
    const std::string controller = entry.first;
    const auto & client = entry.second;

    if (!client->service_is_ready()) {
      setControllerActive(controller, false);
      continue;
    }

    client->prune_pending_requests();

    client->async_send_request(
      std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
      [this, controller](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
        setControllerActive(
          controller,
          future.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      });
  }
}

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring wmx_core_motion_node...");

  std::string message;
  if (api_->createDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  numOfAxes_ = api_->getNumOfAxes();
  if (numOfAxes_ <= 0) {
    RCLCPP_WARN(this->get_logger(), "Engine reported 0 axes; axis state will be empty.");
  }

  RCLCPP_INFO(
    this->get_logger(), "wmx_core_motion_node is configured (%d axes, %d Hz)",
    numOfAxes_, rate_);
  return CallbackReturn::SUCCESS;
}

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  setServoOnService_ = this->create_service<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/set_servo_on",
    std::bind(&WmxCoreMotionNode::setServoOnCallback, this, _1, _2));

  clearAmpAlarmService_ = this->create_service<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/clear_amp_alarm",
    std::bind(&WmxCoreMotionNode::clearAmpAlarmCallback, this, _1, _2));

  setAxisCommandModeService_ = this->create_service<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/set_axis_command_mode",
    std::bind(&WmxCoreMotionNode::setAxisCommandModeCallback, this, _1, _2));

  setAxisPolarityService_ = this->create_service<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/set_axis_polarity",
    std::bind(&WmxCoreMotionNode::setAxisPolarityCallback, this, _1, _2));

  setGearRatioService_ = this->create_service<wmx_r2_message::srv::SetAxesGearRatio>(
    "wmx/axes/set_gear_ratio",
    std::bind(&WmxCoreMotionNode::setGearRatioCallback, this, _1, _2));

  startHomeService_ = this->create_service<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/start_home",
    std::bind(&WmxCoreMotionNode::startHomeCallback, this, _1, _2),
    servicesQos(), homing_cb_group_);

  stopService_ = this->create_service<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/stop",
    std::bind(&WmxCoreMotionNode::stopCallback, this, _1, _2));

  axesStatusPub_ = this->create_publisher<wmx_r2_message::msg::AxesStatus>(
    "wmx/axes/status", 1);

  startPosService_ = this->create_service<wmx_r2_message::srv::StartAxesPose>(
    "wmx/axes/start_pos",
    std::bind(&WmxCoreMotionNode::startPosCallback, this, _1, _2));

  startMovService_ = this->create_service<wmx_r2_message::srv::StartAxesPose>(
    "wmx/axes/start_mov",
    std::bind(&WmxCoreMotionNode::startMovCallback, this, _1, _2));

  startVelService_ = this->create_service<wmx_r2_message::srv::StartAxesVelocity>(
    "wmx/axes/start_vel",
    std::bind(&WmxCoreMotionNode::startVelCallback, this, _1, _2));

  startJogService_ = this->create_service<wmx_r2_message::srv::StartAxesVelocity>(
    "wmx/axes/start_jog",
    std::bind(&WmxCoreMotionNode::startJogCallback, this, _1, _2));

  for (const std::string & controller : motionControllers_) {
    if (controller.empty() || controller == this->get_name()) {
      continue;
    }

    setControllerActive(controller, false);

    getStateClients_[controller] = this->create_client<lifecycle_msgs::srv::GetState>(
      "/" + controller + "/get_state", servicesQos(), clientCbGroup_);

    transitionEventSubs_.push_back(
      this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
        "/" + controller + "/transition_event", 10,
        [this, controller](const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
          this->transitionEventCallback(controller, msg);
        }));

    RCLCPP_INFO(this->get_logger(), "Watching '%s' for motion arbitration", controller.c_str());
  }

  controllerResyncTimer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(controllerResyncPeriod_)),
    std::bind(&WmxCoreMotionNode::resyncControllerStates, this));

  LifecycleNode::on_activate(previous_state);

  axesStatusTimer_ = this->create_wall_timer(
    periodFromRate(rate_),
    std::bind(&WmxCoreMotionNode::axesStatusStep, this));

  jogWatchdogTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&WmxCoreMotionNode::jogWatchdogStep, this));

  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node is active");
  return CallbackReturn::SUCCESS;
}

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  axesStatusTimer_.reset();
  jogWatchdogTimer_.reset();
  std::string jogMessage;
  api_->stopAllJogs(jogMessage);

  LifecycleNode::on_deactivate(previous_state);

  controllerResyncTimer_.reset();
  transitionEventSubs_.clear();
  getStateClients_.clear();
  {
    std::lock_guard<std::mutex> lock(controllerMutex_);
    controllerActive_.clear();
  }

  startPosService_.reset();
  startMovService_.reset();
  startVelService_.reset();
  startJogService_.reset();

  setServoOnService_.reset();
  clearAmpAlarmService_.reset();
  setAxisCommandModeService_.reset();
  setAxisPolarityService_.reset();
  setGearRatioService_.reset();
  startHomeService_.reset();
  stopService_.reset();

  axesStatusPub_.reset();

  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node is inactive");
  return CallbackReturn::SUCCESS;
}

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  std::string jogMessage;
  api_->stopAllJogs(jogMessage);

  api_->closeDevice();

  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node is cleaned up");
  return CallbackReturn::SUCCESS;
}

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void WmxCoreMotionNode::axesStatusStep()
{
  wmx3Api::CoreMotionStatus cmStatus;
  if (api_->getStatus(cmStatus) != ErrorCode::None) {
    return;
  }

  axesStatusMsg_.amp_alarm.clear();
  axesStatusMsg_.servo_on.clear();
  axesStatusMsg_.home_done.clear();
  axesStatusMsg_.motion_complete.clear();
  axesStatusMsg_.negative_ls.clear();
  axesStatusMsg_.positive_ls.clear();
  axesStatusMsg_.home_switch.clear();
  axesStatusMsg_.pos_cmd.clear();
  axesStatusMsg_.velocity_cmd.clear();
  axesStatusMsg_.actual_pos.clear();
  axesStatusMsg_.actual_velocity.clear();
  axesStatusMsg_.actual_torque.clear();

  axesStatusMsg_.header.stamp = this->now();
  axesStatusMsg_.header.frame_id = "base_link";

  for (int i = 0; i < numOfAxes_; ++i) {
    axesStatusMsg_.amp_alarm.push_back(cmStatus.axesStatus[i].ampAlarm);
    axesStatusMsg_.servo_on.push_back(cmStatus.axesStatus[i].servoOn);
    axesStatusMsg_.home_done.push_back(cmStatus.axesStatus[i].homeDone);
    axesStatusMsg_.motion_complete.push_back(cmStatus.axesStatus[i].motionComplete);
    axesStatusMsg_.negative_ls.push_back(cmStatus.axesStatus[i].negativeLS);
    axesStatusMsg_.positive_ls.push_back(cmStatus.axesStatus[i].positiveLS);
    axesStatusMsg_.home_switch.push_back(cmStatus.axesStatus[i].homeSwitch);
    axesStatusMsg_.pos_cmd.push_back(cmStatus.axesStatus[i].posCmd);
    axesStatusMsg_.velocity_cmd.push_back(cmStatus.axesStatus[i].velocityCmd);
    axesStatusMsg_.actual_pos.push_back(cmStatus.axesStatus[i].actualPos);
    axesStatusMsg_.actual_velocity.push_back(cmStatus.axesStatus[i].actualVelocity);
    axesStatusMsg_.actual_torque.push_back(cmStatus.axesStatus[i].actualTorque);
  }
  axesStatusPub_->publish(axesStatusMsg_);
}

void WmxCoreMotionNode::jogWatchdogStep()
{
  std::string message;
  api_->stopExpiredJogs(this->now(), message);
}

void WmxCoreMotionNode::stopCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response)
{
  if (request->axis.empty()) {
    response->success = false;
    response->message = "stop: no axis given";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (request->data.size() != request->axis.size()) {
    response->success = false;
    response->message = "stop: axis and data must be the same size";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->axis.size(); ++i) {
    const int axis = request->axis[i];

    std::string clearMessage;
    api_->clearJog(axis, clearMessage);

    std::string message;
    if (api_->stop(axis, message) != ErrorCode::None) {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::startPosCallback(
  const std::shared_ptr<wmx_r2_message::srv::StartAxesPose::Request> request,
  std::shared_ptr<wmx_r2_message::srv::StartAxesPose::Response> response)
{
  if (isMotionBlocked()) {
    response->success = false;
    response->message = "startPos: rejected, a controller is active and owns the axes";
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (request->axis.empty()) {
    response->success = false;
    response->message = "startPos: no axis given";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  const size_t axisCount = request->axis.size();
  if (request->target.size() != axisCount ||
    request->velocity.size() != axisCount ||
    request->acc.size() != axisCount ||
    request->dec.size() != axisCount)
  {
    response->success = false;
    response->message = "startPos: axis, target, velocity, acc and dec must be the same size";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < axisCount; ++i) {
    std::string message;
    if (api_->startPos(
        request->axis[i], request->target[i], request->velocity[i],
        request->acc[i], request->dec[i], message) != ErrorCode::None)
    {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::startMovCallback(
  const std::shared_ptr<wmx_r2_message::srv::StartAxesPose::Request> request,
  std::shared_ptr<wmx_r2_message::srv::StartAxesPose::Response> response)
{
  if (isMotionBlocked()) {
    response->success = false;
    response->message = "startMov: rejected, a controller is active and owns the axes";
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (request->axis.empty()) {
    response->success = false;
    response->message = "startMov: no axis given";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  const size_t axisCount = request->axis.size();
  if (request->target.size() != axisCount ||
    request->velocity.size() != axisCount ||
    request->acc.size() != axisCount ||
    request->dec.size() != axisCount)
  {
    response->success = false;
    response->message = "startMov: axis, target, velocity, acc and dec must be the same size";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < axisCount; ++i) {
    std::string message;
    if (api_->startMov(
        request->axis[i], request->target[i], request->velocity[i],
        request->acc[i], request->dec[i], message) != ErrorCode::None)
    {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::startVelCallback(
  const std::shared_ptr<wmx_r2_message::srv::StartAxesVelocity::Request> request,
  std::shared_ptr<wmx_r2_message::srv::StartAxesVelocity::Response> response)
{
  if (isMotionBlocked()) {
    response->success = false;
    response->message = "startVel: rejected, a controller is active and owns the axes";
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (request->axis.empty()) {
    response->success = false;
    response->message = "startVel: no axis given";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  const size_t axisCount = request->axis.size();
  if (request->velocity.size() != axisCount ||
    request->acc.size() != axisCount ||
    request->dec.size() != axisCount)
  {
    response->success = false;
    response->message = "startVel: axis, velocity, acc and dec must be the same size";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < axisCount; ++i) {
    std::string message;
    if (api_->startVel(
        request->axis[i], request->velocity[i], request->acc[i],
        request->dec[i], message) != ErrorCode::None)
    {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::startJogCallback(
  const std::shared_ptr<wmx_r2_message::srv::StartAxesVelocity::Request> request,
  std::shared_ptr<wmx_r2_message::srv::StartAxesVelocity::Response> response)
{
  if (isMotionBlocked()) {
    response->success = false;
    response->message = "startJog: rejected, a controller is active and owns the axes";
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (request->axis.empty()) {
    response->success = false;
    response->message = "startJog: no axis given";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  const size_t axisCount = request->axis.size();
  if (request->velocity.size() != axisCount ||
    request->acc.size() != axisCount ||
    request->dec.size() != axisCount)
  {
    response->success = false;
    response->message = "startJog: axis, velocity, acc and dec must be the same size";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  const rclcpp::Time now = this->now();

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < axisCount; ++i) {
    std::string message;
    if (api_->startJog(
        request->axis[i], request->velocity[i], request->acc[i],
        request->dec[i], now, message) != ErrorCode::None)
    {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setServoOnCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response)
{
  if (request->axis.empty()) {
    response->success = false;
    response->message = "No axis given";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (request->data.size() != request->axis.size()) {
    response->success = false;
    response->message = "axis and data must be the same size";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->axis.size(); ++i) {
    std::string message;
    if (api_->setServoOn(request->axis[i], request->data[i], message) != ErrorCode::None) {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisCommandModeCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response)
{
  if (request->axis.empty()) {
    response->success = false;
    response->message = "No axis given";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (request->data.size() != request->axis.size()) {
    response->success = false;
    response->message = "axis and data must be the same size";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->axis.size(); ++i) {
    std::string message;
    if (api_->setAxisCommandMode(request->axis[i], request->data[i], message) !=
      ErrorCode::None)
    {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::clearAmpAlarmCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response)
{
  if (request->axis.empty()) {
    response->success = false;
    response->message = "No axis given";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->axis.size(); ++i) {
    std::string message;
    if (api_->clearAmpAlarm(request->axis[i], message) != ErrorCode::None) {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisPolarityCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response)
{
  if (request->axis.empty()) {
    response->success = false;
    response->message = "No axis given";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (request->data.size() != request->axis.size()) {
    response->success = false;
    response->message = "axis and data must be the same size";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->axis.size(); ++i) {
    std::string message;
    if (api_->setAxisPolarity(request->axis[i], request->data[i], message) != ErrorCode::None) {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setGearRatioCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxesGearRatio::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxesGearRatio::Response> response)
{
  if (request->axis.empty()) {
    response->success = false;
    response->message = "No axis given";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (request->numerator.size() != request->axis.size() ||
    request->denominator.size() != request->axis.size())
  {
    response->success = false;
    response->message = "axis, numerator and denominator must be the same size";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->axis.size(); ++i) {
    std::string message;
    if (api_->setGearRatio(
        request->axis[i], request->numerator[i], request->denominator[i], message) !=
      ErrorCode::None)
    {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::startHomeCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response)
{
  if (isMotionBlocked()) {
    response->success = false;
    response->message = "Homing rejected: a controller is active and owns the axes.";
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (request->axis.empty()) {
    response->success = false;
    response->message = "No axis given";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->axis.size(); ++i) {
    std::string message;
    if (api_->startHome(request->axis[i], message) != ErrorCode::None) {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxCoreMotionNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
