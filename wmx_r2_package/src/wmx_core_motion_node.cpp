// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_core_motion_node.hpp"

using wmx3Api::AxisCommandMode;
using wmx3Api::Config;
using wmx3Api::CoreMotion;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;

WmxCoreMotionNode::WmxCoreMotionNode()
: Node("wmx_core_motion_node")
{
  init_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = init_cb_group_;

  auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
  engineReadySub_ = this->create_subscription<std_msgs::msg::Bool>(
    "wmx/engine/ready", ready_qos,
    std::bind(&WmxCoreMotionNode::onEngineReady, this, _1), sub_opts);

  coreMotionReadyPub_ = this->create_publisher<std_msgs::msg::Bool>(
    "wmx/core_motion/ready", ready_qos);

  setAxisOnService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/set_on",
    std::bind(&WmxCoreMotionNode::setAxisOn, this, _1, _2));

  clearAlarmService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/clear_alarm",
    std::bind(&WmxCoreMotionNode::clearAlarm, this, _1, _2));

  setAxisModeService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/set_mode",
    std::bind(&WmxCoreMotionNode::setAxisMode, this, _1, _2));

  setAxisPolarityService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/set_polarity",
    std::bind(&WmxCoreMotionNode::setAxisPolarity, this, _1, _2));

  setAxisGearRatioService_ = this->create_service<wmx_r2_message::srv::SetAxisGearRatio>(
    "wmx/axis/set_gear_ratio",
    std::bind(&WmxCoreMotionNode::setAxisGearRatio, this, _1, _2));

  setHomingService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/homing",
    std::bind(&WmxCoreMotionNode::setHoming, this, _1, _2));

  stopAxisService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/stop",
    std::bind(&WmxCoreMotionNode::stopAxes, this, _1, _2));

  jogTimeoutMs_ = this->declare_parameter("jog_timeout_ms", 200.0);
  jogRunTimeMs_ = this->declare_parameter("jog_run_time_ms", 2000.0);
  jogJerkRatio_ = this->declare_parameter("jog_jerk_ratio", 0.75);

  loadParamsService_ = this->create_service<wmx_r2_message::srv::LoadWmxParams>(
    "wmx/params/load",
    std::bind(&WmxCoreMotionNode::loadWmxParams, this, _1, _2));

  getParamsService_ = this->create_service<wmx_r2_message::srv::GetWmxParams>(
    "wmx/params/get",
    std::bind(&WmxCoreMotionNode::getWmxParams, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node waiting for engine...");
}

WmxCoreMotionNode::~WmxCoreMotionNode()
{
  if (axisStateTimer_) {
    axisStateTimer_->cancel();
  }
  if (jogWatchdogTimer_) {
    jogWatchdogTimer_->cancel();
  }
  if (initialized_) {
    {
      std::lock_guard<std::mutex> lock(jogMutex_);
      for (const auto & entry : jogState_) {
        wmx3LibCm_->motion->Stop(entry.first);
      }
      jogState_.clear();
    }

    err_ = wmx3Lib_.CloseDevice();
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(this->get_logger(), "Failed to close device");
    } else {
      RCLCPP_INFO(this->get_logger(), "Device closed");
    }
  }
  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node stopped");
}

void WmxCoreMotionNode::onEngineReady(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data || initialized_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Engine ready — initializing CoreMotion...");

  unsigned int timeout = 10000;
  err_ = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout);

  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    if (err_ == ErrorCode::StartProcessLockError) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to attach to device (lock busy, will retry on next signal).");
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to attach to device. Error=%d (%s)", err_, errString_);
    }
    return;
  }

  wmx3Lib_.SetDeviceName("wmx_core_motion_node");
  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");

  wmx3LibCm_ = std::make_unique<CoreMotion>(&wmx3Lib_);

  axisStatePub_ = this->create_publisher<wmx_r2_message::msg::AxisState>(
    "wmx/axis/state", 1);

  axisVelSub_ = this->create_subscription<wmx_r2_message::msg::AxisVelocity>(
    "wmx/axis/velocity", 1,
    std::bind(&WmxCoreMotionNode::axisVelCallback, this, _1));

  axisPoseSub_ = this->create_subscription<wmx_r2_message::msg::AxisPose>(
    "wmx/axis/position", 1,
    std::bind(&WmxCoreMotionNode::axisPoseCallback, this, _1));

  axisPoseRelativeSub_ = this->create_subscription<wmx_r2_message::msg::AxisPose>(
    "wmx/axis/position/relative", 1,
    std::bind(&WmxCoreMotionNode::axisPoseRelativeCallback, this, _1));

  axisJogSub_ = this->create_subscription<wmx_r2_message::msg::AxisVelocity>(
    "wmx/axis/jog", 1,
    std::bind(&WmxCoreMotionNode::axisJogCallback, this, _1));

  axisStateTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / rate_),
    std::bind(&WmxCoreMotionNode::axisStateStep, this));

  jogWatchdogTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&WmxCoreMotionNode::jogWatchdogStep, this));

  initialized_ = true;

  auto ready_msg = std_msgs::msg::Bool();
  ready_msg.data = true;
  coreMotionReadyPub_->publish(ready_msg);

  engineReadySub_.reset();

  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node is ready (100 Hz)");
}

void WmxCoreMotionNode::axisStateStep()
{
  axisStateMsg_.amp_alarm.clear();
  axisStateMsg_.servo_on.clear();
  axisStateMsg_.home_done.clear();
  axisStateMsg_.motion_complete.clear();
  axisStateMsg_.negative_ls.clear();
  axisStateMsg_.positive_ls.clear();
  axisStateMsg_.home_switch.clear();
  axisStateMsg_.pos_cmd.clear();
  axisStateMsg_.velocity_cmd.clear();
  axisStateMsg_.actual_pos.clear();
  axisStateMsg_.actual_velocity.clear();
  axisStateMsg_.actual_torque.clear();

  axisStateMsg_.header.stamp = this->now();
  axisStateMsg_.header.frame_id = "base_link";

  wmx3LibCm_->GetStatus(&cmStatus_);

  for (int i = 0; i < axisCount_; ++i) {
    axisStateMsg_.amp_alarm.push_back(cmStatus_.axesStatus[i].ampAlarm);
    axisStateMsg_.servo_on.push_back(cmStatus_.axesStatus[i].servoOn);
    axisStateMsg_.home_done.push_back(cmStatus_.axesStatus[i].homeDone);
    axisStateMsg_.motion_complete.push_back(cmStatus_.axesStatus[i].motionComplete);
    axisStateMsg_.negative_ls.push_back(cmStatus_.axesStatus[i].negativeLS);
    axisStateMsg_.positive_ls.push_back(cmStatus_.axesStatus[i].positiveLS);
    axisStateMsg_.home_switch.push_back(cmStatus_.axesStatus[i].homeSwitch);
    axisStateMsg_.pos_cmd.push_back(cmStatus_.axesStatus[i].posCmd);
    axisStateMsg_.velocity_cmd.push_back(cmStatus_.axesStatus[i].velocityCmd);
    axisStateMsg_.actual_pos.push_back(cmStatus_.axesStatus[i].actualPos);
    axisStateMsg_.actual_velocity.push_back(cmStatus_.axesStatus[i].actualVelocity);
    axisStateMsg_.actual_torque.push_back(cmStatus_.axesStatus[i].actualTorque);
  }
  axisStatePub_->publish(axisStateMsg_);
}

void WmxCoreMotionNode::axisPoseCallback(const wmx_r2_message::msg::AxisPose::SharedPtr msg)
{
  size_t axis_count = msg->index.size();
  for (size_t i = 0; i < axis_count; i++) {
    position_.axis = msg->index[i];
    position_.target = msg->target[i];
    position_.profile.velocity = msg->velocity[i];
    position_.profile.type = ProfileType::T::Trapezoidal;
    position_.profile.acc = msg->acc[i];
    position_.profile.dec = msg->dec[i];

    err_ = wmx3LibCm_->motion->StartPos(&position_);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to move position motor %d. Error=%d (%s)",
        msg->index[i], err_, errString_);
    }
  }
}

void WmxCoreMotionNode::axisPoseRelativeCallback(
  const wmx_r2_message::msg::AxisPose::SharedPtr msg)
{
  size_t axis_count = msg->index.size();
  for (size_t i = 0; i < axis_count; i++) {
    position_.axis = msg->index[i];
    position_.target = msg->target[i];
    position_.profile.velocity = msg->velocity[i];
    position_.profile.type = ProfileType::T::Trapezoidal;
    position_.profile.acc = msg->acc[i];
    position_.profile.dec = msg->dec[i];

    err_ = wmx3LibCm_->motion->StartMov(&position_);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to move relative motor %d. Error=%d (%s)",
        msg->index[i], err_, errString_);
    }
  }
}

void WmxCoreMotionNode::axisVelCallback(const wmx_r2_message::msg::AxisVelocity::SharedPtr msg)
{
  size_t axis_count = msg->index.size();
  for (size_t i = 0; i < axis_count; i++) {
    velocity_.axis = msg->index[i];
    velocity_.profile.velocity = msg->velocity[i];
    velocity_.profile.type = ProfileType::T::Trapezoidal;
    velocity_.profile.acc = msg->acc[i];
    velocity_.profile.dec = msg->dec[i];

    err_ = wmx3LibCm_->velocity->StartVel(&velocity_);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to move velocity motor %d. Error=%d (%s)",
        msg->index[i], err_, errString_);
    }
  }
}

// Jog command. The publisher (keyboard/joystick teleop, CLI, ...) must keep
// republishing while the operator holds the control; jogWatchdogStep() stops
// the axis once refreshes stop arriving. Velocity sign selects the direction.
void WmxCoreMotionNode::axisJogCallback(const wmx_r2_message::msg::AxisVelocity::SharedPtr msg)
{
  if (!initialized_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Jog ignored: CoreMotion not initialized.");
    return;
  }

  const size_t axis_count = msg->index.size();
  if (msg->velocity.size() != axis_count ||
    msg->acc.size() != axis_count ||
    msg->dec.size() != axis_count)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Jog ignored: index/velocity/acc/dec must be the same length.");
    return;
  }

  const rclcpp::Time deadline =
    this->now() + rclcpp::Duration::from_nanoseconds(
    static_cast<int64_t>(jogTimeoutMs_ * 1e6));

  for (size_t i = 0; i < axis_count; ++i) {
    const int axis = msg->index[i];
    const double velocity = msg->velocity[i];

    if (velocity == 0.0) {
      std::lock_guard<std::mutex> lock(jogMutex_);
      if (jogState_.erase(axis) > 0) {
        stopAxis(axis);
      }
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(jogMutex_);
      auto it = jogState_.find(axis);
      if (it != jogState_.end() && it->second.velocity == velocity) {
        // Same direction and speed: only refresh the dead-man deadline.
        // Re-issuing StartJog here would override a jog with a jog, which the
        // WMX3 manual does not define. It also means that once jog_run_time_ms
        // elapses the axis stays stopped until the operator releases and
        // presses again, which is the intended behavior.
        it->second.deadline = deadline;
        continue;
      }
    }

    // Same profile WOS uses for its jog buttons. A run time requires a
    // time-based profile: Trapezoidal is rejected with ProfileTypeNotSupported.
    // acc/dec stay accelerations on the wire and are converted to ramp times
    // here, so AxisVelocity keeps the same meaning as on wmx/axis/velocity.
    const double accTimeMs = (msg->acc[i] > 0.0) ?
      std::abs(velocity) / msg->acc[i] * 1000.0 : 0.0;
    const double decTimeMs = (msg->dec[i] > 0.0) ?
      std::abs(velocity) / msg->dec[i] * 1000.0 : 0.0;

    wmx3Api::Motion::JogCommand jogCommand = wmx3Api::Motion::JogCommand();
    jogCommand.axis = axis;
    jogCommand.profile.type = ProfileType::T::TimeAccJerkRatio;
    jogCommand.profile.velocity = velocity;
    jogCommand.profile.acc = 0;
    jogCommand.profile.dec = 0;
    jogCommand.profile.jerkAcc = 0;
    jogCommand.profile.jerkDec = 0;
    jogCommand.profile.jerkAccRatio = jogJerkRatio_;
    jogCommand.profile.jerkDecRatio = jogJerkRatio_;
    jogCommand.profile.accTimeMilliseconds = accTimeMs;
    jogCommand.profile.decTimeMilliseconds = decTimeMs;
    jogCommand.profile.startingVelocity = 0;
    jogCommand.profile.endVelocity = 0;
    jogCommand.profile.secondVelocity = 0;
    jogCommand.profile.movingAverageTimeMilliseconds = 0;
    // Backstop: if this node dies mid-jog the engine still decelerates the axis.
    // Once it elapses the axis stays stopped until the operator releases and
    // presses again, because a held key only refreshes the dead-man deadline.
    jogCommand.SetRunTime(jogRunTimeMs_);

    const int err = wmx3LibCm_->motion->StartJog(&jogCommand);
    if (err != ErrorCode::None) {
      char errString[256];
      wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to jog motor %d. Error=%d (%s)", axis, err, errString);
      continue;
    }

    std::lock_guard<std::mutex> lock(jogMutex_);
    jogState_.insert_or_assign(axis, JogState{velocity, deadline});
  }
}

// Dead-man: stop every axis whose jog refresh has expired.
void WmxCoreMotionNode::jogWatchdogStep()
{
  if (!initialized_) {
    return;
  }

  const rclcpp::Time now = this->now();
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

  for (const int axis : expired) {
    stopAxis(axis);
  }
}

// Decelerate an axis to a stop. Returns the WMX3 error code.
// Stopping an already idle axis is expected here (jog_run_time_ms may have
// elapsed before the operator released), so callers decide how loud to be.
int WmxCoreMotionNode::stopAxis(int axis)
{
  const int err = wmx3LibCm_->motion->Stop(axis);
  if (err != ErrorCode::None) {
    char errString[256];
    wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
    RCLCPP_DEBUG(
      this->get_logger(),
      "Stop on axis %d returned %d (%s)", axis, err, errString);
  }
  return err;
}

void WmxCoreMotionNode::stopAxes(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "CoreMotion not initialized. Engine not ready.";
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    const int axis = request->index[i];

    {
      std::lock_guard<std::mutex> lock(jogMutex_);
      jogState_.erase(axis);
    }

    const int err = stopAxis(axis);
    if (err != ErrorCode::None) {
      char errString[256];
      wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
      msg_stream << "Failed to stop axis " << axis
                 << ". Error=" << err << " (" << errString << "); ";
      all_success = false;
    } else {
      msg_stream << "Stopped axis " << axis << "; ";
    }
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisOn(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "CoreMotion not initialized. Engine not ready.";
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    int axis_index = request->index[i];
    int on_off = request->data[i];

    err_ = wmx3LibCm_->axisControl->SetServoOn(axis_index, on_off, 1000);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      snprintf(
        buffer_, sizeof(buffer_),
        "Failed to set axis %d %s. Error=%d (%s)",
        axis_index, on_off ? "on" : "off", err_, errString_);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
      all_success = false;
    } else {
      snprintf(
        buffer_, sizeof(buffer_), "Set axis %d %s",
        axis_index, on_off ? "on" : "off");
      RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    }
    msg_stream << buffer_ << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisMode(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "CoreMotion not initialized. Engine not ready.";
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    int axis_index = request->index[i];
    int mode_value = request->data[i];

    if (mode_value == 0) {
      err_ = wmx3LibCm_->axisControl->SetAxisCommandMode(
        axis_index, AxisCommandMode::Position);
    } else if (mode_value == 1) {
      err_ = wmx3LibCm_->axisControl->SetAxisCommandMode(
        axis_index, AxisCommandMode::Velocity);
    } else {
      snprintf(
        buffer_, sizeof(buffer_),
        "Invalid mode %d for axis %d", mode_value, axis_index);
      RCLCPP_WARN(this->get_logger(), "%s", buffer_);
      msg_stream << buffer_ << "; ";
      all_success = false;
      continue;
    }

    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      snprintf(
        buffer_, sizeof(buffer_),
        "Failed to set axis %d mode %d. Error=%d (%s)",
        axis_index, mode_value, err_, errString_);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
      all_success = false;
    } else {
      const char * mode_str = (mode_value == 0) ? "Position" : "Velocity";
      snprintf(buffer_, sizeof(buffer_), "Set axis %d in %s mode", axis_index, mode_str);
      RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    }
    msg_stream << buffer_ << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::clearAlarm(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "CoreMotion not initialized. Engine not ready.";
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    err_ = wmx3LibCm_->axisControl->ClearAmpAlarm(request->index[i]);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      snprintf(
        buffer_, sizeof(buffer_),
        "Failed to clear alarm axis %d. Error=%d (%s)",
        request->index[i], err_, errString_);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
      all_success = false;
    } else {
      snprintf(buffer_, sizeof(buffer_), "Cleared alarm axis %d", request->index[i]);
      RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    }
    msg_stream << buffer_ << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisPolarity(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "CoreMotion not initialized. Engine not ready.";
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    if (request->data[i] != 1 && request->data[i] != -1) {
      snprintf(
        buffer_, sizeof(buffer_),
        "Invalid polarity value for axis %d: %d",
        request->index[i], request->data[i]);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
      msg_stream << buffer_ << "; ";
      all_success = false;
      continue;
    }

    err_ = wmx3LibCm_->config->SetAxisPolarity(request->index[i], request->data[i]);
    if (err_ != ErrorCode::None) {
      snprintf(
        buffer_, sizeof(buffer_),
        "Failed to set polarity on axis %d", request->index[i]);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
      all_success = false;
    } else {
      snprintf(buffer_, sizeof(buffer_), "Set polarity on axis %d", request->index[i]);
      RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    }
    msg_stream << buffer_ << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisGearRatio(
  const std::shared_ptr<wmx_r2_message::srv::SetAxisGearRatio::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxisGearRatio::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "CoreMotion not initialized. Engine not ready.";
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    err_ = wmx3LibCm_->config->SetGearRatio(
      request->index[i], request->numerator[i], request->denominator[i]);
    if (err_ != ErrorCode::None) {
      all_success = false;
      snprintf(
        buffer_, sizeof(buffer_),
        "Failed to set gear ratio on axis %d", request->index[i]);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    } else {
      snprintf(buffer_, sizeof(buffer_), "Set gear ratio on axis %d", request->index[i]);
      RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    }
    msg_stream << buffer_ << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setHoming(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "CoreMotion not initialized. Engine not ready.";
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    wmx3LibCm_->config->GetHomeParam(request->index[i], &homeParam_);
    homeParam_.homeType = Config::HomeType::CurrentPos;
    wmx3LibCm_->config->SetHomeParam(request->index[i], &homeParam_);
    wmx3LibCm_->home->StartHome(request->index[i]);
    err_ = wmx3LibCm_->motion->Wait(request->index[i]);

    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      snprintf(
        buffer_, sizeof(buffer_),
        "Failed to home axis %d. Error=%d (%s)",
        request->index[i], err_, errString_);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
      all_success = false;
    } else {
      snprintf(buffer_, sizeof(buffer_), "Homed axis %d", request->index[i]);
      RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    }
    msg_stream << buffer_ << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::loadWmxParams(
  const std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Request> request,
  std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "CoreMotion not initialized. Engine not ready.";
    return;
  }

  Config::SystemParam sysParamErr;
  Config::AxisParam axisParamErr;

  err_ = wmx3LibCm_->config->ImportAndSetAll(
    const_cast<char *>(request->file_path.c_str()), &sysParamErr, &axisParamErr);

  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    snprintf(buffer_, sizeof(buffer_), "Failed to load params: %s", errString_);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    response->success = false;
    response->message = buffer_;
  } else {
    RCLCPP_INFO(this->get_logger(), "Loaded WMX params from: %s", request->file_path.c_str());
    response->success = true;
    response->message = "Loaded params from: " + request->file_path;
  }
}

void WmxCoreMotionNode::getWmxParams(
  const std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "CoreMotion not initialized. Engine not ready.";
    return;
  }

  Config::SystemParam sysParam;
  Config::AxisParam axisParam;

  wmx3LibCm_->config->GetParam(&sysParam);
  wmx3LibCm_->config->GetAxisParam(&axisParam);

  auto & lines = response->params_dump;
  for (int32_t i : request->index) {
    lines.push_back("=== Axis " + std::to_string(i) + " ===");

    lines.push_back("[AxisParam]");
    lines.push_back(
      "  GearRatio          = " + std::to_string(axisParam.gearRatioNumerator[i]) +
      " / " + std::to_string(axisParam.gearRatioDenominator[i]));
    lines.push_back("  AxisUnit           = " + std::to_string(axisParam.axisUnit[i]));
    lines.push_back(
      "  AxisPolarity       = " +
      std::to_string(static_cast<int>(axisParam.axisPolarity[i])));
    lines.push_back(
      "  CommandMode        = " +
      std::to_string(static_cast<int>(axisParam.axisCommandMode[i])));
    lines.push_back("  MaxTrqLimit        = " + std::to_string(axisParam.maxTrqLimit[i]));
    lines.push_back("  MaxMotorSpeed      = " + std::to_string(axisParam.maxMotorSpeed[i]));
    lines.push_back(
      "  VelFeedforwardGain = " +
      std::to_string(axisParam.velocityFeedforwardGain[i]));

    lines.push_back("[HomeParam]");
    lines.push_back(
      "  HomeType           = " +
      std::to_string(static_cast<int>(sysParam.homeParam[i].homeType)));
    lines.push_back(
      "  HomeDirection      = " +
      std::to_string(static_cast<int>(sysParam.homeParam[i].homeDirection)));
    lines.push_back(
      "  HomingVelSlow      = " +
      std::to_string(sysParam.homeParam[i].homingVelocitySlow));
    lines.push_back(
      "  HomingVelFast      = " +
      std::to_string(sysParam.homeParam[i].homingVelocityFast));
    lines.push_back("  HomePosition       = " + std::to_string(sysParam.homeParam[i].homePosition));

    lines.push_back("[FeedbackParam]");
    lines.push_back(
      "  InPosWidth         = " +
      std::to_string(sysParam.feedbackParam[i].inPosWidth));
    lines.push_back(
      "  PosSetWidth        = " + std::to_string(
        sysParam.feedbackParam[i].posSetWidth));
    lines.push_back(
      "  DelayedPosSetWidth = " +
      std::to_string(sysParam.feedbackParam[i].delayedPosSetWidth));

    lines.push_back("[AlarmParam]");
    lines.push_back(
      "  FollowErrStopped   = " +
      std::to_string(sysParam.alarmParam[i].followingErrorStopped));
    lines.push_back(
      "  FollowErrMoving    = " +
      std::to_string(sysParam.alarmParam[i].followingErrorMoving));

    lines.push_back("[LimitParam]");
    lines.push_back(
      "  SoftLimitPosPos    = " +
      std::to_string(sysParam.limitParam[i].softLimitPositivePos));
    lines.push_back(
      "  SoftLimitNegPos    = " +
      std::to_string(sysParam.limitParam[i].softLimitNegativePos));
    lines.push_back("");
  }

  response->success = true;
  response->message = "OK";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxCoreMotionNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
