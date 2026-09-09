// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_CORE_MOTION_NODE_HPP_
#define WMX_CORE_MOTION_NODE_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "wmx_r2_message/srv/set_axes.hpp"
#include "wmx_r2_message/srv/set_axes_gear_ratio.hpp"
#include "wmx_r2_message/srv/start_axes_pose.hpp"
#include "wmx_r2_message/srv/start_axes_velocity.hpp"
#include "wmx_r2_message/msg/axes_status.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

class WmxCoreMotionNodeApi
{
public:
  struct Config
  {
    double jogTimeoutMs = 200.0;
    double jogRunTimeMs = 2000.0;
    double jogJerkRatio = 0.75;
  };

  WmxCoreMotionNodeApi(const rclcpp::Logger & logger, const Config & config);
  ~WmxCoreMotionNodeApi();

  int createDevice(std::string & message);
  void closeDevice();

  int getNumOfAxes();
  int getStatus(wmx3Api::CoreMotionStatus & status);

  int startPos(
    int axis, double target, double velocity, double acc, double dec, std::string & message);
  int startMov(
    int axis, double target, double velocity, double acc, double dec, std::string & message);
  int startVel(int axis, double velocity, double acc, double dec, std::string & message);
  int stop(int axis, std::string & message);

  int startJog(
    int axis, double velocity, double acc, double dec, const rclcpp::Time & now,
    std::string & message);
  int stopExpiredJogs(const rclcpp::Time & now, std::string & message);
  int stopAllJogs(std::string & message);
  int clearJog(int axis, std::string & message);

  int setServoOn(int axis, int newStatus, std::string & message);
  int setAxisCommandMode(int axis, int mode, std::string & message);
  int clearAmpAlarm(int axis, std::string & message);
  int setAxisPolarity(int axis, int polarity, std::string & message);
  int setGearRatio(int axis, double numerator, double denominator, std::string & message);
  int startHome(int axis, std::string & message);

private:
  struct JogState
  {
    double velocity;
    rclcpp::Time deadline;
  };

  std::string errorToString(int err);

  rclcpp::Logger logger_;
  Config config_;

  std::mutex jogMutex_;
  std::unordered_map<int, JogState> jogState_;

  const char * deviceName_ = "wmx_core_motion_node";
  unsigned int timeout_ = 10000;
  unsigned int servoOnTimeout_ = 1000;

  mutable std::mutex deviceMutex_;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::CoreMotion cm_;
};

class WmxCoreMotionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  WmxCoreMotionNode();
  ~WmxCoreMotionNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::unique_ptr<WmxCoreMotionNodeApi> api_;

  int numOfAxes_ = 0;
  int rate_ = 1;

  rclcpp::TimerBase::SharedPtr axesStatusTimer_;
  rclcpp::TimerBase::SharedPtr jogWatchdogTimer_;
  wmx_r2_message::msg::AxesStatus axesStatusMsg_;

  std::vector<std::string> motionControllers_;
  double controllerResyncPeriod_ = 0.2;

  mutable std::mutex controllerMutex_;
  std::map<std::string, bool> controllerActive_;

  rclcpp::CallbackGroup::SharedPtr clientCbGroup_;
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> getStateClients_;
  std::vector<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr>
  transitionEventSubs_;
  rclcpp::TimerBase::SharedPtr controllerResyncTimer_;

  void setControllerActive(const std::string & controller, bool active);
  void transitionEventCallback(
    const std::string & controller,
    const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);
  void resyncControllerStates();

  bool isMotionBlocked() const;

  rclcpp::CallbackGroup::SharedPtr homing_cb_group_;
  rclcpp_lifecycle::LifecyclePublisher<wmx_r2_message::msg::AxesStatus>::SharedPtr axesStatusPub_;
  rclcpp::Service<wmx_r2_message::srv::StartAxesPose>::SharedPtr startPosService_;
  rclcpp::Service<wmx_r2_message::srv::StartAxesPose>::SharedPtr startMovService_;
  rclcpp::Service<wmx_r2_message::srv::StartAxesVelocity>::SharedPtr startVelService_;
  rclcpp::Service<wmx_r2_message::srv::StartAxesVelocity>::SharedPtr startJogService_;

  rclcpp::Service<wmx_r2_message::srv::SetAxes>::SharedPtr setServoOnService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxes>::SharedPtr clearAmpAlarmService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxes>::SharedPtr setAxisCommandModeService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxes>::SharedPtr setAxisPolarityService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxesGearRatio>::SharedPtr setGearRatioService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxes>::SharedPtr startHomeService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxes>::SharedPtr stopService_;

  void axesStatusStep();
  void jogWatchdogStep();

  void startPosCallback(
    const std::shared_ptr<wmx_r2_message::srv::StartAxesPose::Request> request,
    std::shared_ptr<wmx_r2_message::srv::StartAxesPose::Response> response);
  void startMovCallback(
    const std::shared_ptr<wmx_r2_message::srv::StartAxesPose::Request> request,
    std::shared_ptr<wmx_r2_message::srv::StartAxesPose::Response> response);
  void startVelCallback(
    const std::shared_ptr<wmx_r2_message::srv::StartAxesVelocity::Request> request,
    std::shared_ptr<wmx_r2_message::srv::StartAxesVelocity::Response> response);
  void startJogCallback(
    const std::shared_ptr<wmx_r2_message::srv::StartAxesVelocity::Request> request,
    std::shared_ptr<wmx_r2_message::srv::StartAxesVelocity::Response> response);

  void setServoOnCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response);
  void setAxisCommandModeCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response);
  void clearAmpAlarmCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response);
  void setAxisPolarityCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response);
  void setGearRatioCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxesGearRatio::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxesGearRatio::Response> response);
  void startHomeCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response);
  void stopCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response);
};

#endif  // WMX_CORE_MOTION_NODE_HPP_
