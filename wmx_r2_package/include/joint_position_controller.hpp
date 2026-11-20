// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef JOINT_POSITION_CONTROLLER_HPP_
#define JOINT_POSITION_CONTROLLER_HPP_

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "ApiBufferApi.h"

class JointPositionControllerApi
{
public:
  explicit JointPositionControllerApi(const rclcpp::Logger & logger);
  ~JointPositionControllerApi();

  int createDevice(std::string & message);
  void closeDevice();

  int setAxisSelection(const std::vector<int64_t> & axes, std::string & message);
  int getStatus(
    const std::vector<int> & axes, std::vector<double> & posCmd, std::vector<double> & velCmd,
    std::string & message);

  int openChannel(unsigned int channel, std::string & message);
  int recordOverride(
    const std::vector<int> & axes,
    const std::vector<double> & targets,
    const std::vector<double> & maxVelocity,
    const std::vector<double> & maxAcc,
    double endVelocityRatio,
    std::string & message);
  int channelStatus(int64_t & remaining, int64_t & errors, std::string & message);

  int stop(unsigned int waitTimeout, std::string & message);

private:
  rclcpp::Logger logger_;

  const char * deviceName_ = "joint_position_controller";
  unsigned int timeout_ = 10000;

  mutable std::mutex deviceMutex_;

  wmx3Api::AxisSelection axisSel_;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::CoreMotion cm_;

  // StartRecordBufferChannel records every call made through its device, so
  // recording runs on a second device and status reads stay on the first.
  wmx3Api::WMX3Api wmx3Rec_;
  wmx3Api::CoreMotion cmRec_;
  wmx3Api::ApiBuffer ab_;
  wmx3Api::ApiBuffer abRec_;
  unsigned int channel_ = 0;
  bool channelOpen_ = false;
  bool recAttached_ = false;

  void closeChannelLocked();
  static void fill(
    wmx3Api::Motion::LinearIntplCommand & intpl, const std::vector<int> & axes,
    const std::vector<double> & targets, const std::vector<double> & maxVelocity,
    const std::vector<double> & maxAcc, double endVelocityRatio);
};

class JointPositionController : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  JointPositionController();
  ~JointPositionController() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::unique_ptr<JointPositionControllerApi> api_;

  std::vector<int64_t> jointAxes_;
  std::vector<std::string> jointNames_;
  std::string jointTrajectoryTopic_;
  double publishPeriod_ = 0.0;
  double lookaheadTime_ = 0.0;
  double streamTimeout_ = 0.0;
  double minOverrideInterval_ = 0.0;
  int bufferChannel_ = 0;
  int64_t bufferErrors_ = 0;
  double endVelocityRatio_ = 0.0;
  bool waiting_ = false;
  std::chrono::steady_clock::time_point waitingSince_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr paramCallback_;
  double positionTolerance_ = 0.0;
  std::vector<double> maxVelocity_;
  std::vector<double> maxAcceleration_;

  std::vector<int> axes_;
  std::map<std::string, size_t> indexByName_;

  std::atomic<bool> inExecution_{false};

  bool streaming_ = false;
  bool havePrevious_ = false;
  bool servoMoving_ = false;
  bool pending_ = false;
  size_t sent_ = 0;
  size_t retries_ = 0;
  std::vector<double> velocity_;
  double acceptInterval_ = 0.0;
  std::chrono::steady_clock::time_point lastAccept_;
  std::vector<double> previous_;
  std::chrono::steady_clock::time_point previousTime_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execActiveSub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr jointTrajectorySub_;
  rclcpp::TimerBase::SharedPtr watchdogTimer_;
  rclcpp::TimerBase::SharedPtr retryTimer_;

  void setRosParameter();
  std::vector<double> perJoint(const std::string & name, std::vector<double> values);

  bool readPoint(
    const trajectory_msgs::msg::JointTrajectory & traj,
    std::vector<double> & position, std::vector<double> & velocity);
  void track(const std::vector<double> & position, const std::vector<double> & velocity);
  void send();
  void brake(const char * reason);
  void haltAxes(const char * reason);

  void execActiveCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void watchdogCallback();
};

#endif  // JOINT_POSITION_CONTROLLER_HPP_
