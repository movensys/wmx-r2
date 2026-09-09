// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef JOINT_POSITION_CONTROLLER_HPP_
#define JOINT_POSITION_CONTROLLER_HPP_

#include <atomic>
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

class JointPositionControllerApi
{
public:
  explicit JointPositionControllerApi(const rclcpp::Logger & logger);
  ~JointPositionControllerApi();

  int createDevice(std::string & message);
  void closeDevice();

  int startLinearIntplPos(
    const std::vector<int> & axes,
    const std::vector<double> & targets,
    const std::vector<double> & maxVelocity,
    const std::vector<double> & maxAcc,
    std::string & message);

  int stop(std::string & message);
  int setAxisSelection(const std::vector<int64_t> & axes, std::string & message);
  int getPosCmd(
    const std::vector<int> & axes, std::vector<double> & posCmd, std::string & message);

private:
  rclcpp::Logger logger_;

  const char * deviceName_ = "joint_position_controller";
  unsigned int timeout_ = 10000;

  mutable std::mutex deviceMutex_;

  wmx3Api::AxisSelection axisSel_;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::CoreMotion cm_;
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
  double accelRatio_ = 0.0;
  double defaultVelocity_ = 0.0;
  double minStep_ = 0.0;

  std::map<std::string, int> axisByName_;

  std::atomic<bool> inExecution_{false};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execActiveSub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr jointTrajectorySub_;

  void setRosParameter();

  bool buildCommand(
    const trajectory_msgs::msg::JointTrajectory & traj,
    std::vector<int> & axes,
    std::vector<double> & targets,
    std::vector<double> & maxVelocity,
    std::vector<double> & maxAcc);

  void execActiveCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
};

#endif  // JOINT_POSITION_CONTROLLER_HPP_
