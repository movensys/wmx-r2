// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef JOINT_TRAJECTORY_CONTROLLER_HPP_
#define JOINT_TRAJECTORY_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/bool.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "AdvancedMotionApi.h"

class JointTrajectoryControllerApi
{
public:
  static constexpr int kSplineChannel = 0;
  static constexpr unsigned int kMaxTrajectoryPoints = 1000;

  explicit JointTrajectoryControllerApi(const rclcpp::Logger & logger);
  ~JointTrajectoryControllerApi();

  int attachDevice(std::string & message);
  void releaseDevice();

  int startCSplinePos(
    const std::vector<std::vector<double>> & positions,
    const std::vector<double> & timesMs,
    std::string & message);

  int setAxes(const std::vector<int64_t> & axes, std::string & message);
  int getInPos(bool & inPos, std::string & message);
  int stopAxes(std::string & message);

  bool isDeviceOpen() const {return isDeviceAttached_;}
  size_t axisCount() const {return axisCount_;}

private:
  int createSplineBuffer(std::string & message);
  void freeSplineBuffer();

  rclcpp::Logger logger_;

  const char * deviceName_ = "joint_trajectory_controller";
  unsigned int timeout_ = 10000;

  mutable std::mutex deviceMutex_;

  size_t axisCount_ = 0;
  bool isDeviceAttached_ = false;

  wmx3Api::AxisSelection axisSel_;
  wmx3Api::AdvMotion::PointTimeSplineCommand splineCommand_;
  std::vector<wmx3Api::AdvMotion::SplinePoint> splinePoints_;
  std::vector<double> splineTimesMs_;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::CoreMotion cm_;
  wmx3Api::AdvancedMotion am_;
};

class JointTrajectoryController : public rclcpp_lifecycle::LifecycleNode
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  JointTrajectoryController();
  ~JointTrajectoryController() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::unique_ptr<JointTrajectoryControllerApi> api_;

  std::vector<int64_t> jointAxes_;
  std::string jointTrajectoryAction_;

  std::atomic<bool> isNodeActive_{false};

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr actionServer_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr execActivePub_;
  rclcpp_lifecycle::LifecyclePublisher<control_msgs::msg::JointJog>::SharedPtr servoNodeResetPub_;

  bool isNodeActive() const;
  std::string notActiveMessage();

  void setRosParameter();

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandleFJT> goalHandle);

  void handleAccepted(std::shared_ptr<GoalHandleFJT> goalHandle);

  void executeGoal(std::shared_ptr<GoalHandleFJT> goalHandle);

  bool buildSplineInput(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    std::vector<std::vector<double>> & positions,
    std::vector<double> & timesMs,
    std::string & message);

  void logTrajectory(const trajectory_msgs::msg::JointTrajectory & trajectory);
  void publishExecActive(bool active);
  void resetServo(const std::vector<std::string> & jointNames);
};

#endif  // JOINT_TRAJECTORY_CONTROLLER_HPP_
