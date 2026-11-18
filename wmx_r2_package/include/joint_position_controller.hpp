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
#include "CyclicBufferApi.h"

class JointPositionControllerApi
{
public:
  explicit JointPositionControllerApi(const rclcpp::Logger & logger);
  ~JointPositionControllerApi();

  int createDevice(std::string & message);
  void closeDevice();

  int setAxisSelection(const std::vector<int64_t> & axes, std::string & message);
  int getStatus(
    const std::vector<int> & axes, std::vector<double> & posCmd, double & cycleTimeMs,
    std::string & message);

  int openCyclicBuffer(unsigned int numOfCycles, std::string & message);
  int execute(std::string & message);
  int addCommand(
    const std::vector<int> & axes, const std::vector<std::vector<double>> & samples,
    unsigned int intervalCycles, std::string & message);
  int getCyclicBufferStatus(
    const std::vector<int> & axes, unsigned int & remainCount, unsigned int & availableCount,
    wmx3Api::CyclicBufferState::T & state, std::string & message);
  int abort(std::string & message);
  int closeCyclicBuffer(std::string & message);

private:
  rclcpp::Logger logger_;

  const char * deviceName_ = "joint_position_controller";
  unsigned int timeout_ = 10000;

  mutable std::mutex deviceMutex_;

  wmx3Api::AxisSelection axisSel_;
  bool bufferOpen_ = false;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::CoreMotion cm_;
  wmx3Api::CyclicBuffer cyc_;

  int closeCyclicBufferLocked(std::string & message);
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
  struct JointSample
  {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
  };

  enum class Interpolation { LINEAR, CUBIC, QUINTIC };

  std::unique_ptr<JointPositionControllerApi> api_;

  std::vector<int64_t> jointAxes_;
  std::vector<std::string> jointNames_;
  std::string jointTrajectoryTopic_;
  int bufferCycles_ = 0;
  int sampleCycles_ = 0;
  double publishPeriod_ = 0.0;
  double leadTime_ = 0.0;
  double streamTimeout_ = 0.0;
  double stopDeceleration_ = 0.0;
  std::string interpolationName_;
  Interpolation interpolation_ = Interpolation::QUINTIC;

  std::vector<int> axes_;
  std::map<std::string, size_t> indexByName_;
  double cycleTime_ = 0.0;

  std::atomic<bool> inExecution_{false};

  bool streaming_ = false;
  bool draining_ = false;
  JointSample last_;
  double lastStamp_ = 0.0;
  double segmentPeriod_ = 0.0;
  std::chrono::steady_clock::time_point lastReceived_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execActiveSub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr jointTrajectorySub_;
  rclcpp::TimerBase::SharedPtr watchdogTimer_;

  void setRosParameter();

  bool startStream();
  void stopStream(const char * reason);
  bool queueSegment(const JointSample & target, double duration);
  void queueStop();

  bool readPoint(
    const trajectory_msgs::msg::JointTrajectory & traj, size_t index, JointSample & sample);

  void execActiveCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void watchdogCallback();
};

#endif  // JOINT_POSITION_CONTROLLER_HPP_
