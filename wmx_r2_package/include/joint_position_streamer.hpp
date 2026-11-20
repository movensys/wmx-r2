// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef JOINT_POSITION_STREAMER_HPP_
#define JOINT_POSITION_STREAMER_HPP_

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "CyclicBufferApi.h"

class JointPositionStreamerApi
{
public:
  JointPositionStreamerApi(const rclcpp::Logger & logger, const std::string & deviceName);
  ~JointPositionStreamerApi();

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

  std::string deviceName_;
  unsigned int timeout_ = 10000;

  mutable std::mutex deviceMutex_;

  wmx3Api::AxisSelection axisSel_;
  bool bufferOpen_ = false;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::CoreMotion cm_;
  wmx3Api::CyclicBuffer cyc_;

  int closeCyclicBufferLocked(std::string & message);
};

class JointPositionStreamer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ~JointPositionStreamer() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  struct JointSample
  {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
  };

  enum class Interpolation { LINEAR, CUBIC, QUINTIC };

  JointPositionStreamer(
    const std::string & nodeName, const std::string & deviceName,
    const std::vector<std::string> & defaultBlockTopics);

  virtual void onStreamStopped(const char * /*reason*/) {}

  bool startStream();
  void stopStream(const char * reason);

  bool queueSegment(const JointSample & target, double duration);
  bool planSegment(double duration, size_t & count);
  bool appendSegment(const JointSample & target, size_t count);

  double sampleTime() const {return cycleTime_ * sampleCycles_;}

  void limitAcceleration(std::vector<std::vector<double>> & samples);
  bool settled() const;
  void queueSettle();
  void queueStop();

  bool readPoint(
    const trajectory_msgs::msg::JointTrajectory & traj, size_t index, JointSample & sample);

  bool blocked() const {return inExecution_.load();}

  std::unique_ptr<JointPositionStreamerApi> api_;

  std::vector<int64_t> jointAxes_;
  std::vector<std::string> jointNames_;
  int bufferCycles_ = 0;
  int sampleCycles_ = 0;
  double leadTime_ = 0.0;
  double streamTimeout_ = 0.0;
  double stopDeceleration_ = 0.0;
  std::vector<double> maxVelocity_;
  std::vector<double> maxAcceleration_;
  std::string interpolationName_;
  Interpolation interpolation_ = Interpolation::CUBIC;
  std::vector<std::string> blockTopics_;

  std::vector<int> axes_;
  std::map<std::string, size_t> indexByName_;
  double cycleTime_ = 0.0;

  bool streaming_ = false;
  bool draining_ = false;
  JointSample last_;
  std::vector<double> refLast_;
  std::vector<double> outPos_;
  std::vector<double> outVel_;
  std::chrono::steady_clock::time_point lastReceived_;

  std::string nodeName_;

private:
  std::atomic<bool> inExecution_{false};
  std::vector<bool> blockFlags_;

  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> blockSubs_;
  rclcpp::TimerBase::SharedPtr watchdogTimer_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr streamActivePub_;

  void setRosParameter(const std::vector<std::string> & defaultBlockTopics);
  void publishStreamActive(bool active);

  void blockCallback(size_t index, const std_msgs::msg::Bool::SharedPtr msg);
  void watchdogCallback();
};

class JointPositionController : public JointPositionStreamer
{
public:
  JointPositionController();

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::string jointTrajectoryTopic_;
  double publishPeriod_ = 0.0;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr jointTrajectorySub_;

  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
};

class DynamicTrajectoryController : public JointPositionStreamer
{
public:
  DynamicTrajectoryController();

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  void onStreamStopped(const char * reason) override;

private:
  struct Waypoint
  {
    double time = 0.0;
    JointSample state;
    bool hasVelocity = false;
  };

  std::string jointTrajectoryTopic_;
  double slicePeriod_ = 0.01;

  std::vector<Waypoint> trajectory_;
  size_t segment_ = 0;
  double tau_ = 0.0;
  bool running_ = false;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr jointTrajectorySub_;
  rclcpp::TimerBase::SharedPtr sliceTimer_;

  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void sliceCallback();
  JointSample sampleAt(double t);
  void clearTrajectory();
};

#endif  // JOINT_POSITION_STREAMER_HPP_
