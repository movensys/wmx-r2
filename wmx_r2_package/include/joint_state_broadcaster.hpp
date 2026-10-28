// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef JOINT_STATE_BROADCASTER_HPP_
#define JOINT_STATE_BROADCASTER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "wmx_r2_message/srv/get_axis_param.hpp"
#include "wmx_r2_message/srv/set_axes.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "IOApi.h"

class JointStateBroadcasterApi
{
public:
  struct AxisFeedback
  {
    double actualPos = 0.0;
    double actualVelocity = 0.0;
  };

  explicit JointStateBroadcasterApi(const rclcpp::Logger & logger);
  ~JointStateBroadcasterApi();

  int createDevice(std::string & message);
  void closeDevice();

  int getStatus(
    const std::vector<int64_t> & axes, std::vector<AxisFeedback> & feedback,
    std::string & message);

  int getOutBit(int32_t addr, int32_t bit, uint8_t & data, std::string & message);

  int setServoOn(int axis, int newStatus, std::string & message);

private:
  rclcpp::Logger logger_;

  const char * deviceName_ = "joint_state_broadcaster";
  unsigned int timeout_ = 10000;

  mutable std::mutex deviceMutex_;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::CoreMotion cm_;
  wmx3Api::IO io_;
};

class JointStateBroadcaster : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  JointStateBroadcaster();
  ~JointStateBroadcaster() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::unique_ptr<JointStateBroadcasterApi> api_;

  int jointFeedbackRate_ = 0;
  float gripperOpenValue_ = 0.0f;
  float gripperCloseValue_ = 0.0f;
  std::vector<int64_t> jointAxes_;
  std::vector<std::string> jointNames_;
  std::vector<std::string> gripperJointNames_;
  std::vector<int64_t> gripperAddress_;
  std::string encoderJointTopic_;
  std::string isaacsimJointTopic_;
  std::string gazeboPositionTopic_;
  std::string gazeboVelocityTopic_;
  std::vector<int64_t> gazeboPositionAxes_;
  std::vector<int64_t> gazeboVelocityAxes_;

  rclcpp::CallbackGroup::SharedPtr axisClientCbGroup_;
  rclcpp::Client<wmx_r2_message::srv::SetAxes>::SharedPtr clearAlarmClient_;
  rclcpp::Client<wmx_r2_message::srv::SetAxes>::SharedPtr setAxisOnClient_;
  rclcpp::Client<wmx_r2_message::srv::GetAxisParam>::SharedPtr getAxisParamClient_;

  // SCHED_FIFO priority of the feedback loop thread (same name as in ros2_control). 0 = off.
  // 피드백 루프 스레드의 SCHED_FIFO 우선순위 (ros2_control과 같은 이름). 0 = 끔.
  int threadPriority_ = 50;

  // The feedback loop runs on this thread, not on a ROS timer. loopRunning_ stops it.
  // 피드백 루프는 ROS 타이머가 아니라 이 스레드에서 돈다. loopRunning_으로 멈춘다.
  std::thread loopThread_;
  std::atomic<bool> loopRunning_{false};

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr encoderJointPub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr isaacsimJointPub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    gazeboPositionPub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    gazeboVelocityPub_;

  void setRosParameter();

  void servoOff();

  bool callSetAxesService(
    rclcpp::Client<wmx_r2_message::srv::SetAxes>::SharedPtr client,
    const std::string & serviceName,
    const std::vector<int64_t> & axes,
    const std::vector<int64_t> & data);

  bool getAxisParam(std::string & message);

  void feedbackLoop();

  void publishJointState();
};

#endif  // JOINT_STATE_BROADCASTER_HPP_
