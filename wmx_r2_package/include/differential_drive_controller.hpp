// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef DIFFERENTIAL_DRIVE_CONTROLLER_HPP_
#define DIFFERENTIAL_DRIVE_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

class DifferentialDriveControllerApi
{
public:
  struct Config
  {
    double accTimeMilliseconds = 1.0;
    double decTimeMilliseconds = 1.0;
  };

  struct AxisFeedback
  {
    double actualVelocity = 0.0;
    bool servoOn = false;
    bool ampAlarm = false;
  };

  DifferentialDriveControllerApi(const rclcpp::Logger & logger, const Config & config);
  ~DifferentialDriveControllerApi();

  int createDevice(std::string & message);
  void closeDevice();

  int getStatus(
    int leftAxis, int rightAxis,
    AxisFeedback & left, AxisFeedback & right, bool & communicating,
    std::string & message);

  int startVel(int axis, double omega, std::string & message);

private:
  rclcpp::Logger logger_;
  Config config_;

  const char * deviceName_ = "differential_drive_controller";
  unsigned int timeout_ = 10000;

  mutable std::mutex deviceMutex_;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::CoreMotion cm_;
};

class DifferentialDriveController : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  DifferentialDriveController();
  ~DifferentialDriveController() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::unique_ptr<DifferentialDriveControllerApi> api_;

  int leftAxis_ = 0;
  int rightAxis_ = 1;
  int rate_ = 100;
  double accTime_ = 1.0;
  double decTime_ = 1.0;
  double wheelRadius_ = 0.095;
  double wheelToWheel_ = 0.55;

  double cmdVelTimeout_ = 0.25;
  bool publishTf_ = false;
  std::string odomFrame_;
  std::string baseFrame_;
  std::vector<std::string> jointName_;

  std::string cmdVelTopic_;
  std::string cmdOmegaTopic_;
  std::string encoderOmegaTopic_;
  std::string encoderOdometryTopic_;

  double poseX_ = 0.0;
  double poseY_ = 0.0;
  double poseTheta_ = 0.0;

  rclcpp::Time prevFeedbackTime_;
  bool haveFeedbackTime_ = false;

  geometry_msgs::msg::Twist lastCmdVel_;
  rclcpp::Time lastCmdVelTime_;
  bool haveCmdVel_ = false;

  double sentOmegaLeft_ = 0.0;
  double sentOmegaRight_ = 0.0;
  bool sentOmegaValid_ = false;

  rclcpp::TimerBase::SharedPtr controlTimer_;
  rclcpp::TimerBase::SharedPtr feedbackTimer_;

  rclcpp::CallbackGroup::SharedPtr controlCbGroup_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmdVelStampedSub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr cmdOmegaPub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
    encoderOmegaPub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr encoderOdometryPub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

  void setRosParameter();
  bool parametersValid() const;

  void cmdStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void controlStep();
  void publishMotorFeedback();
  void commandWheels(double omegaLeft, double omegaRight);
  void stopWheelsOnFault();
  bool startVel(int axis, double omega);

  void inverseKinematics(
    double linear, double angular, double & omegaLeft, double & omegaRight) const;
  void forwardKinematics(
    double omegaLeft, double omegaRight, double & linear, double & angular) const;
  void odometryPoseCalculation(double ds, double dtheta);

  void publishCmdOmega(const rclcpp::Time & stamp);
  void publishOmega(const rclcpp::Time & stamp, double omegaLeft, double omegaRight);
  void publishOdometry(const rclcpp::Time & stamp, double linear, double angular);
  void publishTf(const rclcpp::Time & stamp);

  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
  static double sinc(double a);
};

#endif  // DIFFERENTIAL_DRIVE_CONTROLLER_HPP_
