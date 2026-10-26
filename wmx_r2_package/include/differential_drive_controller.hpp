// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef DIFFERENTIAL_DRIVE_CONTROLLER_HPP_
#define DIFFERENTIAL_DRIVE_CONTROLLER_HPP_

#include <atomic>
#include <cassert>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

namespace diff_drive
{
struct BodyVel
{
  double linear = 0.0;
  double angular = 0.0;
};

struct WheelOmega
{
  double left = 0.0;
  double right = 0.0;
};

struct DiffDriveModel
{
  double wheel_radius = 0.0;
  double wheel_separation = 0.0;

  WheelOmega inverse(const BodyVel & cmd) const
  {
    assert(wheel_radius > 0.0);
    return {
      (2.0 * cmd.linear - cmd.angular * wheel_separation) / (2.0 * wheel_radius),
      (2.0 * cmd.linear + cmd.angular * wheel_separation) / (2.0 * wheel_radius)};
  }

  BodyVel forward(const WheelOmega & omega) const
  {
    assert(wheel_radius > 0.0 && wheel_separation > 0.0);
    return {
      (omega.right * wheel_radius + omega.left * wheel_radius) / 2.0,
      (omega.right * wheel_radius - omega.left * wheel_radius) / wheel_separation};
  }

  BodyVel forwardDelta(double d_phi_left, double d_phi_right) const
  {
    // DiffDriveModel::forward, not std::forward (cpplint IWYU false positive):
    return forward({d_phi_left, d_phi_right});  // NOLINT(build/include_what_you_use)
  }
};

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

class OdometryIntegrator
{
public:
  explicit OdometryIntegrator(double straight_eps = 1e-3)
  : straight_eps_(straight_eps) {}

  void odometryPoseCalculation(const BodyVel & vel, double dt)
  {
    if (!std::isfinite(dt) || dt <= 0.0) {return;}
    const double next_theta = pose_.theta + vel.angular * dt;
    if (std::abs(vel.angular) < straight_eps_) {
      const double dist = vel.linear * dt;
      pose_.x += dist * std::cos(pose_.theta);
      pose_.y += dist * std::sin(pose_.theta);
    } else {
      const double radius = vel.linear / vel.angular;
      pose_.x += radius * (std::sin(next_theta) - std::sin(pose_.theta));
      pose_.y -= radius * (std::cos(next_theta) - std::cos(pose_.theta));
    }
    pose_.theta = next_theta;
  }

  void odometryPoseCalculation(double ds, double dtheta)
  {
    if (!std::isfinite(ds) || !std::isfinite(dtheta)) {return;}  // ignore invalid steps
    const double half = 0.5 * dtheta;
    const double mid = pose_.theta + half;
    const double k = ds * sinc(half);
    pose_.x += k * std::cos(mid);
    pose_.y += k * std::sin(mid);
    pose_.theta += dtheta;
  }

  const Pose2D & pose() const {return pose_;}
  void reset(const Pose2D & p = {}) {pose_ = p;}

private:
  static double sinc(double a)
  {
    if (std::abs(a) < 1e-8) {return 1.0 - a * a / 6.0;}
    return std::sin(a) / a;
  }

  Pose2D pose_;
  double straight_eps_;
};

struct OdomDelta
{
  double linear = 0.0;
  double angular = 0.0;
};

class OdomDeltaAccumulator
{
public:
  void odometryDeltaAccumulation(const BodyVel & vel, double dt)
  {
    if (!std::isfinite(dt) || dt <= 0.0) {return;}  // ignore invalid/negative dt
    delta_.linear += std::abs(vel.linear * dt);
    delta_.angular += std::abs(vel.angular * dt);
  }

  void odometryDeltaAccumulation(double ds, double dtheta)
  {
    if (!std::isfinite(ds) || !std::isfinite(dtheta)) {return;}  // ignore invalid steps
    delta_.linear += std::abs(ds);
    delta_.angular += std::abs(dtheta);
  }

  OdomDelta take()
  {
    const OdomDelta out = delta_;
    delta_ = {};
    return out;
  }

  const OdomDelta & peek() const {return delta_;}

private:
  OdomDelta delta_;
};

struct BodyAccel
{
  double linear = 0.0;
  double angular = 0.0;
};

class AccelEstimator
{
public:
  explicit AccelEstimator(double alpha = 0.3, double vel_epsilon = 1e-4)
  : alpha_(alpha), vel_epsilon_(vel_epsilon) {}

  BodyAccel update(const BodyVel & vel, double dt)
  {
    if (!primed_ || !std::isfinite(dt) || dt <= 0.0) {
      prev_ = vel;
      primed_ = true;
      filtered_ = {};
      return filtered_;
    }

    const double raw_linear = (vel.linear - prev_.linear) / dt;
    const double raw_angular = (vel.angular - prev_.angular) / dt;

    filtered_.linear = filterAxis(filtered_.linear, raw_linear, vel.linear, prev_.linear);
    filtered_.angular = filterAxis(filtered_.angular, raw_angular, vel.angular, prev_.angular);

    prev_ = vel;
    return filtered_;
  }

  void reset()
  {
    primed_ = false;
    filtered_ = {};
    prev_ = {};
  }

private:
  double filterAxis(double prev_filtered, double raw, double v_now, double v_prev) const
  {
    if (std::abs(v_now) < vel_epsilon_ && std::abs(v_prev) < vel_epsilon_) {
      return 0.0;
    }
    return alpha_ * raw + (1.0 - alpha_) * prev_filtered;
  }

  double alpha_;
  double vel_epsilon_;
  bool primed_ = false;
  BodyVel prev_;
  BodyAccel filtered_;
};

}  // namespace diff_drive

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
    double actualPos = 0.0;
    double actualVelocity = 0.0;
    bool servoOn = false;
    bool ampAlarm = false;
  };

  DifferentialDriveControllerApi(const rclcpp::Logger & logger, const Config & config);
  ~DifferentialDriveControllerApi();

  int attachDevice(std::string & message);
  void releaseDevice();

  int getStatus(
    int leftAxis, int rightAxis,
    AxisFeedback & left, AxisFeedback & right, bool & communicating,
    std::string & message);

  int startVel(int axis, double omega, std::string & message);

  bool isDeviceOpen() const {return isDeviceAttached_;}

private:
  rclcpp::Logger logger_;
  Config config_;

  const char * deviceName_ = "differential_drive_controller";
  unsigned int timeout_ = 10000;

  mutable std::mutex deviceMutex_;
  bool isDeviceAttached_ = false;

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
  double accelPublishRate_ = 10.0;
  double accelAlpha_ = 0.3;
  bool publishTf_ = false;
  std::string odomFrame_;
  std::string baseFrame_;
  double posUnitScale_ = 1.0;
  double jumpGuardTol_ = 0.5;

  std::string cmdVelTopic_;
  std::string encoderOmegaTopic_;
  std::string encoderOdometryTopic_;
  std::string odomDeltasTopic_;
  std::string odomAccelTopic_;

  std::atomic<bool> isNodeActive_{false};

  diff_drive::DiffDriveModel model_;
  diff_drive::OdometryIntegrator integrator_;
  diff_drive::OdomDeltaAccumulator deltas_;
  std::unique_ptr<diff_drive::AccelEstimator> accel_;

  rclcpp::Time prevLoopTime_;
  double prevPosLeft_ = 0.0;
  double prevPosRight_ = 0.0;
  bool havePrev_ = false;
  rclcpp::Time prevAccelTime_;
  bool haveAccelClock_ = false;

  geometry_msgs::msg::Twist cmdVelMsg_;
  rclcpp::Time lastCmdTime_;
  bool haveCmd_ = false;

  double lastSentLeft_ = 0.0;
  double lastSentRight_ = 0.0;
  bool lastSentValid_ = false;

  rclcpp::TimerBase::SharedPtr controlTimer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmdVelStampedSub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    encoderOmegaPub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr encoderOdometryPub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr odomDeltasPub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::AccelStamped>::SharedPtr odomAccelPub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

  bool isNodeActive() const;
  std::string notActiveMessage();

  void setRosParameter();

  void cmdStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void controlStep();
  void commandWheels(double omegaLeft, double omegaRight);
  bool setVelocity(int axis, double omega);

  void publishOmega(const diff_drive::WheelOmega & enc);
  void publishOdometry(const rclcpp::Time & stamp, const diff_drive::BodyVel & body);
  void publishDeltas(const rclcpp::Time & stamp);
  void publishAccel(const rclcpp::Time & stamp, const diff_drive::BodyVel & body);
  void publishTf(const rclcpp::Time & stamp);

  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
};

#endif  // DIFFERENTIAL_DRIVE_CONTROLLER_HPP_
