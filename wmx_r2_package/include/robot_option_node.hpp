// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef ROBOT_OPTION_NODE_HPP_
#define ROBOT_OPTION_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "wmx_r2_message/msg/robot_cartesian_pose.hpp"
#include "wmx_r2_message/msg/robot_status.hpp"
#include "wmx_r2_message/srv/robot_calc_forward_kinematics.hpp"
#include "wmx_r2_message/srv/robot_calc_inverse_kinematics.hpp"
#include "wmx_r2_message/srv/robot_get_coordinate.hpp"
#include "wmx_r2_message/srv/robot_id.hpp"
#include "wmx_r2_message/srv/robot_override_velocity.hpp"
#include "wmx_r2_message/srv/robot_set_coordinate.hpp"
#include "wmx_r2_message/srv/robot_set_robot_param.hpp"
#include "wmx_r2_message/srv/robot_start_motion.hpp"
#include "wmx_r2_message/srv/robot_start_ptp.hpp"

#include "WMX3Api.h"
#include "CoordinateApi.h"
#include "CoreMotionApi.h"
#include "KinematicsApi.h"
#include "RobotMotionApi.h"

class RobotOptionNodeApi
{
public:
  explicit RobotOptionNodeApi(const rclcpp::Logger & logger);
  ~RobotOptionNodeApi();

  int createDevice(std::string & message);
  void closeDevice();

  int setRobotParam(
    const std::string & paramFile, int32_t robotId, int32_t & outRobotId, int32_t & numJoints,
    std::string & message);
  int releaseRobot(int32_t robotId, std::string & message);

  bool hasRobot() const;
  int robotId() const;
  int numJoints() const;

  int updateRobotStatus(wmx3Api::RobotStatus & status, std::string & message);

  int startPtp(
    int32_t robotId, int32_t mode, bool useCartesian,
    const std::vector<double> & targetJoint,
    const wmx3Api::coordinate::CartesianPose & targetPose,
    char s, char e, char r, std::string & message);
  int startMotion(
    int32_t robotId, int32_t trajectoryType, bool isToolCoordinate,
    const wmx3Api::coordinate::CartesianPose & targetPose,
    const std::vector<wmx3Api::coordinate::CartesianPose> & throughPose,
    double arcAngle, std::string & message);

  int stopMotion(int32_t robotId, std::string & message);
  int pauseMotion(int32_t robotId, std::string & message);
  int resumeMotion(int32_t robotId, std::string & message);
  int clearMotionError(int32_t robotId, std::string & message);
  int eStop(int32_t robotId, std::string & message);
  int releaseEStop(int32_t robotId, std::string & message);

  int overrideVelocity(
    int32_t robotId, double velRatio, double accRatio, double decRatio, std::string & message);

  int setToolCoordinate(
    int32_t robotId, const wmx3Api::coordinate::CartesianPose & pose, std::string & message);
  int getToolCoordinate(
    int32_t robotId, wmx3Api::coordinate::CartesianPose & pose, std::string & message);
  int setWorkCoordinate(
    int32_t robotId, const wmx3Api::coordinate::CartesianPose & pose, std::string & message);
  int getWorkCoordinate(
    int32_t robotId, wmx3Api::coordinate::CartesianPose & pose, std::string & message);

  int calcForwardKinematics(
    int32_t robotId, const std::vector<double> & jointPosition,
    wmx3Api::coordinate::CartesianPose & toolPose, std::string & message);
  int calcInverseKinematics(
    int32_t robotId, const wmx3Api::coordinate::CartesianPose & toolPose, char s, char e, char r,
    std::vector<double> & jointPosition, std::string & message);

private:
  int checkRobot(int32_t robotId, std::string & message) const;
  int fillJoints(
    const std::vector<double> & source,
    double (& target)[wmx3Api::kinematics::constants::MAX_NUMBER_OF_JOINT],
    std::string & message) const;

  rclcpp::Logger logger_;

  const char * deviceName_ = "robot_option_node";
  unsigned int timeout_ = 10000;

  mutable std::mutex robotMutex_;
  bool robotLoaded_ = false;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::RobotMotion robot_;
  wmx3Api::RobotMotionParam robotMotionParam_;
};

class RobotOptionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  RobotOptionNode();
  ~RobotOptionNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::unique_ptr<RobotOptionNodeApi> api_;

  std::string robotParamFile_;
  int rate_ = 10;

  rclcpp::TimerBase::SharedPtr robotStatusTimer_;
  wmx_r2_message::msg::RobotStatus robotStatusMsg_;
  rclcpp_lifecycle::LifecyclePublisher<wmx_r2_message::msg::RobotStatus>::SharedPtr
    robotStatusPub_;

  rclcpp::Service<wmx_r2_message::srv::RobotSetRobotParam>::SharedPtr setRobotParamService_;
  rclcpp::Service<wmx_r2_message::srv::RobotId>::SharedPtr releaseRobotService_;
  rclcpp::Service<wmx_r2_message::srv::RobotStartPtp>::SharedPtr startPtpService_;
  rclcpp::Service<wmx_r2_message::srv::RobotStartMotion>::SharedPtr startMotionService_;
  rclcpp::Service<wmx_r2_message::srv::RobotId>::SharedPtr stopMotionService_;
  rclcpp::Service<wmx_r2_message::srv::RobotId>::SharedPtr pauseMotionService_;
  rclcpp::Service<wmx_r2_message::srv::RobotId>::SharedPtr resumeMotionService_;
  rclcpp::Service<wmx_r2_message::srv::RobotId>::SharedPtr clearMotionErrorService_;
  rclcpp::Service<wmx_r2_message::srv::RobotId>::SharedPtr eStopService_;
  rclcpp::Service<wmx_r2_message::srv::RobotId>::SharedPtr releaseEStopService_;
  rclcpp::Service<wmx_r2_message::srv::RobotOverrideVelocity>::SharedPtr
    overrideVelocityService_;
  rclcpp::Service<wmx_r2_message::srv::RobotSetCoordinate>::SharedPtr setToolCoordinateService_;
  rclcpp::Service<wmx_r2_message::srv::RobotGetCoordinate>::SharedPtr getToolCoordinateService_;
  rclcpp::Service<wmx_r2_message::srv::RobotSetCoordinate>::SharedPtr setWorkCoordinateService_;
  rclcpp::Service<wmx_r2_message::srv::RobotGetCoordinate>::SharedPtr getWorkCoordinateService_;
  rclcpp::Service<wmx_r2_message::srv::RobotCalcForwardKinematics>::SharedPtr
    calcForwardKinematicsService_;
  rclcpp::Service<wmx_r2_message::srv::RobotCalcInverseKinematics>::SharedPtr
    calcInverseKinematicsService_;

  void robotStatusStep();

  void setRobotParamCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotSetRobotParam::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotSetRobotParam::Response> response);
  void releaseRobotCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotId::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotId::Response> response);
  void startPtpCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotStartPtp::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotStartPtp::Response> response);
  void startMotionCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotStartMotion::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotStartMotion::Response> response);
  void stopMotionCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotId::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotId::Response> response);
  void pauseMotionCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotId::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotId::Response> response);
  void resumeMotionCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotId::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotId::Response> response);
  void clearMotionErrorCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotId::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotId::Response> response);
  void eStopCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotId::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotId::Response> response);
  void releaseEStopCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotId::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotId::Response> response);
  void overrideVelocityCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotOverrideVelocity::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotOverrideVelocity::Response> response);
  void setToolCoordinateCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotSetCoordinate::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotSetCoordinate::Response> response);
  void getToolCoordinateCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotGetCoordinate::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotGetCoordinate::Response> response);
  void setWorkCoordinateCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotSetCoordinate::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotSetCoordinate::Response> response);
  void getWorkCoordinateCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotGetCoordinate::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotGetCoordinate::Response> response);
  void calcForwardKinematicsCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotCalcForwardKinematics::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotCalcForwardKinematics::Response> response);
  void calcInverseKinematicsCallback(
    const std::shared_ptr<wmx_r2_message::srv::RobotCalcInverseKinematics::Request> request,
    std::shared_ptr<wmx_r2_message::srv::RobotCalcInverseKinematics::Response> response);
};

#endif  // ROBOT_OPTION_NODE_HPP_
