// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <memory>
#include <thread>
#include <sstream>
#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <queue>

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "AdvancedMotionApi.h"
#include "CyclicBufferApi.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/bool.hpp"

#define MAX_TRAJ_POINTS 1000

using wmx3Api::AdvancedMotion;
using wmx3Api::AdvMotion;
using wmx3Api::AxisSelection;
using wmx3Api::Config;
using wmx3Api::CoreMotion;
using wmx3Api::CyclicBuffer;
using wmx3Api::CyclicBufferSingleAxisStatus;
using wmx3Api::CyclicBufferMultiAxisCommands;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::WMX3Api;
using wmx3Api::OperationState;

class JointTrajectoryController : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  JointTrajectoryController();
  ~JointTrajectoryController();

  std::vector<int64_t> jointAxes_;
  std::string jointTrajectoryAction_;
  std::string jointTrajectoryTopic_;
  std::string wmxParamFilePath_;

  int err_;
  char errString_[256];

private:
  bool initialized_ = false;

  WMX3Api wmx3Lib_;
  CoreMotion wmx3LibCm_;
  CyclicBuffer wmx3LibCb_;
  AdvancedMotion wmx3LibAm_;
  AdvMotion::PointTimeSplineCommand spl;
  AdvMotion::SplinePoint pt_spl[MAX_TRAJ_POINTS];
  double time_spl[MAX_TRAJ_POINTS];
  AxisSelection axisSel;
  Config::AxisParam axisParam_;
  CyclicBufferSingleAxisStatus cycStatus;
  // CyclicBufferMultiAxisCommands cycCmds;
  bool isMoving;
  bool afterExecQuickStop;
  std::queue<CyclicBufferMultiAxisCommands> cmdQueue;
  std::vector<double> prePos;
  std::vector<double> preVel;
  std::vector<double> a;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr jointTrajectorySub_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  // Action server callback declarations
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    std::shared_ptr<GoalHandleFJT> goal_handle);

  void handle_accepted(std::shared_ptr<GoalHandleFJT> goal_handle);

  void execute(std::shared_ptr<GoalHandleFJT> goal_handle);

  void setRosParameter();
  void setWmxParam(char * path);
  void getWmxParam();
  void onEngineReady(std_msgs::msg::Bool::ConstSharedPtr msg);
  void onJointTrajectory(
    trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg);
  void logTrajectory(const trajectory_msgs::msg::JointTrajectory & trajectory);
  void calculateAB(std::vector<double> & a, int cycle, const std::vector<double> & positions, const std::vector<double> & velocities, std::vector<double> & prePos, std::vector<double> & preVel);
  void pushToCyclicBuffer(std::vector<double> & a, int cycle, const std::vector<double> & positions, const std::vector<double> & velocities, std::queue<CyclicBufferMultiAxisCommands> & cmdQueue, std::vector<double> & prePos, std::vector<double> & preVel, CyclicBufferMultiAxisCommands & cycCmds, AxisSelection & axisSel);
};

JointTrajectoryController::JointTrajectoryController()
: Node("joint_trajectory_controller")
{
  setRosParameter();

  auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
  engineReadySub_ = this->create_subscription<std_msgs::msg::Bool>(
    "wmx/engine/ready", ready_qos,
    std::bind(&JointTrajectoryController::onEngineReady, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller waiting for engine...");
  isMoving = false;
  afterExecQuickStop = false;
}

JointTrajectoryController::~JointTrajectoryController()
{
  RCLCPP_INFO(this->get_logger(), "Stop joint_trajectory_controller");

  if (initialized_) {
    wmx3LibAm_.advMotion->FreeSplineBuffer(0);

    err_ = wmx3Lib_.CloseDevice();
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(this->get_logger(), "Failed to close device");
    } else {
      RCLCPP_INFO(this->get_logger(), "Device closed");
    }
  }

  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is stopped");
}

void JointTrajectoryController::onEngineReady(std_msgs::msg::Bool::ConstSharedPtr msg)
{
  if (!msg->data || initialized_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Engine ready — initializing AdvancedMotion...");

  unsigned int timeout = 10000;
  err_ = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout);

  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    if (err_ == ErrorCode::StartProcessLockError) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to attach to device (lock busy, will retry on next signal).");
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to attach to device. Error=%d (%s)", err_, errString_);
    }
    return;
  }

  wmx3Lib_.SetDeviceName("joint_trajectory_controller");
  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");

  wmx3LibCm_ = CoreMotion(&wmx3Lib_);
  wmx3LibAm_ = AdvancedMotion(&wmx3Lib_);
  wmx3LibAm_.advMotion->CreateSplineBuffer(0, MAX_TRAJ_POINTS);
  wmx3LibCb_ = CyclicBuffer(&wmx3Lib_);
  axisSel.axisCount = 6;
  for (int i = 0; i < 6; ++i) {
    axisSel.axis[i] = i + 1;
  }
  err_ = wmx3LibCb_.OpenCyclicBuffer(&axisSel, 3000);
  if (err_ != 0) {
    wmx3LibCb_.ErrorToString(err_, errString_, 256);
    RCLCPP_ERROR(this->get_logger(), "OpenCyclicBuffer Error: %s", errString_);
    return;
  }

  setWmxParam(const_cast<char *>(wmxParamFilePath_.c_str()));
  getWmxParam();

  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    this,
    jointTrajectoryAction_,
    std::bind(
      &JointTrajectoryController::handle_goal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&JointTrajectoryController::handle_cancel, this, std::placeholders::_1),
    std::bind(&JointTrajectoryController::handle_accepted, this, std::placeholders::_1)
  );

  jointTrajectorySub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    jointTrajectoryTopic_, rclcpp::QoS(10),
    std::bind(&JointTrajectoryController::onJointTrajectory, this, std::placeholders::_1));

  initialized_ = true;
  engineReadySub_.reset();

  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is ready");
  for (int i = 0; i < 6; ++i) {
    prePos.push_back(0.0);
    preVel.push_back(0.0);
    a.push_back(0.0);
  }
}

void JointTrajectoryController::setWmxParam(char * path)
{
  Config::SystemParam sysParamError;
  Config::AxisParam axisParamError;
  err_ = wmx3LibCm_.config->ImportAndSetAll(path, &sysParamError, &axisParamError);
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Failed to set WMX params. Error=%d (%s)", err_, errString_);
    for (int axis : jointAxes_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "  [axis %d] AxisParam error flags: gearNum=%.0f gearDen=%.6f polarity=%d "
        "absEnc=%d maxSpd=%.0f maxSpdUnitNum=%.6f maxSpdUnitDen=%.6f singleTurnMode=%d "
        "singleTurnCnt=%u maxTrq=%.1f posTrq=%.1f negTrq=%.1f axisUnit=%.6f cmdMode=%d",
        axis,
        axisParamError.gearRatioNumerator[axis], axisParamError.gearRatioDenominator[axis],
        static_cast<int>(axisParamError.axisPolarity[axis]),
        static_cast<int>(axisParamError.absoluteEncoderMode[axis]),
        axisParamError.maxMotorSpeed[axis],
        axisParamError.maxMotorSpeedUnitNumerator[axis],
        axisParamError.maxMotorSpeedUnitDenominator[axis],
        static_cast<int>(axisParamError.singleTurnMode[axis]),
        axisParamError.singleTurnEncoderCount[axis],
        axisParamError.maxTrqLimit[axis], axisParamError.positiveTrqLimit[axis],
        axisParamError.negativeTrqLimit[axis], axisParamError.axisUnit[axis],
        static_cast<int>(axisParamError.axisCommandMode[axis]));
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Success to set WMX params");
  }
}

void JointTrajectoryController::getWmxParam()
{
  err_ = wmx3LibCm_.config->GetAxisParam(&axisParam_);
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Failed to get axis params. Error=%d (%s)", err_, errString_);
  } else {
    for (int axis : jointAxes_) {
      RCLCPP_INFO(
        this->get_logger(), "axis: %d, numerator: %f", axis, axisParam_.gearRatioNumerator[axis]);
      RCLCPP_INFO(
        this->get_logger(), "axis: %d, denominator: %f", axis,
        axisParam_.gearRatioDenominator[axis]);
      RCLCPP_INFO(
        this->get_logger(), "axis: %d, polarity: %d", axis,
        (int)axisParam_.axisPolarity[axis]);
      RCLCPP_INFO(
        this->get_logger(), "axis: %d, abs encoder: %d", axis,
        axisParam_.absoluteEncoderMode[axis]);
      RCLCPP_INFO(this->get_logger(), "axis: %d, mode: %d", axis, axisParam_.axisCommandMode[axis]);
    }
  }
}

void JointTrajectoryController::setRosParameter()
{
  this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  this->declare_parameter<std::string>(
    "joint_trajectory_action", "/joint_trajectory_action/no_param");
  this->declare_parameter<std::string>("joint_trajectory_topic", "/joint_trajectory");
  this->declare_parameter<std::string>("wmx_param_file_path", "/joint_trajectory/no_param");

  this->get_parameter("joint_axes", jointAxes_);
  this->get_parameter("joint_trajectory_action", jointTrajectoryAction_);
  this->get_parameter("joint_trajectory_topic", jointTrajectoryTopic_);
  this->get_parameter("wmx_param_file_path", wmxParamFilePath_);

  std::string joint_axes_str;
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    if (i > 0) {joint_axes_str += ", ";}
    joint_axes_str += std::to_string(jointAxes_[i]);
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", joint_axes_str.c_str());
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_action: %s", jointTrajectoryAction_.c_str());
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_topic: %s", jointTrajectoryTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "wmx_param_file_path: %s", wmxParamFilePath_.c_str());
  RCLCPP_INFO(this->get_logger(), "===========================");
}

rclcpp_action::GoalResponse JointTrajectoryController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryController::handle_cancel(
  std::shared_ptr<GoalHandleFJT> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryController::handle_accepted(std::shared_ptr<GoalHandleFJT> goal_handle)
{
  std::thread{std::bind(&JointTrajectoryController::execute, this, std::placeholders::_1),
    goal_handle}.detach();
}

void JointTrajectoryController::execute(std::shared_ptr<GoalHandleFJT> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const auto & trajectory = goal->trajectory;

  int num_points = trajectory.points.size();

  RCLCPP_INFO(this->get_logger(), "Received a new trajectory goal! Point number: [%d]", num_points);

  auto result = std::make_shared<FollowJointTrajectory::Result>();
  double timeMilliseconds;

  if (num_points > MAX_TRAJ_POINTS) {
    RCLCPP_WARN(
      this->get_logger(),
      "Too many trajectory point size! "
      "current points:%d / max traj points:%d \nAborting current goal.",
      num_points, MAX_TRAJ_POINTS);
    goal_handle->abort(result);
    return;
  }

  logTrajectory(trajectory);

  // Generate spline commands from trajectory.points
  axisSel.axisCount = jointAxes_.size();
  spl.dimensionCount = jointAxes_.size();
  for (size_t j = 0; j < jointAxes_.size(); ++j) {
    axisSel.axis[j] = jointAxes_[j];
    spl.axis[j] = jointAxes_[j];
  }

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & pt = trajectory.points[i];
    timeMilliseconds = rclcpp::Duration(pt.time_from_start).seconds() * 1000;
    time_spl[i] = timeMilliseconds;

    for (size_t j = 0; j < jointAxes_.size(); ++j) {
      pt_spl[i].pos[j] = pt.positions.at(j);
    }
  }

  // If first time interval is not zero, make it zero
  if (time_spl[0] != 0.0) {
    time_spl[0] = 0.0;
  }

  // if last time interval is less than 1ms, ignore the last point.
  double last = trajectory.points.size() - 1;
  if (rclcpp::Duration(trajectory.points[last].time_from_start).seconds() -
    rclcpp::Duration(trajectory.points[last - 1].time_from_start).seconds() < 1e-3)
  {
    num_points -= 1;
  }

  if (num_points == 0) {
    RCLCPP_INFO(this->get_logger(), "Point count is zero. It is already in the targeted position");
  } else {
    RCLCPP_INFO(this->get_logger(), "Command Start!!!");
    err_ = wmx3LibAm_.advMotion->StartCSplinePos(0, &spl, num_points, pt_spl, time_spl);
    if (err_ != 0) {
      wmx3LibAm_.ErrorToString(err_, errString_, 256);
      RCLCPP_ERROR(this->get_logger(), "StartCSplinePos Error: %s", errString_);
      result->error_code = err_;
      goal_handle->abort(result);
      return;
    }

    while (true) {
      if (goal_handle->is_canceling()) {
        wmx3LibCm_.motion->Stop(&axisSel);
        wmx3LibCm_.motion->Wait(&axisSel);
        result->error_code = 0;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled, axes stopped");
        return;
      }

      CoreMotionStatus cmStatus;
      wmx3LibCm_.GetStatus(&cmStatus);
      bool all_done = true;
      for (int axis : jointAxes_) {
        if (!cmStatus.axesStatus[axis].inPos) {all_done = false; break;}
      }
      if (all_done) {break;}

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  result->error_code = 0;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully");
}

void JointTrajectoryController::onJointTrajectory(
  trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg)
{
  // TODO: Implement topic-based JointTrajectory handling here.
  // Example:
  // - Validate msg->joint_names and msg->points
  // - Convert trajectory points into WMX spline commands
  // - Start/stop motion or update controller state
  if (msg->points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "msg->points is empty");
    if (isMoving) {
      err_ = wmx3LibCb_.ExecQuickStop(&axisSel);
      if (err_ != 0) {
        wmx3LibCb_.ErrorToString(err_, errString_, 256);
        RCLCPP_ERROR(this->get_logger(), "ExecQuickStop Error: %s", errString_);
        return;
      }
      else {
        RCLCPP_INFO(this->get_logger(), "Executed Quick Stop!!");
        afterExecQuickStop = true;
      }
      isMoving = false;
    }
    return;
  }

  CyclicBufferMultiAxisCommands cycCmds;
  const auto& point = msg->points[0];
  const auto& positions = point.positions;
  const auto& velocities = point.velocities;
  axisSel.axisCount = msg->joint_names.size();
  if (msg->header.frame_id == "start_point_trajectory") {
    if (afterExecQuickStop) {
      RCLCPP_INFO(this->get_logger(), "Trajectory is reproduced.");
      afterExecQuickStop = false;
      for (int i = 0; i < axisSel.axisCount; ++i) {
        axisSel.axis[i] = i + 1;
        cycCmds.cmd[axisSel.axis[i]].type = wmx3Api::CyclicBufferCommandType::AbsolutePos;
        cycCmds.cmd[axisSel.axis[i]].command = positions[i];
        cycCmds.cmd[axisSel.axis[i]].intervalCycles = 2000;
        prePos[i] = positions[i];
        preVel[i] = velocities[i] / 1.02;
      }
      cmdQueue.push(cycCmds);
    }
    else {
      for (int i = 0; i < axisSel.axisCount; ++i) {
        prePos[i] = positions[i];
        preVel[i] = velocities[i] / 1.02;
      }
      return;
    }
  }
  else {
    calculateAB(a, 102, positions, velocities, prePos, preVel);
    pushToCyclicBuffer(a, 102, positions, velocities, cmdQueue, prePos, preVel, cycCmds, axisSel);
  }
  
  CoreMotionStatus cmStatus;
  wmx3LibCm_.GetStatus(&cmStatus);
  bool axisIsStopping = false;
  for (int i = 0; i < axisSel.axisCount; ++i) {
    if (cmStatus.axesStatus[axisSel.axis[i]].opState == OperationState::Stop) {
      axisIsStopping = true;
    }
  }
  if (!axisIsStopping) {
    int size = cmdQueue.size();
    if (size > 1) {
      RCLCPP_INFO(this->get_logger(), "Queue size is [%zu]", size);
    }
    while (!cmdQueue.empty()) {
      CyclicBufferMultiAxisCommands cmds = cmdQueue.front();
      err_ = wmx3LibCb_.AddCommand(&axisSel, &cmds);
      if (err_ != 0) {
        wmx3LibCb_.ErrorToString(err_, errString_, 256);
        RCLCPP_ERROR(this->get_logger(), "AddCommand Error: %s", errString_);
        return;
      }
      cmdQueue.pop();
    }
    err_ = wmx3LibCb_.Execute(&axisSel);
    if (err_ != 0) {
      wmx3LibCb_.ErrorToString(err_, errString_, 256);
      RCLCPP_ERROR(this->get_logger(), "Execute Error: %s", errString_);
      return;
    }
  }

  isMoving = true;
}

void JointTrajectoryController::logTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory){
  std::ostringstream jn;
  for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    if (i) {jn << ", ";}
    jn << trajectory.joint_names[i];
  }
  RCLCPP_INFO(this->get_logger(), "Joint Names: [%s]", jn.str().c_str());

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & pt = trajectory.points[i];
    std::ostringstream pos, vel, acc;
    for (size_t k = 0; k < pt.positions.size(); ++k) {
      if (k) {pos << ", ";}
      pos << pt.positions[k];
    }
    for (size_t k = 0; k < pt.velocities.size(); ++k) {
      if (k) {vel << ", ";}
      vel << pt.velocities[k];
    }
    for (size_t k = 0; k < pt.accelerations.size(); ++k) {
      if (k) {acc << ", ";}
      acc << pt.accelerations[k];
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Point %zu: Positions: [%s], Velocities: [%s], "
      "Accelerations: [%s], TimeFromStart: %d s %u ns",
      i, pos.str().c_str(), vel.str().c_str(), acc.str().c_str(),
      pt.time_from_start.sec, pt.time_from_start.nanosec);

    if (i != 0) {
      rclcpp::Duration duration_cur(trajectory.points[i].time_from_start);
      rclcpp::Duration duration_pre(trajectory.points[i - 1].time_from_start);
      RCLCPP_INFO(
        this->get_logger(), "Time interval: %f",
        (duration_cur - duration_pre).seconds());
    }
  }
}

void JointTrajectoryController::calculateAB(std::vector<double> & a, int cycle, const std::vector<double> & positions, const std::vector<double> & velocities, std::vector<double> & prePos, std::vector<double> & preVel) {
  double dt = cycle / 1000.0;
  for (int i = 0; i < a.size(); ++i) {
    a[i] = (velocities[i] / 1.02 - preVel[i]) / dt;
  }
}

void JointTrajectoryController::pushToCyclicBuffer(std::vector<double> & a, int cycle, const std::vector<double> & positions, const std::vector<double> & velocities, std::queue<CyclicBufferMultiAxisCommands> & cmdQueue, std::vector<double> & prePos, std::vector<double> & preVel, CyclicBufferMultiAxisCommands & cycCmds, AxisSelection & axisSel) {
  std::vector<double> preCyclePos;
  for (int j = 0; j < axisSel.axisCount; ++j) {
    preCyclePos.push_back(0.0);
  }
  for (int i = 0; i < cycle; ++i) {
    double dt = (i + 1) / 1000.0;
    for (int j = 0; j < axisSel.axisCount; ++j) {
      axisSel.axis[j] = j + 1;
      cycCmds.cmd[axisSel.axis[j]].type = wmx3Api::CyclicBufferCommandType::AbsolutePos;
      cycCmds.cmd[axisSel.axis[j]].command = 0.5 * a[j] * dt * dt + preVel[j] * dt + prePos[j];
      cycCmds.cmd[axisSel.axis[j]].intervalCycles = 1;
      preCyclePos[j] = 0.5 * a[j] * dt * dt + preVel[j] * dt;
      if (i + 1 == cycle) {
        prePos[j] = cycCmds.cmd[axisSel.axis[j]].command;
        preVel[j] = velocities[j] * 100.0 / 102.0;
      }
    }
    cmdQueue.push(cycCmds);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTrajectoryController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
