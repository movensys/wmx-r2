// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_robot_option_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>

#include "wmx_qos_compat.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::coordinate::CartesianPose;
using wmx3Api::kinematics::Kinematics;
using wmx3Api::kinematics::PTPParam;

namespace
{
constexpr int maxJoints = wmx3Api::kinematics::constants::MAX_NUMBER_OF_JOINT;

// A URDF carries no robot id, so the one registered at configure is always 0.
// An XML takes its id from the file's own <Robot ID="..."> instead.
constexpr int32_t kUrdfRobotId = 0;

std::string errorToString(int err)
{
  char errString[256] = {};
  Kinematics::ErrorToString(err, errString, sizeof(errString));
  return errString;
}

std::string failureText(const std::string & call, const std::string & where, int err)
{
  return call + " failed. " + where + " Error=" + std::to_string(err) +
         " (" + errorToString(err) + ")";
}

std::string robotIdText(int32_t robotId)
{
  return "robotId=" + std::to_string(robotId);
}

std::chrono::nanoseconds periodFromRate(int rate)
{
  return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / static_cast<double>(rate)));
}

bool endsWith(const std::string & text, const std::string & suffix)
{
  if (text.size() < suffix.size()) {
    return false;
  }

  std::string tail = text.substr(text.size() - suffix.size());
  std::transform(tail.begin(), tail.end(), tail.begin(), ::tolower);
  return tail == suffix;
}

CartesianPose toCartesianPose(const wmx_r2_message::msg::RobotCartesianPose & pose)
{
  return CartesianPose(pose.x, pose.y, pose.z, pose.u, pose.v, pose.w);
}

wmx_r2_message::msg::RobotCartesianPose toPoseMsg(const CartesianPose & pose)
{
  wmx_r2_message::msg::RobotCartesianPose msg;
  msg.x = pose.point.x;
  msg.y = pose.point.y;
  msg.z = pose.point.z;
  msg.u = pose.rotation.u;
  msg.v = pose.rotation.v;
  msg.w = pose.rotation.w;
  return msg;
}

std::vector<CartesianPose> toCartesianPoses(
  const std::vector<wmx_r2_message::msg::RobotCartesianPose> & poses)
{
  std::vector<CartesianPose> result;
  result.reserve(poses.size());
  for (const auto & pose : poses) {
    result.push_back(toCartesianPose(pose));
  }
  return result;
}

}  // namespace

WmxRobotOptionNodeApi::WmxRobotOptionNodeApi(const rclcpp::Logger & logger)
: logger_(logger), robot_(&wmx3Lib_)
{
}

WmxRobotOptionNodeApi::~WmxRobotOptionNodeApi()
{
  closeDevice();
}

int WmxRobotOptionNodeApi::createDevice(std::string & message)
{
  int err = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout_);
  if (err != ErrorCode::None) {
    if (err == ErrorCode::StartProcessLockError) {
      message = "Failed to attach to device (lock busy). Is the engine communicating?";
    } else {
      message = "Failed to attach to device. Error=" + std::to_string(err) +
        " (" + errorToString(err) + ")";
    }
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = wmx3Lib_.SetDeviceName(deviceName_);
  if (err != ErrorCode::None) {
    message = "Failed to name the device '" + std::string(deviceName_) + "'. Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    wmx3Lib_.CloseDevice();
    return err;
  }

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void WmxRobotOptionNodeApi::closeDevice()
{
  {
    std::lock_guard<std::mutex> lock(robotMutex_);
    robotLoaded_ = false;
  }

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorToString(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "Device closed");
  }
}

bool WmxRobotOptionNodeApi::hasRobot() const
{
  std::lock_guard<std::mutex> lock(robotMutex_);
  return robotLoaded_;
}

int WmxRobotOptionNodeApi::robotId() const
{
  std::lock_guard<std::mutex> lock(robotMutex_);
  return robotMotionParam_.robotParam.robotId;
}

int WmxRobotOptionNodeApi::numJoints() const
{
  std::lock_guard<std::mutex> lock(robotMutex_);
  return robotMotionParam_.robotParam.numJoints;
}

int WmxRobotOptionNodeApi::checkRobot(int32_t robotId, std::string & message) const
{
  if (!robotLoaded_) {
    message = "No robot parameter is loaded. Call wmx/robot/set_robot_param first.";
    return ErrorCode::IDNotDefined;
  }

  if (robotId != robotMotionParam_.robotParam.robotId) {
    message = "Unknown " + robotIdText(robotId) + ". This node serves " +
      robotIdText(robotMotionParam_.robotParam.robotId) + ".";
    return ErrorCode::ArgumentOutOfRange;
  }

  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::fillJoints(
  const std::vector<double> & source,
  double (& target)[wmx3Api::kinematics::constants::MAX_NUMBER_OF_JOINT],
  std::string & message) const
{
  const int joints = robotMotionParam_.robotParam.numJoints;

  if (static_cast<int>(source.size()) != joints) {
    message = "Expected " + std::to_string(joints) + " joint values, got " +
      std::to_string(source.size()) + ".";
    return ErrorCode::ArgumentOutOfRange;
  }

  for (int i = 0; i < maxJoints; ++i) {
    target[i] = i < joints ? source[i] : 0.0;
  }

  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::setRobotParam(
  const std::string & paramFile, int32_t robotId, int32_t & outRobotId, int32_t & numJoints,
  std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  if (paramFile.empty()) {
    message = "param_file is empty.";
    return ErrorCode::ArgumentIsNull;
  }

  const bool isUrdf = endsWith(paramFile, ".urdf");

  int err = isUrdf ?
    robot_.mRobotConfig.ImportParamURDF(paramFile.c_str(), robotId, robotMotionParam_) :
    robot_.mRobotConfig.ImportParamXML(paramFile.c_str(), robotMotionParam_);
  if (err != ErrorCode::None) {
    message = failureText(
      isUrdf ? "ImportParamURDF" : "ImportParamXML", "file=" + paramFile, err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = robot_.mKinematics.SetRobotParam(robotMotionParam_.robotParam);
  if (err != ErrorCode::None) {
    message = failureText("SetRobotParam", "file=" + paramFile, err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  robotLoaded_ = true;
  outRobotId = robotMotionParam_.robotParam.robotId;
  numJoints = robotMotionParam_.robotParam.numJoints;

  message = "SetRobotParam done. " + robotIdText(outRobotId) + " joints=" +
    std::to_string(numJoints) + " file=" + paramFile;
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::releaseRobot(int32_t robotId, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  err = robot_.mKinematics.ReleaseRobot(robotId);
  if (err != ErrorCode::None) {
    message = failureText("ReleaseRobot", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  robotLoaded_ = false;
  message = "ReleaseRobot done. " + robotIdText(robotId);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::updateRobotStatus(wmx3Api::RobotStatus & status, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  if (!robotLoaded_) {
    message = "No robot parameter is loaded.";
    return ErrorCode::IDNotDefined;
  }

  int errorCode[wmx3Api::RobotStatus::ErrorBit::ERROR_BIT_SIZE] = {};
  const int errorBit = robot_.UpdateRobotStatus(
    robotMotionParam_.robotParam.robotId, status, errorCode);

  // The motion error bit reports robot state, not a read failure: it belongs in the status.
  const int readBits = errorBit & ~(1 << wmx3Api::RobotStatus::ErrorBit::ERROR_BIT_MOTION_ERROR);
  if (readBits == 0) {
    return ErrorCode::None;
  }

  int err = ErrorCode::UnknownError;
  for (int i = 0; i < wmx3Api::RobotStatus::ErrorBit::ERROR_BIT_SIZE; ++i) {
    if (((readBits >> i) & 0x01) && errorCode[i] != ErrorCode::None) {
      err = errorCode[i];
      message = failureText("UpdateRobotStatus", "bit=" + std::to_string(i), err);
      break;
    }
  }

  return err;
}

int WmxRobotOptionNodeApi::startPtp(
  int32_t robotId, int32_t mode, bool useCartesian, const std::vector<double> & targetJoint,
  const CartesianPose & targetPose, char s, char e, char r, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  const wmx3Api::kinematics::RobotParam & param = robotMotionParam_.robotParam;
  const wmx3Api::RobotMotionProfile & profile = robotMotionParam_.profile;
  double joints[maxJoints] = {};

  switch (mode) {
    case PTPParam::PTPMode::Pos:
      if (useCartesian) {
        err = robot_.mKinematics.StartPTPPos(
          wmx3Api::PtpMotionParam::PtpPosParam(param, profile), targetPose, s, e, r);
      } else {
        err = fillJoints(targetJoint, joints, message);
        if (err != ErrorCode::None) {
          return err;
        }
        err = robot_.mKinematics.StartPTPPos(
          wmx3Api::PtpMotionParam::PtpPosParam(param, profile, joints));
      }
      break;

    case PTPParam::PTPMode::Mov:
      if (useCartesian) {
        err = robot_.mKinematics.StartPTPPos(
          wmx3Api::PtpMotionParam::PtpMovParam(param, profile), targetPose, s, e, r);
      } else {
        err = fillJoints(targetJoint, joints, message);
        if (err != ErrorCode::None) {
          return err;
        }
        err = robot_.mKinematics.StartPTPPos(
          wmx3Api::PtpMotionParam::PtpMovParam(param, profile, joints));
      }
      break;

    case PTPParam::PTPMode::Vel:
      err = fillJoints(targetJoint, joints, message);
      if (err != ErrorCode::None) {
        return err;
      }
      err = robot_.mKinematics.StartPTPPos(
        wmx3Api::PtpMotionParam::PtpJogParam(param, profile, joints));
      break;

    default:
      message = "Unknown mode " + std::to_string(mode) + ". Use 0 pos, 1 mov or 2 vel.";
      return ErrorCode::ArgumentOutOfRange;
  }

  if (err != ErrorCode::None) {
    message = failureText(
      "StartPTPPos", robotIdText(robotId) + " mode=" + std::to_string(mode), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "StartPTPPos done. " + robotIdText(robotId) + " mode=" + std::to_string(mode);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::startMotion(
  int32_t robotId, int32_t trajectoryType, bool isToolCoordinate, const CartesianPose & targetPose,
  const std::vector<CartesianPose> & throughPose, double arcAngle, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  const wmx3Api::RobotMotionProfile & profile = robotMotionParam_.profile;

  wmx3Api::coordinate::PoseArray poseArray;
  if (!throughPose.empty()) {
    poseArray.allocate(static_cast<int>(throughPose.size()));
    for (size_t i = 0; i < throughPose.size(); ++i) {
      poseArray.poseArray[i].point = throughPose[i].point;
      poseArray.poseArray[i].rotation = throughPose[i].rotation;
    }
  }

  switch (trajectoryType) {
    case 0:
      err = robot_.mKinematics.SetMotion(
        robotId,
        wmx3Api::TrajectoryMotionParam::TrajectoryLineMotionParam(
          profile, targetPose, isToolCoordinate));
      break;

    case 1:
      if (poseArray.numPoints != 1) {
        message = "An arc needs exactly one through_pose, got " +
          std::to_string(throughPose.size()) + ".";
        poseArray.destroy();
        return ErrorCode::ArgumentOutOfRange;
      }
      err = arcAngle == 0.0 ?
        robot_.mKinematics.SetMotion(
        robotId,
        wmx3Api::TrajectoryMotionParam::TrajectoryArcMotionParam(profile, targetPose),
        poseArray) :
        robot_.mKinematics.SetMotion(
        robotId,
        wmx3Api::TrajectoryMotionParam::TrajectoryArcMotionParam(profile, targetPose, arcAngle),
        poseArray);
      break;

    case 2:
    case 3:
      if (poseArray.numPoints < 2) {
        message = "A spline needs at least two through_pose points, got " +
          std::to_string(throughPose.size()) + ".";
        poseArray.destroy();
        return ErrorCode::ArgumentOutOfRange;
      }
      err = trajectoryType == 2 ?
        robot_.mKinematics.SetMotion(
        robotId,
        wmx3Api::TrajectoryMotionParam::TrajectoryCSplineMotionParam(profile), poseArray) :
        robot_.mKinematics.SetMotion(
        robotId,
        wmx3Api::TrajectoryMotionParam::TrajectoryBSplineMotionParam(profile), poseArray);
      break;

    default:
      message = "Unknown trajectory_type " + std::to_string(trajectoryType) +
        ". Use 0 line, 1 arc, 2 c-spline or 3 b-spline.";
      poseArray.destroy();
      return ErrorCode::ArgumentOutOfRange;
  }

  poseArray.destroy();

  if (err != ErrorCode::None) {
    message = failureText(
      "SetMotion", robotIdText(robotId) + " trajectory_type=" + std::to_string(trajectoryType),
      err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = robot_.mKinematics.StartMotion(robotId);
  if (err != ErrorCode::None) {
    message = failureText("StartMotion", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    robot_.mKinematics.ClearMotion(robotId);
    return err;
  }

  message = "StartMotion done. " + robotIdText(robotId) + " trajectory_type=" +
    std::to_string(trajectoryType);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::stopMotion(int32_t robotId, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  err = robot_.mKinematics.StopMotion(robotId);
  if (err != ErrorCode::None) {
    message = failureText("StopMotion", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "StopMotion done. " + robotIdText(robotId);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::pauseMotion(int32_t robotId, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  err = robot_.mKinematics.PauseMotion(robotId);
  if (err != ErrorCode::None) {
    message = failureText("PauseMotion", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "PauseMotion done. " + robotIdText(robotId);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::resumeMotion(int32_t robotId, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  err = robot_.mKinematics.ResumeMotion(robotId);
  if (err != ErrorCode::None) {
    message = failureText("ResumeMotion", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "ResumeMotion done. " + robotIdText(robotId);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::clearMotionError(int32_t robotId, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  err = robot_.mKinematics.ClearMotionError(robotId);
  if (err != ErrorCode::None) {
    message = failureText("ClearMotionError", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "ClearMotionError done. " + robotIdText(robotId);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::eStop(int32_t robotId, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  err = robot_.mKinematics.EStop(robotId);
  if (err != ErrorCode::None) {
    message = failureText("EStop", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "EStop done. " + robotIdText(robotId);
  RCLCPP_WARN(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::releaseEStop(int32_t robotId, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  err = robot_.mKinematics.ReleaseEStop(robotId);
  if (err != ErrorCode::None) {
    message = failureText("ReleaseEStop", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "ReleaseEStop done. " + robotIdText(robotId);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::overrideVelocity(
  int32_t robotId, double velRatio, double accRatio, double decRatio, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  err = robot_.mKinematics.OverrideVelocityByRatio(robotId, velRatio, accRatio, decRatio);
  if (err != ErrorCode::None) {
    message = failureText("OverrideVelocityByRatio", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "OverrideVelocityByRatio done. " + robotIdText(robotId) + " vel=" +
    std::to_string(velRatio);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::setToolCoordinate(
  int32_t robotId, const CartesianPose & pose, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  err = robot_.mKinematics.SetToolCoordinate(robotId, pose);
  if (err != ErrorCode::None) {
    message = failureText("SetToolCoordinate", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "SetToolCoordinate done. " + robotIdText(robotId);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::getToolCoordinate(
  int32_t robotId, CartesianPose & pose, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  err = robot_.mKinematics.GetToolCoordinate(robotId, pose);
  if (err != ErrorCode::None) {
    message = failureText("GetToolCoordinate", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "GetToolCoordinate done. " + robotIdText(robotId);
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::setWorkCoordinate(
  int32_t robotId, const CartesianPose & pose, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  err = robot_.mKinematics.SetWorkCoordinate(robotId, pose);
  if (err != ErrorCode::None) {
    message = failureText("SetWorkCoordinate", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "SetWorkCoordinate done. " + robotIdText(robotId);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::getWorkCoordinate(
  int32_t robotId, CartesianPose & pose, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  err = robot_.mKinematics.GetWorkCoordinate(robotId, pose);
  if (err != ErrorCode::None) {
    message = failureText("GetWorkCoordinate", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "GetWorkCoordinate done. " + robotIdText(robotId);
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::calcForwardKinematics(
  int32_t robotId, const std::vector<double> & jointPosition, CartesianPose & toolPose,
  std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  double joints[maxJoints] = {};
  err = fillJoints(jointPosition, joints, message);
  if (err != ErrorCode::None) {
    return err;
  }

  err = robot_.mKinematics.CalcForwardKinematics(robotId, joints, toolPose);
  if (err != ErrorCode::None) {
    message = failureText("CalcForwardKinematics", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "CalcForwardKinematics done. " + robotIdText(robotId);
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::calcInverseKinematics(
  int32_t robotId, const CartesianPose & toolPose, char s, char e, char r,
  std::vector<double> & jointPosition, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  double joints[maxJoints] = {};
  err = robot_.mKinematics.CalcInverseKinematics(robotId, toolPose, joints, maxJoints, s, e, r);
  if (err != ErrorCode::None) {
    message = failureText("CalcInverseKinematics", robotIdText(robotId), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  jointPosition.assign(joints, joints + robotMotionParam_.robotParam.numJoints);
  message = "CalcInverseKinematics done. " + robotIdText(robotId);
  return ErrorCode::None;
}

WmxRobotOptionNode::WmxRobotOptionNode()
: LifecycleNode("wmx_robot_option_node")
{
  api_ = std::make_unique<WmxRobotOptionNodeApi>(this->get_logger());

  robotParamFile_ = this->declare_parameter("robot_param_file", std::string());

  rate_ = this->declare_parameter("robot_status_rate", 10);
  if (rate_ <= 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "robot_status_rate must be > 0, got %d. Falling back to %d Hz.", rate_, 10);
    rate_ = 10;
  }

  RCLCPP_INFO(
    this->get_logger(), "wmx_robot_option_node is unconfigured, waiting for configure...");
}

WmxRobotOptionNode::~WmxRobotOptionNode()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "wmx_robot_option_node stopped");
}

WmxRobotOptionNode::CallbackReturn WmxRobotOptionNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring wmx_robot_option_node...");

  std::string message;
  if (api_->createDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  if (robotParamFile_.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "robot_param_file is empty; no robot is registered. "
      "Call wmx/robot/set_robot_param before commanding motion.");
  } else {
    int32_t robotId = 0;
    int32_t numJoints = 0;
    if (api_->setRobotParam(robotParamFile_, kUrdfRobotId, robotId, numJoints, message) !=
      ErrorCode::None)
    {
      api_->closeDevice();
      return CallbackReturn::FAILURE;
    }
  }

  RCLCPP_INFO(this->get_logger(), "wmx_robot_option_node is configured (%d Hz)", rate_);
  return CallbackReturn::SUCCESS;
}

WmxRobotOptionNode::CallbackReturn WmxRobotOptionNode::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  setRobotParamService_ = this->create_service<wmx_r2_message::srv::RobotSetRobotParam>(
    "wmx/robot/set_robot_param",
    std::bind(&WmxRobotOptionNode::setRobotParamCallback, this, _1, _2));

  releaseRobotService_ = this->create_service<wmx_r2_message::srv::RobotId>(
    "wmx/robot/release_robot",
    std::bind(&WmxRobotOptionNode::releaseRobotCallback, this, _1, _2));

  startPtpService_ = this->create_service<wmx_r2_message::srv::RobotStartPtp>(
    "wmx/robot/start_ptp",
    std::bind(&WmxRobotOptionNode::startPtpCallback, this, _1, _2));

  startMotionService_ = this->create_service<wmx_r2_message::srv::RobotStartMotion>(
    "wmx/robot/start_motion",
    std::bind(&WmxRobotOptionNode::startMotionCallback, this, _1, _2));

  stopMotionService_ = this->create_service<wmx_r2_message::srv::RobotId>(
    "wmx/robot/stop_motion",
    std::bind(&WmxRobotOptionNode::stopMotionCallback, this, _1, _2));

  pauseMotionService_ = this->create_service<wmx_r2_message::srv::RobotId>(
    "wmx/robot/pause_motion",
    std::bind(&WmxRobotOptionNode::pauseMotionCallback, this, _1, _2));

  resumeMotionService_ = this->create_service<wmx_r2_message::srv::RobotId>(
    "wmx/robot/resume_motion",
    std::bind(&WmxRobotOptionNode::resumeMotionCallback, this, _1, _2));

  clearMotionErrorService_ = this->create_service<wmx_r2_message::srv::RobotId>(
    "wmx/robot/clear_motion_error",
    std::bind(&WmxRobotOptionNode::clearMotionErrorCallback, this, _1, _2));

  eStopService_ = this->create_service<wmx_r2_message::srv::RobotId>(
    "wmx/robot/e_stop",
    std::bind(&WmxRobotOptionNode::eStopCallback, this, _1, _2));

  releaseEStopService_ = this->create_service<wmx_r2_message::srv::RobotId>(
    "wmx/robot/release_e_stop",
    std::bind(&WmxRobotOptionNode::releaseEStopCallback, this, _1, _2));

  overrideVelocityService_ = this->create_service<wmx_r2_message::srv::RobotOverrideVelocity>(
    "wmx/robot/override_velocity",
    std::bind(&WmxRobotOptionNode::overrideVelocityCallback, this, _1, _2));

  setToolCoordinateService_ = this->create_service<wmx_r2_message::srv::RobotSetCoordinate>(
    "wmx/robot/set_tool_coordinate",
    std::bind(&WmxRobotOptionNode::setToolCoordinateCallback, this, _1, _2));

  getToolCoordinateService_ = this->create_service<wmx_r2_message::srv::RobotGetCoordinate>(
    "wmx/robot/get_tool_coordinate",
    std::bind(&WmxRobotOptionNode::getToolCoordinateCallback, this, _1, _2));

  setWorkCoordinateService_ = this->create_service<wmx_r2_message::srv::RobotSetCoordinate>(
    "wmx/robot/set_work_coordinate",
    std::bind(&WmxRobotOptionNode::setWorkCoordinateCallback, this, _1, _2));

  getWorkCoordinateService_ = this->create_service<wmx_r2_message::srv::RobotGetCoordinate>(
    "wmx/robot/get_work_coordinate",
    std::bind(&WmxRobotOptionNode::getWorkCoordinateCallback, this, _1, _2));

  calcForwardKinematicsService_ =
    this->create_service<wmx_r2_message::srv::RobotCalcForwardKinematics>(
    "wmx/robot/calc_forward_kinematics",
    std::bind(&WmxRobotOptionNode::calcForwardKinematicsCallback, this, _1, _2));

  calcInverseKinematicsService_ =
    this->create_service<wmx_r2_message::srv::RobotCalcInverseKinematics>(
    "wmx/robot/calc_inverse_kinematics",
    std::bind(&WmxRobotOptionNode::calcInverseKinematicsCallback, this, _1, _2));

  robotStatusPub_ = this->create_publisher<wmx_r2_message::msg::RobotStatus>(
    "wmx/robot/status", 1);

  LifecycleNode::on_activate(previous_state);

  robotStatusTimer_ = this->create_wall_timer(
    periodFromRate(rate_),
    std::bind(&WmxRobotOptionNode::robotStatusStep, this));

  RCLCPP_INFO(this->get_logger(), "wmx_robot_option_node is active");
  return CallbackReturn::SUCCESS;
}

WmxRobotOptionNode::CallbackReturn WmxRobotOptionNode::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_deactivate(previous_state);

  robotStatusTimer_.reset();
  robotStatusPub_.reset();

  setRobotParamService_.reset();
  releaseRobotService_.reset();
  startPtpService_.reset();
  startMotionService_.reset();
  stopMotionService_.reset();
  pauseMotionService_.reset();
  resumeMotionService_.reset();
  clearMotionErrorService_.reset();
  eStopService_.reset();
  releaseEStopService_.reset();
  overrideVelocityService_.reset();
  setToolCoordinateService_.reset();
  getToolCoordinateService_.reset();
  setWorkCoordinateService_.reset();
  getWorkCoordinateService_.reset();
  calcForwardKinematicsService_.reset();
  calcInverseKinematicsService_.reset();

  RCLCPP_INFO(this->get_logger(), "wmx_robot_option_node is inactive");
  return CallbackReturn::SUCCESS;
}

WmxRobotOptionNode::CallbackReturn WmxRobotOptionNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  api_->closeDevice();

  RCLCPP_INFO(this->get_logger(), "wmx_robot_option_node is cleaned up");
  return CallbackReturn::SUCCESS;
}

WmxRobotOptionNode::CallbackReturn WmxRobotOptionNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(previous_state);
  }

  return on_cleanup(previous_state);
}

void WmxRobotOptionNode::robotStatusStep()
{
  if (!api_->hasRobot()) {
    return;
  }

  wmx3Api::RobotStatus status;
  std::string message;
  if (api_->updateRobotStatus(status, message) != ErrorCode::None) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "%s", message.c_str());
    return;
  }

  const int joints = api_->numJoints();

  robotStatusMsg_.joint_pos_cmd.clear();
  robotStatusMsg_.joint_pos_fb.clear();
  robotStatusMsg_.joint_velocity_cmd.clear();
  robotStatusMsg_.joint_velocity_fb.clear();
  robotStatusMsg_.joint_torque_fb.clear();

  robotStatusMsg_.header.stamp = this->now();
  robotStatusMsg_.header.frame_id = "base_link";

  robotStatusMsg_.robot_id = status.stateCommand.robotId;
  robotStatusMsg_.motion_state = static_cast<int32_t>(status.motionState);
  robotStatusMsg_.motion_error_code = static_cast<int32_t>(status.motionErrorInfo.errorCode);
  robotStatusMsg_.motion_error_joint = status.motionErrorInfo.errorJoint;

  robotStatusMsg_.tool_pose_cmd = toPoseMsg(status.stateCommand.toolPose[0]);
  robotStatusMsg_.tool_pose_fb = toPoseMsg(status.stateFeedback.toolPose[0]);

  for (int i = 0; i < joints; ++i) {
    robotStatusMsg_.joint_pos_cmd.push_back(status.stateCommand.jointPosition[i]);
    robotStatusMsg_.joint_pos_fb.push_back(status.stateFeedback.jointPosition[i]);
    robotStatusMsg_.joint_velocity_cmd.push_back(status.stateCommand.jointVelocity[i]);
    robotStatusMsg_.joint_velocity_fb.push_back(status.stateFeedback.jointVelocity[i]);
    robotStatusMsg_.joint_torque_fb.push_back(status.stateFeedback.jointTorque[i]);
  }

  robotStatusPub_->publish(robotStatusMsg_);
}

void WmxRobotOptionNode::setRobotParamCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotSetRobotParam::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotSetRobotParam::Response> response)
{
  std::string message;
  response->success = api_->setRobotParam(
    request->param_file, request->robot_id, response->robot_id, response->num_joints,
    message) == ErrorCode::None;
  response->message = message;
}

void WmxRobotOptionNode::releaseRobotCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotId::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotId::Response> response)
{
  std::string message;
  response->success = api_->releaseRobot(request->robot_id, message) == ErrorCode::None;
  response->message = message;
}

void WmxRobotOptionNode::startPtpCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotStartPtp::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotStartPtp::Response> response)
{
  std::string message;
  response->success = api_->startPtp(
    request->robot_id, request->mode, request->use_cartesian, request->target_joint,
    toCartesianPose(request->target_pose), request->s, request->e, request->r,
    message) == ErrorCode::None;
  response->message = message;
}

void WmxRobotOptionNode::startMotionCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotStartMotion::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotStartMotion::Response> response)
{
  std::string message;
  response->success = api_->startMotion(
    request->robot_id, request->trajectory_type, request->is_tool_coordinate,
    toCartesianPose(request->target_pose), toCartesianPoses(request->through_pose),
    request->arc_angle, message) == ErrorCode::None;
  response->message = message;
}

void WmxRobotOptionNode::stopMotionCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotId::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotId::Response> response)
{
  std::string message;
  response->success = api_->stopMotion(request->robot_id, message) == ErrorCode::None;
  response->message = message;
}

void WmxRobotOptionNode::pauseMotionCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotId::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotId::Response> response)
{
  std::string message;
  response->success = api_->pauseMotion(request->robot_id, message) == ErrorCode::None;
  response->message = message;
}

void WmxRobotOptionNode::resumeMotionCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotId::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotId::Response> response)
{
  std::string message;
  response->success = api_->resumeMotion(request->robot_id, message) == ErrorCode::None;
  response->message = message;
}

void WmxRobotOptionNode::clearMotionErrorCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotId::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotId::Response> response)
{
  std::string message;
  response->success = api_->clearMotionError(request->robot_id, message) == ErrorCode::None;
  response->message = message;
}

void WmxRobotOptionNode::eStopCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotId::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotId::Response> response)
{
  std::string message;
  response->success = api_->eStop(request->robot_id, message) == ErrorCode::None;
  response->message = message;
}

void WmxRobotOptionNode::releaseEStopCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotId::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotId::Response> response)
{
  std::string message;
  response->success = api_->releaseEStop(request->robot_id, message) == ErrorCode::None;
  response->message = message;
}

void WmxRobotOptionNode::overrideVelocityCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotOverrideVelocity::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotOverrideVelocity::Response> response)
{
  std::string message;
  response->success = api_->overrideVelocity(
    request->robot_id, request->vel_ratio, request->acc_ratio, request->dec_ratio,
    message) == ErrorCode::None;
  response->message = message;
}

void WmxRobotOptionNode::setToolCoordinateCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotSetCoordinate::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotSetCoordinate::Response> response)
{
  std::string message;
  response->success = api_->setToolCoordinate(
    request->robot_id, toCartesianPose(request->pose), message) == ErrorCode::None;
  response->message = message;
}

void WmxRobotOptionNode::getToolCoordinateCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotGetCoordinate::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotGetCoordinate::Response> response)
{
  CartesianPose pose;
  std::string message;
  response->success =
    api_->getToolCoordinate(request->robot_id, pose, message) == ErrorCode::None;
  response->pose = toPoseMsg(pose);
  response->message = message;
}

void WmxRobotOptionNode::setWorkCoordinateCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotSetCoordinate::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotSetCoordinate::Response> response)
{
  std::string message;
  response->success = api_->setWorkCoordinate(
    request->robot_id, toCartesianPose(request->pose), message) == ErrorCode::None;
  response->message = message;
}

void WmxRobotOptionNode::getWorkCoordinateCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotGetCoordinate::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotGetCoordinate::Response> response)
{
  CartesianPose pose;
  std::string message;
  response->success =
    api_->getWorkCoordinate(request->robot_id, pose, message) == ErrorCode::None;
  response->pose = toPoseMsg(pose);
  response->message = message;
}

void WmxRobotOptionNode::calcForwardKinematicsCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotCalcForwardKinematics::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotCalcForwardKinematics::Response> response)
{
  CartesianPose toolPose;
  std::string message;
  response->success = api_->calcForwardKinematics(
    request->robot_id, request->joint_position, toolPose, message) == ErrorCode::None;
  response->tool_pose = toPoseMsg(toolPose);
  response->message = message;
}

void WmxRobotOptionNode::calcInverseKinematicsCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotCalcInverseKinematics::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotCalcInverseKinematics::Response> response)
{
  std::string message;
  response->success = api_->calcInverseKinematics(
    request->robot_id, toCartesianPose(request->tool_pose), request->s, request->e, request->r,
    response->joint_position, message) == ErrorCode::None;
  response->message = message;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxRobotOptionNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
