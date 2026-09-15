// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_robot_option_node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>

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

constexpr int32_t kUrdfRobotId = 0;

constexpr int32_t kFrameBase = 0;
constexpr int32_t kFrameTool = 1;
constexpr int32_t kTargetJoint = 0;
constexpr int32_t kTargetPose = 1;
constexpr int32_t kPathPtp = 0;
constexpr int32_t kPathLine = 1;

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

using RotationMatrix = std::array<std::array<double, 3>, 3>;

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

RotationMatrix rotationMatrixOf(const CartesianPose & pose)
{
  const double su = std::sin(pose.rotation.u * kDegToRad);
  const double cu = std::cos(pose.rotation.u * kDegToRad);
  const double sv = std::sin(pose.rotation.v * kDegToRad);
  const double cv = std::cos(pose.rotation.v * kDegToRad);
  const double sw = std::sin(pose.rotation.w * kDegToRad);
  const double cw = std::cos(pose.rotation.w * kDegToRad);

  return RotationMatrix{{
    {{cw * cv, cw * sv * su - sw * cu, cw * sv * cu + sw * su}},
    {{sw * cv, sw * sv * su + cw * cu, sw * sv * cu - cw * su}},
    {{-sv, cv * su, cv * cu}}
  }};
}

RotationMatrix multiplyRotations(
  const RotationMatrix & left, const RotationMatrix & right)
{
  RotationMatrix result{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result[i][j] = left[i][0] * right[0][j] +
        left[i][1] * right[1][j] +
        left[i][2] * right[2][j];
    }
  }
  return result;
}

void setEulerFromRotationMatrix(CartesianPose & pose, const RotationMatrix & rotation)
{
  const double cv = std::hypot(rotation[0][0], rotation[1][0]);

  if (cv < 1e-9) {
    pose.rotation.u = 0.0;
    pose.rotation.v = std::atan2(-rotation[2][0], cv) * kRadToDeg;
    pose.rotation.w = std::atan2(-rotation[0][1], rotation[1][1]) * kRadToDeg;
    return;
  }

  pose.rotation.u = std::atan2(rotation[2][1], rotation[2][2]) * kRadToDeg;
  pose.rotation.v = std::atan2(-rotation[2][0], cv) * kRadToDeg;
  pose.rotation.w = std::atan2(rotation[1][0], rotation[0][0]) * kRadToDeg;
}

CartesianPose applyWorkFrameDisplacement(
  const CartesianPose & current, const CartesianPose & displacement)
{
  CartesianPose result = current;
  result.point.x += displacement.point.x;
  result.point.y += displacement.point.y;
  result.point.z += displacement.point.z;

  const bool hasRotation = displacement.rotation.u != 0.0 ||
    displacement.rotation.v != 0.0 ||
    displacement.rotation.w != 0.0;
  if (hasRotation) {
    setEulerFromRotationMatrix(result, multiplyRotations(
        rotationMatrixOf(displacement), rotationMatrixOf(current)));
  }

  return result;
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

}  // namespace

WmxRobotOptionNodeApi::WmxRobotOptionNodeApi(const rclcpp::Logger & logger)
: logger_(logger), robot_(&wmx3Lib_)
{
}

WmxRobotOptionNodeApi::~WmxRobotOptionNodeApi()
{
  releaseRobotIfLoaded();
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

void WmxRobotOptionNodeApi::releaseRobotIfLoaded()
{
  if (!hasRobot()) {
    return;
  }

  std::string message;
  releaseRobot(robotId(), message);
}

int WmxRobotOptionNodeApi::updateRobotStatus(wmx3Api::RobotStatus & status, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);
  return updateRobotStatusLocked(status, message);
}

int WmxRobotOptionNodeApi::updateRobotStatusLocked(
  wmx3Api::RobotStatus & status, std::string & message)
{
  if (!robotLoaded_) {
    message = "No robot parameter is loaded.";
    return ErrorCode::IDNotDefined;
  }

  int errorCode[wmx3Api::RobotStatus::ErrorBit::ERROR_BIT_SIZE] = {};
  const int errorBit = robot_.UpdateRobotStatus(
    robotMotionParam_.robotParam.robotId, status, errorCode);

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

int WmxRobotOptionNodeApi::commandedToolPose(CartesianPose & pose, std::string & message)
{
  wmx3Api::RobotStatus status;

  const int err = updateRobotStatusLocked(status, message);
  if (err != ErrorCode::None) {
    return err;
  }

  pose.point = status.stateCommand.toolPose[0].point;
  pose.rotation = status.stateCommand.toolPose[0].rotation;
  return ErrorCode::None;
}

int WmxRobotOptionNodeApi::startMotion(
  int32_t robotId, int32_t mode, int32_t frame, int32_t targetType, int32_t path,
  const std::vector<double> & targetJoint, const CartesianPose & targetPose,
  char s, char e, char r, std::string & message)
{
  std::lock_guard<std::mutex> lock(robotMutex_);

  int err = checkRobot(robotId, message);
  if (err != ErrorCode::None) {
    return err;
  }

  if (mode != PTPParam::PTPMode::Pos && mode != PTPParam::PTPMode::Mov) {
    message = "Unknown mode " + std::to_string(mode) + ". Use 0 pos or 1 mov.";
    return ErrorCode::ArgumentOutOfRange;
  }

  if (targetType != kTargetJoint && targetType != kTargetPose) {
    message = "Unknown target_type " + std::to_string(targetType) +
      ". Use 0 joint or 1 pose.";
    return ErrorCode::ArgumentOutOfRange;
  }

  if (frame != kFrameBase && frame != kFrameTool) {
    message = "Unknown frame " + std::to_string(frame) + ". Use 0 base/work or 1 tool.";
    return ErrorCode::ArgumentOutOfRange;
  }

  const wmx3Api::kinematics::RobotParam & param = robotMotionParam_.robotParam;
  const wmx3Api::RobotMotionProfile & profile = robotMotionParam_.profile;
  const bool relative = mode == PTPParam::PTPMode::Mov;

  switch (path) {
    case kPathLine: {
        if (targetType != kTargetPose) {
          message = "A line needs target_type 1 pose; joint targets have no "
            "straight line to follow.";
          return ErrorCode::ArgumentOutOfRange;
        }

        const bool isToolFrameDisplacement = relative && frame == kFrameTool;
        const bool isWorkFrameDisplacement = relative && frame == kFrameBase;

        CartesianPose commandedPose;
        if (isWorkFrameDisplacement) {
          err = commandedToolPose(commandedPose, message);
          if (err != ErrorCode::None) {
            RCLCPP_ERROR(logger_, "%s", message.c_str());
            return err;
          }
        }

        const CartesianPose destination = isWorkFrameDisplacement ?
          applyWorkFrameDisplacement(commandedPose, targetPose) : targetPose;

        err = robot_.mKinematics.SetMotion(
          robotId,
          wmx3Api::TrajectoryMotionParam::TrajectoryLineMotionParam(
            profile, destination, isToolFrameDisplacement));
        if (err != ErrorCode::None) {
          message = failureText("SetMotion", robotIdText(robotId) + " line", err);
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
        break;
      }

    case kPathPtp: {
        double joints[maxJoints] = {};
        const char * wmxCall = "StartPTPPos";

        if (targetType == kTargetJoint) {
          err = fillJoints(targetJoint, joints, message);
          if (err != ErrorCode::None) {
            return err;
          }
          err = relative ?
            robot_.mKinematics.StartPTPPos(
            wmx3Api::PtpMotionParam::PtpMovParam(param, profile, joints)) :
            robot_.mKinematics.StartPTPPos(
            wmx3Api::PtpMotionParam::PtpPosParam(param, profile, joints));
        } else if (relative && frame == kFrameTool) {
          // The only tool frame PTP entry point. StartPTPPos would read the
          // same displacement in the work frame.
          wmxCall = "StartToolPTPMov";
          err = robot_.mKinematics.StartToolPTPMov(
            wmx3Api::PtpMotionParam::PtpMovParam(param, profile), targetPose, s, e, r);
        } else {
          err = relative ?
            robot_.mKinematics.StartPTPPos(
            wmx3Api::PtpMotionParam::PtpMovParam(param, profile), targetPose, s, e, r) :
            robot_.mKinematics.StartPTPPos(
            wmx3Api::PtpMotionParam::PtpPosParam(param, profile), targetPose, s, e, r);
        }

        if (err != ErrorCode::None) {
          message = failureText(
            wmxCall, robotIdText(robotId) + " mode=" + std::to_string(mode), err);
          RCLCPP_ERROR(logger_, "%s", message.c_str());
          return err;
        }
        break;
      }

    default:
      message = "Unknown path " + std::to_string(path) + ". Use 0 ptp or 1 line.";
      return ErrorCode::ArgumentOutOfRange;
  }

  message = std::string(path == kPathLine ? "Line" : "PTP") + " motion started. " +
    robotIdText(robotId) + " mode=" + std::to_string(mode) +
    " target_type=" + std::to_string(targetType);
  if (relative && targetType == kTargetPose) {
    message += std::string(" frame=") + (frame == kFrameTool ? "tool" : "base");
  }
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

int WmxRobotOptionNodeApi::overrideVelocityByRatio(
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

  startMotionService_ = this->create_service<wmx_r2_message::srv::RobotStartMotion>(
    "wmx/robot/start_motion",
    std::bind(&WmxRobotOptionNode::startMotionCallback, this, _1, _2));

  stopMotionService_ = this->create_service<wmx_r2_message::srv::RobotId>(
    "wmx/robot/stop_motion",
    std::bind(&WmxRobotOptionNode::stopMotionCallback, this, _1, _2));

  clearMotionErrorService_ = this->create_service<wmx_r2_message::srv::RobotId>(
    "wmx/robot/clear_motion_error",
    std::bind(&WmxRobotOptionNode::clearMotionErrorCallback, this, _1, _2));

  eStopService_ = this->create_service<wmx_r2_message::srv::RobotId>(
    "wmx/robot/e_stop",
    std::bind(&WmxRobotOptionNode::eStopCallback, this, _1, _2));

  releaseEStopService_ = this->create_service<wmx_r2_message::srv::RobotId>(
    "wmx/robot/release_e_stop",
    std::bind(&WmxRobotOptionNode::releaseEStopCallback, this, _1, _2));

  overrideVelocityByRatioService_ =
    this->create_service<wmx_r2_message::srv::RobotOverrideVelocityByRatio>(
    "wmx/robot/override_velocity_by_ratio",
    std::bind(&WmxRobotOptionNode::overrideVelocityByRatioCallback, this, _1, _2));

  setToolCoordinateService_ = this->create_service<wmx_r2_message::srv::RobotSetCoordinate>(
    "wmx/robot/set_tool_coordinate",
    std::bind(&WmxRobotOptionNode::setToolCoordinateCallback, this, _1, _2));

  getToolCoordinateService_ = this->create_service<wmx_r2_message::srv::RobotGetCoordinate>(
    "wmx/robot/get_tool_coordinate",
    std::bind(&WmxRobotOptionNode::getToolCoordinateCallback, this, _1, _2));

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
  startMotionService_.reset();
  stopMotionService_.reset();
  clearMotionErrorService_.reset();
  eStopService_.reset();
  releaseEStopService_.reset();
  overrideVelocityByRatioService_.reset();
  setToolCoordinateService_.reset();
  getToolCoordinateService_.reset();

  RCLCPP_INFO(this->get_logger(), "wmx_robot_option_node is inactive");
  return CallbackReturn::SUCCESS;
}

WmxRobotOptionNode::CallbackReturn WmxRobotOptionNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  api_->releaseRobotIfLoaded();
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

void WmxRobotOptionNode::startMotionCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotStartMotion::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotStartMotion::Response> response)
{
  std::string message;
  response->success = api_->startMotion(
    request->robot_id, request->mode, request->frame, request->target_type, request->path,
    request->target_joint, toCartesianPose(request->target_pose),
    request->s, request->e, request->r, message) == ErrorCode::None;
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

void WmxRobotOptionNode::overrideVelocityByRatioCallback(
  const std::shared_ptr<wmx_r2_message::srv::RobotOverrideVelocityByRatio::Request> request,
  std::shared_ptr<wmx_r2_message::srv::RobotOverrideVelocityByRatio::Response> response)
{
  std::string message;
  response->success = api_->overrideVelocityByRatio(
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxRobotOptionNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
