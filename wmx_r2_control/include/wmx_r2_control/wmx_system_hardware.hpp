// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_R2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_
#define WMX_R2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_

#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "IOApi.h"
#include "AdvancedMotionApi.h"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#if defined(__has_include)
#  if __has_include("hardware_interface/types/hardware_component_interface_params.hpp")
#    include "hardware_interface/types/hardware_component_interface_params.hpp"
#    define WMX_HAS_HW_COMPONENT_INTERFACE_PARAMS 1
#  endif
#endif

#include "rclcpp/macros.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace wmx_r2_control
{

class WmxSystemHardwareApi
{
public:
  // Same spline channel and point limit as wmx_r2_package joint_trajectory_controller.
  static constexpr int kSplineChannel = 0;
  static constexpr unsigned int kMaxTrajectoryPoints = 1000;

  struct Config
  {
    std::string sdkPath;
    std::string deviceName;
    double accTimeMilliseconds = 1.0;
    double decTimeMilliseconds = 1.0;
    int maxDeviceRetries = 30;
  };

  struct AxisFeedback
  {
    double actualPos = 0.0;
    double actualVelocity = 0.0;
    bool servoOn = false;
    bool ampAlarm = false;
    double posCmd = 0.0;
    bool inPos = false;
  };

  WmxSystemHardwareApi(const rclcpp::Logger & logger, const Config & config);
  ~WmxSystemHardwareApi();

  int createDevice(std::string & message);
  void closeDevice();

  int importAndSetAll(const std::string & path, std::string & message);

  int getStatus(
    const std::vector<int> & axes, std::vector<AxisFeedback> & feedback,
    bool & communicating, std::string & message);

  int startVel(int axis, double omega, std::string & message);
  int stop(int axis, std::string & message);
  int waitForMotionComplete(
    const std::vector<int> & axes, unsigned int timeoutMilliseconds, std::string & message);
  int setServoOn(int axis, int newStatus, std::string & message);
  int clearAmpAlarm(int axis, std::string & message);
  int startLinearIntplPos(
    const std::vector<int> & axes, const std::vector<double> & targets,
    const std::vector<double> & maxVelocity, const std::vector<double> & maxAcc,
    std::string & message);
  int setOutBit(int addr, int bit, bool on);
  int setOutByte(int addr, int data);

  // CSpline passthrough (mirrors wmx_r2_package JointTrajectoryControllerApi).
  int createSplineBuffer(std::string & message);
  void freeSplineBuffer();
  bool splineBufferCreated() const {return splineBufferCreated_;}
  // positions is row-major: pointCount rows of axes.size() values.
  int startCSplinePos(
    const std::vector<int> & axes, const std::vector<double> & positions,
    const std::vector<double> & timesMs, unsigned int pointCount, std::string & message);
  int stopAxes(const std::vector<int> & axes, std::string & message);

private:
  rclcpp::Logger logger_;
  Config config_;

  unsigned int timeout_ = 10000;
  unsigned int servoOnTimeout_ = 2000;

  mutable std::mutex deviceMutex_;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::CoreMotion cm_;
  wmx3Api::IO io_;
  wmx3Api::AdvancedMotion am_;

  bool splineBufferCreated_ = false;
  wmx3Api::AdvMotion::PointTimeSplineCommand splineCommand_;
  std::vector<wmx3Api::AdvMotion::SplinePoint> splinePoints_;
  std::vector<double> splineTimesMs_;
};

enum class JointMode
{
  Velocity,    // command_interface "velocity" -> WMX CoreMotion StartVel
  Position,    // command_interface "position" -> WMX CoreMotion StartLinearIntplPos
  StateOnly    // no command_interface -> feedback only (motion driven elsewhere)
};

struct WmxJoint
{
  std::string name;
  int axis = -1;
  JointMode mode = JointMode::Velocity;

  double posState = 0.0;
  double velState = 0.0;

  double cmd = 0.0;
  double lastCmd = std::numeric_limits<double>::quiet_NaN();
};

// On/off gripper on one WMX3 digital output bit; commanded through a position interface.
struct WmxGripper
{
  std::string name;
  int ioByte = 0;
  int ioBit = 0;
  double closeThreshold = 0.0;
  bool hasPower = false;
  int powerByte = 0;
  int powerValue = 0;
  // State-only joints (<param name="mimic">) that report this gripper's position/velocity.
  std::vector<std::string> mimicJoints;

  double posState = 0.0;
  double velState = 0.0;
  double cmd = 0.0;
  int lastClosed = -1;  // -1: nothing written yet
};

// Arm command source: the joint position interfaces (StartLinearIntplPos each cycle) or the
// trajectory_passthrough GPIO (one StartCSplinePos per trajectory).
enum class ArmCommandMode
{
  Position,
  Passthrough
};

// Transfer states, ros2 passthrough-trajectory controller format.
namespace passthrough_state
{
constexpr double kIdle = 0.0;
constexpr double kWaitingForPoint = 1.0;
constexpr double kTransferring = 2.0;
constexpr double kTransferDone = 3.0;
constexpr double kInMotion = 4.0;
constexpr double kDone = 5.0;
constexpr double kNewTrajectory = 6.0;
}  // namespace passthrough_state

class WmxSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(WmxSystemHardware)

#if WMX_HAS_HW_COMPONENT_INTERFACE_PARAMS
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
#else
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
#endif

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void stopAllAxes();
  void writePositions();
  void writeGrippers();
  void writePassthrough();
  void finishPassthrough(bool aborted);
  void stopPassthroughAxes();
  void resyncPositionCommands();
  bool isPassthroughInterface(const std::string & name) const;
  bool isArmPositionInterface(const std::string & name) const;

  rclcpp::Logger logger_ = rclcpp::get_logger("WmxSystemHardware");
  rclcpp::Clock clock_{RCL_STEADY_TIME};

  std::unique_ptr<WmxSystemHardwareApi> api_;

  std::vector<WmxJoint> joints_;
  std::vector<WmxGripper> grippers_;
  std::vector<int> axes_;

  std::vector<WmxSystemHardwareApi::AxisFeedback> feedback_;
  bool communicating_ = false;

  std::string wmxParamFile_;
  int maxDeviceRetries_ = 30;
  bool autoServoOn_ = true;
  unsigned int stopTimeout_ = 2000;
  // Scratch buffers for writePositions(), reserved once so write() never allocates.
  std::vector<int> positionAxes_;
  std::vector<size_t> positionJoints_;
  std::vector<double> targets_;
  std::vector<double> maxVelocity_;
  std::vector<double> maxAcc_;
  double accelRatio_ = 0.3;
  double minStep_ = 0.001;

  // trajectory_passthrough GPIO (exported only when declared in the URDF).
  static constexpr const char * kPassthroughGpio = "trajectory_passthrough";
  bool hasPassthroughGpio_ = false;
  ArmCommandMode armMode_ = ArmCommandMode::Position;
  std::vector<double> ptSetpointPositions_;  // one per position joint, joints_ order
  double ptTimeFromStart_ = 0.0;
  double ptTransferState_ = passthrough_state::kIdle;
  double ptAbort_ = 0.0;
  double ptTrajectorySize_ = 0.0;
  // Trajectory received from the controller, reserved in on_activate.
  std::vector<int> ptAxes_;          // spline axes, position joints in joints_ order
  std::vector<size_t> ptJoints_;     // joints_ indices of ptAxes_
  std::vector<double> ptPositions_;  // row-major kMaxTrajectoryPoints x ptAxes_.size()
  std::vector<double> ptTimesMs_;   // absolute ms from trajectory start
  std::vector<std::string> armPositionInterfaces_;  // "<joint>/position" of the arm joints
  int ptClaimed_ = 0;   // claimed trajectory_passthrough/* command interfaces
  int posClaimed_ = 0;  // claimed arm "<joint>/position" command interfaces
  double ptDurationS_ = 0.0;
  size_t ptExpected_ = 0;
  size_t ptReceived_ = 0;
  double ptElapsed_ = 0.0;
  bool ptSawMotion_ = false;
  bool ptStopping_ = false;  // abort: Stop issued, waiting for inPos (deceleration done)
  int ptStopCycles_ = 0;     // read() cycles since Stop; inPos is trusted only after kStopSettleCycles
  static constexpr int kStopSettleCycles = 2;
  hardware_interface::CallbackReturn initImpl();
  std::string getHwParam(const std::string & key, const std::string & def) const;
  bool waitForCommunicating();
  bool seedJointStates();
};

}  // namespace wmx_r2_control

#endif  // WMX_R2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_
