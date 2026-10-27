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

#include "wmx_r2_control/wmx_cyclic_stream.hpp"

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
  int getPosCommand(int axis, double & position, std::string & message);
  /// [local 2026-10-27] Emergency stop Final: the engine sends servo off to every axis at once (no
  /// deceleration). No axis can be commanded until the stop is released.
  int execEStopFinal(std::string & message);
  /// Release a latched emergency stop; no-op when none is active. `released` says whether one was.
  int releaseEStopIfActive(bool & released, std::string & message);

  /// Raw handles for WmxCyclicStream, which polls and appends to the cyclic
  /// buffer from write() itself. Valid from createDevice() until this object is
  /// destroyed; the stream must be closed before closeDevice().
  wmx3Api::WMX3Api * device() {return &wmx3Lib_;}
  wmx3Api::CoreMotion * coreMotion() {return &cm_;}

private:
  rclcpp::Logger logger_;
  Config config_;

  unsigned int timeout_ = 10000;
  unsigned int servoOnTimeout_ = 2000;

  mutable std::mutex deviceMutex_;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::CoreMotion cm_;
};

enum class JointMode
{
  Position,    // command_interface "position" -> WMX CyclicBuffer (AbsolutePos)
  Velocity,    // command_interface "velocity" -> WMX CoreMotion StartVel
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

  /// Position joints with cyclic_subsample "hermite" also take a 'velocity'
  /// command interface (the controller's desired velocity) for the cubic.
  bool velFf = false;
  double velCmd = 0.0;
  bool velClaimed = false;   ///< a controller currently owns <joint>/velocity
};

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

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Track which velocity command interfaces are claimed (cyclic_subsample "hermite").
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  void stopAllAxes();
  void eStopFinal();

  rclcpp::Logger logger_ = rclcpp::get_logger("WmxSystemHardware");
  rclcpp::Clock clock_{RCL_STEADY_TIME};

  std::unique_ptr<WmxSystemHardwareApi> api_;

  // Position-mode streaming: all position joints share one cyclic buffer, since
  // they are commanded by a single multi-axis AddCommand. Declared after api_ so
  // it is destroyed first: closing the buffer needs the device still open.
  WmxCyclicStream stream_;
  std::vector<size_t> positionJointIdx_;   ///< indices into joints_, in axis order
  std::vector<double> qCmdScratch_;        ///< preallocated: write() must not allocate
  std::vector<double> vCmdScratch_;        ///< preallocated, parallel to qCmdScratch_
  std::string streamErr_;                  ///< reused so the happy path stays allocation-free
  bool streamStarted_ = false;

  std::vector<WmxJoint> joints_;
  std::vector<int> axes_;

  std::vector<WmxSystemHardwareApi::AxisFeedback> feedback_;
  bool communicating_ = false;

  std::string wmxParamFile_;
  int maxDeviceRetries_ = 30;
  bool autoServoOn_ = true;
  unsigned int stopTimeout_ = 2000;

  // Cyclic buffer parameters (from <hardware><param> in the ros2_control xacro)
  double cyclicBufferHorizonS_ = 5.0;
  int cyclicTargetPeriods_ = 2;
  double cyclicMaxAcc_ = 0.0;
  /// Expected write() period [ms]; only a seed, the real one is measured from
  /// write()'s `period`. Should match 1000 / controller_manager update_rate.
  double cyclicPushPeriodMs_ = 10.0;
  /// cyclic_subsample: "linear" (one command per write(), split linearly by the
  /// engine) or "hermite" (one-cycle samples on the (position, velocity) cubic).
  bool cyclicHermite_ = false;

  hardware_interface::CallbackReturn initImpl();
  std::string getHwParam(const std::string & key, const std::string & def) const;
  bool waitForCommunicating();
  void seedJointStates();
  /// Seed each position joint's command from its current WMX *command* position,
  /// so a write() that lands before any controller has written is a hold.
  bool seedPositionCommands();
  bool openAndStartStream();
};

}  // namespace wmx_r2_control

#endif  // WMX_R2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_
