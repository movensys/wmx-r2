// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_R2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_
#define WMX_R2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "WMX3Api.h"
#include "CoreMotionApi.h"

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

  int attachDevice(std::string & message);
  void releaseDevice();

  int importAndSetAll(const std::string & path, std::string & message);

  int getStatus(
    const std::vector<int> & axes, std::vector<AxisFeedback> & feedback,
    bool & communicating, std::string & message);

  int startVel(int axis, double omega, std::string & message);
  int setServoOn(int axis, int on, std::string & message);
  int clearAmpAlarm(int axis, std::string & message);

  bool isDeviceOpen() const {return isDeviceAttached_;}

private:
  rclcpp::Logger logger_;
  Config config_;

  unsigned int timeout_ = 10000;
  unsigned int servoOnTimeout_ = 2000;

  mutable std::mutex deviceMutex_;
  bool isDeviceAttached_ = false;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::CoreMotion cm_;
};

enum class JointMode
{
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
  double lastCmd = 0.0;
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

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("WmxSystemHardware");
  rclcpp::Clock clock_{RCL_STEADY_TIME};

  std::unique_ptr<WmxSystemHardwareApi> api_;

  std::vector<WmxJoint> joints_;
  std::vector<int> axes_;

  std::vector<WmxSystemHardwareApi::AxisFeedback> feedback_;
  bool communicating_ = false;

  std::string wmxParamFile_;
  int maxDeviceRetries_ = 30;
  bool autoServoOn_ = true;
  hardware_interface::CallbackReturn initImpl();
  std::string getHwParam(const std::string & key, const std::string & def) const;
  bool waitForCommunicating();
  void seedJointStates();
};

}  // namespace wmx_r2_control

#endif  // WMX_R2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_
