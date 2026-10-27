// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// on_init() tests for WmxSystemHardware: joint-mode parsing, the position-vs-
// velocity mixing rejection, and which command interfaces get exported.
//
// on_init only reads the parsed HardwareInfo -- it never touches the WMX device
// (that starts at on_configure) -- so these run with no engine, no EtherCAT and
// no hardware. They do need the SDK headers to compile, so they are not built on
// a CI runner; see CMakeLists.txt.
#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include "wmx_r2_control/wmx_system_hardware.hpp"

using wmx_r2_control::WmxSystemHardware;

namespace
{

hardware_interface::InterfaceInfo makeInterface(const std::string & name)
{
  hardware_interface::InterfaceInfo info;
  info.name = name;
  return info;
}

/// Build a HardwareInfo for six joints, each with the given command interface
/// ("" for none), mirroring what the ros2_control xacro parses to.
hardware_interface::HardwareInfo makeInfo(
  const std::vector<std::string> & command_interfaces, bool with_axis_param = true)
{
  hardware_interface::HardwareInfo info;
  info.name = "WmxTestSystem";
  info.type = "system";
  info.hardware_plugin_name = "wmx_r2_control/WmxSystemHardware";

  int axis = 0;
  for (const std::string & command_interface : command_interfaces) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint" + std::to_string(axis + 1);
    if (with_axis_param) {
      joint.parameters["axis"] = std::to_string(axis);
    }
    if (!command_interface.empty()) {
      joint.command_interfaces.push_back(makeInterface(command_interface));
    }
    joint.state_interfaces.push_back(makeInterface("position"));
    joint.state_interfaces.push_back(makeInterface("velocity"));
    info.joints.push_back(joint);
    ++axis;
  }
  return info;
}

hardware_interface::CallbackReturn initWith(
  WmxSystemHardware * hardware, const hardware_interface::HardwareInfo & info)
{
#if WMX_HAS_HW_COMPONENT_INTERFACE_PARAMS
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  return hardware->on_init(params);
#else
  return hardware->on_init(info);
#endif
}

// export_command_interfaces() is deprecated upstream in favour of
// on_export_command_interfaces(), but it is the method this class overrides, so
// it is what the test must exercise. Migrating is a separate change.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
std::vector<std::string> exportedCommandInterfaceNames(WmxSystemHardware * hardware)
{
  std::vector<std::string> names;
  for (const auto & interface : hardware->export_command_interfaces()) {
    names.push_back(interface.get_name());
  }
  return names;
}
#pragma GCC diagnostic pop

constexpr size_t kJoints = 6u;

std::vector<std::string> allOf(const std::string & name)
{
  return std::vector<std::string>(kJoints, name);
}

}  // namespace

TEST(WmxSystemHardwareInit, AcceptsSixPositionJoints)
{
  WmxSystemHardware hardware;
  ASSERT_EQ(initWith(&hardware, makeInfo(allOf("position"))),
    hardware_interface::CallbackReturn::SUCCESS);

  const auto names = exportedCommandInterfaceNames(&hardware);
  ASSERT_EQ(names.size(), kJoints);
  for (size_t i = 0; i < kJoints; ++i) {
    EXPECT_EQ(names[i], "joint" + std::to_string(i + 1) + "/position");
  }
}

// Regression: the diffbot's velocity path must be untouched by the addition of
// position mode.
TEST(WmxSystemHardwareInit, AcceptsVelocityJointsAsBefore)
{
  WmxSystemHardware hardware;
  ASSERT_EQ(initWith(&hardware, makeInfo(allOf("velocity"))),
    hardware_interface::CallbackReturn::SUCCESS);

  const auto names = exportedCommandInterfaceNames(&hardware);
  ASSERT_EQ(names.size(), kJoints);
  for (size_t i = 0; i < kJoints; ++i) {
    EXPECT_EQ(names[i], "joint" + std::to_string(i + 1) + "/velocity");
  }
}

// Regression that matters most for this change: the CR3A xacro as currently
// shipped declares no command interfaces, so every joint is state-only and this
// commit must leave that configuration exporting nothing and commanding nothing.
TEST(WmxSystemHardwareInit, StateOnlyJointsExportNoCommandInterfaces)
{
  WmxSystemHardware hardware;
  ASSERT_EQ(initWith(&hardware, makeInfo(allOf(""))),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(exportedCommandInterfaceNames(&hardware).empty());
}

// Position joints stream through the cyclic buffer, which puts their axes in
// DirectControl; velocity joints use StartVel, which needs the axis idle. One
// ros2_control block cannot serve both, so this must fail loudly at init rather
// than fight over the axes at runtime.
TEST(WmxSystemHardwareInit, RejectsMixedPositionAndVelocityJoints)
{
  WmxSystemHardware hardware;
  const auto info = makeInfo({"position", "position", "position", "velocity", "", ""});
  EXPECT_EQ(initWith(&hardware, info), hardware_interface::CallbackReturn::ERROR);
}

// Position mixed with state-only is fine: the state-only joints are simply not
// commanded, which is how a gripper or an idler axis rides along.
TEST(WmxSystemHardwareInit, AllowsPositionMixedWithStateOnly)
{
  WmxSystemHardware hardware;
  const auto info = makeInfo({"position", "position", "", "", "", ""});
  ASSERT_EQ(initWith(&hardware, info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(exportedCommandInterfaceNames(&hardware).size(), 2u);
}

TEST(WmxSystemHardwareInit, RejectsUnsupportedCommandInterface)
{
  WmxSystemHardware hardware;
  EXPECT_EQ(initWith(&hardware, makeInfo(allOf("effort"))),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(WmxSystemHardwareInit, RejectsMultipleCommandInterfacesOnOneJoint)
{
  WmxSystemHardware hardware;
  auto info = makeInfo(allOf("position"));
  info.joints[0].command_interfaces.push_back(makeInterface("velocity"));
  EXPECT_EQ(initWith(&hardware, info), hardware_interface::CallbackReturn::ERROR);
}

TEST(WmxSystemHardwareInit, RejectsJointWithoutAxisParameter)
{
  WmxSystemHardware hardware;
  EXPECT_EQ(initWith(&hardware, makeInfo(allOf("position"), false)),
    hardware_interface::CallbackReturn::ERROR);
}
