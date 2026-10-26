// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "wmx_r2_message/msg/axes_pose.hpp"
#include "wmx_r2_message/msg/axes_status.hpp"
#include "wmx_r2_message/msg/axes_velocity.hpp"
#include "wmx_r2_message/srv/ecat_get_network_state.hpp"
#include "wmx_r2_message/srv/ecat_register_read.hpp"
#include "wmx_r2_message/srv/ecat_reset_statistics.hpp"
#include "wmx_r2_message/srv/ecat_start_hotconnect.hpp"
#include "wmx_r2_message/srv/get_io_bit.hpp"
#include "wmx_r2_message/srv/get_io_bytes.hpp"
#include "wmx_r2_message/srv/get_wmx_params.hpp"
#include "wmx_r2_message/srv/load_wmx_params.hpp"
#include "wmx_r2_message/srv/set_axes.hpp"
#include "wmx_r2_message/srv/set_axes_gear_ratio.hpp"
#include "wmx_r2_message/srv/set_engine.hpp"
#include "wmx_r2_message/srv/set_io_bit.hpp"
#include "wmx_r2_message/srv/set_io_bytes.hpp"

TEST(AxesStatus, roundtrip_fields) {
  wmx_r2_message::msg::AxesStatus msg;
  msg.amp_alarms = {false, true};
  msg.amp_alarm_codes = {0, 0x7380};
  msg.servo_on = {true, false};
  msg.actual_positions = {1.5, -2.0};
  msg.actual_velocities = {0.1, 0.2};
  msg.position_commands = {3.0, 4.0};

  EXPECT_EQ(msg.amp_alarms.size(), 2u);
  EXPECT_EQ(msg.amp_alarm_codes[1], 0x7380);
  EXPECT_EQ(msg.servo_on[0], true);
  EXPECT_DOUBLE_EQ(msg.actual_positions[1], -2.0);
  EXPECT_DOUBLE_EQ(msg.position_commands[0], 3.0);
}

TEST(AxesPose, roundtrip_fields) {
  wmx_r2_message::msg::AxesPose msg;
  msg.indices = {0, 1, 2};
  msg.positions = {10.0, 20.0, 30.0};
  msg.velocities = {1.0, 2.0, 3.0};
  msg.accelerations = {100.0, 100.0, 100.0};
  msg.decelerations = {50.0, 50.0, 50.0};

  EXPECT_EQ(msg.indices.size(), 3u);
  EXPECT_DOUBLE_EQ(msg.positions[2], 30.0);
}

TEST(AxesVelocity, roundtrip_fields) {
  wmx_r2_message::msg::AxesVelocity msg;
  msg.indices = {0};
  msg.velocities = {5.5};
  msg.accelerations = {200.0};
  msg.decelerations = {200.0};

  EXPECT_EQ(msg.indices[0], 0);
  EXPECT_DOUBLE_EQ(msg.velocities[0], 5.5);
}

TEST(SetEngine, request_and_response) {
  wmx_r2_message::srv::SetEngine::Request req;
  req.data = true;
  req.path = "/opt/wmx3/";
  req.name = "test_node";
  EXPECT_TRUE(req.data);
  EXPECT_EQ(req.path, "/opt/wmx3/");

  wmx_r2_message::srv::SetEngine::Response res;
  res.success = false;
  res.message = "not ready";
  EXPECT_FALSE(res.success);
}

TEST(SetAxes, request_arrays) {
  wmx_r2_message::srv::SetAxes::Request req;
  req.indices = {0, 1, 2};
  req.data = {1, 1, 0};
  ASSERT_EQ(req.indices.size(), req.data.size());
  EXPECT_EQ(req.data[2], 0);
}

TEST(SetAxesGearRatio, request_arrays) {
  wmx_r2_message::srv::SetAxesGearRatio::Request req;
  req.indices = {0};
  req.numerators = {1.0};
  req.denominators = {2.0};
  EXPECT_DOUBLE_EQ(req.numerators[0] / req.denominators[0], 0.5);
}

TEST(GetIoBit, request_and_response) {
  wmx_r2_message::srv::GetIoBit::Request req;
  req.byte = 4;
  req.bit = 7;
  EXPECT_EQ(req.byte, 4);

  wmx_r2_message::srv::GetIoBit::Response res;
  res.success = true;
  res.value = 1;
  EXPECT_EQ(res.value, 1);
}

TEST(GetIoBytes, request_validates_length) {
  wmx_r2_message::srv::GetIoBytes::Request req;
  req.byte = 0;
  req.length = 8;
  EXPECT_GT(req.length, 0);

  wmx_r2_message::srv::GetIoBytes::Response res;
  res.data = {0xAB, 0xCD};
  EXPECT_EQ(res.data.size(), 2u);
  EXPECT_EQ(res.data[1], 0xCD);
}

TEST(SetIoBit, request_value_clamps) {
  wmx_r2_message::srv::SetIoBit::Request req;
  req.byte = 4;
  req.bit = 0;
  req.value = 1;
  EXPECT_EQ(req.value, 1);
}

TEST(SetIoBytes, request_payload) {
  wmx_r2_message::srv::SetIoBytes::Request req;
  req.byte = 0;
  req.data = {1, 2, 3};
  EXPECT_EQ(req.data.size(), 3u);
}

TEST(EcatGetNetworkState, response_master_fields) {
  wmx_r2_message::srv::EcatGetNetworkState::Response res;
  res.master_state = 16;
  res.master_mode = 0;
  res.num_of_slaves = 3;
  res.slave_ids = {0, 1, 2};
  res.slave_states = {16, 16, 16};
  EXPECT_EQ(res.slave_ids.size(), 3u);
  EXPECT_EQ(res.master_state, 16);
}

TEST(EcatRegisterRead, request_bounds) {
  wmx_r2_message::srv::EcatRegisterRead::Request req;
  req.master_id = 0;
  req.slave_id = 0;
  req.reg_address = 0x130;
  req.length = 2;
  EXPECT_GE(req.reg_address, 0);
  EXPECT_LE(req.reg_address, 0xFFF);
}

TEST(EcatResetStatistics, master_id) {
  wmx_r2_message::srv::EcatResetStatistics::Request req;
  req.master_id = 1;
  EXPECT_EQ(req.master_id, 1);
}

TEST(EcatStartHotconnect, master_id) {
  wmx_r2_message::srv::EcatStartHotconnect::Request req;
  req.master_id = 0;
  EXPECT_EQ(req.master_id, 0);
}

TEST(LoadWmxParams, path) {
  wmx_r2_message::srv::LoadWmxParams::Request req;
  req.file_path = "/etc/wmx3/params.xml";
  EXPECT_FALSE(req.file_path.empty());
}

TEST(GetWmxParams, dump_lines) {
  wmx_r2_message::srv::GetWmxParams::Request req;
  req.indices = {0, 1};
  wmx_r2_message::srv::GetWmxParams::Response res;
  res.success = true;
  res.params_dump = {"=== Axis 0 ===", "[AxisParam]"};
  EXPECT_EQ(res.params_dump.size(), 2u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
