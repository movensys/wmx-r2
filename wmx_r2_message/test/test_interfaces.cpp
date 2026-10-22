// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "wmx_r2_message/msg/axis_pose.hpp"
#include "wmx_r2_message/msg/axis_state.hpp"
#include "wmx_r2_message/msg/axis_velocity.hpp"
#include "wmx_r2_message/srv/ecat_get_network_state.hpp"
#include "wmx_r2_message/srv/ecat_register_read.hpp"
#include "wmx_r2_message/srv/ecat_reset_statistics.hpp"
#include "wmx_r2_message/srv/ecat_start_hotconnect.hpp"
#include "wmx_r2_message/srv/get_io_bit.hpp"
#include "wmx_r2_message/srv/get_io_bytes.hpp"
#include "wmx_r2_message/srv/get_wmx_params.hpp"
#include "wmx_r2_message/srv/load_wmx_params.hpp"
#include "wmx_r2_message/srv/set_axis.hpp"
#include "wmx_r2_message/srv/set_axis_gear_ratio.hpp"
#include "wmx_r2_message/srv/set_engine.hpp"
#include "wmx_r2_message/srv/set_io_bit.hpp"
#include "wmx_r2_message/srv/set_io_bytes.hpp"

TEST(AxisState, roundtrip_fields) {
  wmx_r2_message::msg::AxisState msg;
  msg.amp_alarm = {false, true};
  msg.amp_alarm_code = {0, 0x7380};
  msg.servo_on = {true, false};
  msg.actual_pos = {1.5, -2.0};
  msg.actual_velocity = {0.1, 0.2};
  msg.pos_cmd = {3.0, 4.0};

  EXPECT_EQ(msg.amp_alarm.size(), 2u);
  EXPECT_EQ(msg.amp_alarm_code[1], 0x7380);
  EXPECT_EQ(msg.servo_on[0], true);
  EXPECT_DOUBLE_EQ(msg.actual_pos[1], -2.0);
  EXPECT_DOUBLE_EQ(msg.pos_cmd[0], 3.0);
}

TEST(AxisPose, roundtrip_fields) {
  wmx_r2_message::msg::AxisPose msg;
  msg.index = {0, 1, 2};
  msg.target = {10.0, 20.0, 30.0};
  msg.velocity = {1.0, 2.0, 3.0};
  msg.acc = {100.0, 100.0, 100.0};
  msg.dec = {50.0, 50.0, 50.0};

  EXPECT_EQ(msg.index.size(), 3u);
  EXPECT_DOUBLE_EQ(msg.target[2], 30.0);
}

TEST(AxisVelocity, roundtrip_fields) {
  wmx_r2_message::msg::AxisVelocity msg;
  msg.index = {0};
  msg.velocity = {5.5};
  msg.acc = {200.0};
  msg.dec = {200.0};

  EXPECT_EQ(msg.index[0], 0);
  EXPECT_DOUBLE_EQ(msg.velocity[0], 5.5);
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

TEST(SetAxis, request_arrays) {
  wmx_r2_message::srv::SetAxis::Request req;
  req.index = {0, 1, 2};
  req.data = {1, 1, 0};
  ASSERT_EQ(req.index.size(), req.data.size());
  EXPECT_EQ(req.data[2], 0);
}

TEST(SetAxisGearRatio, request_arrays) {
  wmx_r2_message::srv::SetAxisGearRatio::Request req;
  req.index = {0};
  req.numerator = {1.0};
  req.denominator = {2.0};
  EXPECT_DOUBLE_EQ(req.numerator[0] / req.denominator[0], 0.5);
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
  req.index = {0, 1};
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
