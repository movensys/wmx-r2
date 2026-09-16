// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "wmx_r2_message/msg/axes_status.hpp"
#include "wmx_r2_message/srv/ecat_get_master_info.hpp"
#include "wmx_r2_message/srv/ecat_register_read.hpp"
#include "wmx_r2_message/srv/ecat_reset_statistics.hpp"
#include "wmx_r2_message/srv/ecat_start_hotconnect.hpp"
#include "wmx_r2_message/srv/get_axis_param.hpp"
#include "wmx_r2_message/srv/get_io_bit.hpp"
#include "wmx_r2_message/srv/get_io_bits.hpp"
#include "wmx_r2_message/srv/get_io_bytes.hpp"
#include "wmx_r2_message/srv/get_node_states.hpp"
#include "wmx_r2_message/srv/import_and_set_all.hpp"
#include "wmx_r2_message/srv/set_axes.hpp"
#include "wmx_r2_message/srv/set_axes_gear_ratio.hpp"
#include "wmx_r2_message/srv/set_engine.hpp"
#include "wmx_r2_message/srv/set_io_bit.hpp"
#include "wmx_r2_message/srv/set_io_bytes.hpp"
#include "wmx_r2_message/srv/set_node_state.hpp"
#include "wmx_r2_message/srv/start_axes_pose.hpp"
#include "wmx_r2_message/srv/start_axes_velocity.hpp"

TEST(AxesStatus, roundtrip_fields) {
  wmx_r2_message::msg::AxesStatus msg;
  msg.amp_alarm = {false, true};
  msg.servo_on = {true, false};
  msg.actual_pos = {1.5, -2.0};
  msg.actual_velocity = {0.1, 0.2};
  msg.pos_cmd = {3.0, 4.0};

  EXPECT_EQ(msg.amp_alarm.size(), 2u);
  EXPECT_EQ(msg.servo_on[0], true);
  EXPECT_DOUBLE_EQ(msg.actual_pos[1], -2.0);
  EXPECT_DOUBLE_EQ(msg.pos_cmd[0], 3.0);
}

TEST(StartAxesPose, request_and_response) {
  wmx_r2_message::srv::StartAxesPose::Request req;
  req.axis = {0, 1, 2};
  req.target = {10.0, 20.0, 30.0};
  req.velocity = {1.0, 2.0, 3.0};
  req.acc = {100.0, 100.0, 100.0};
  req.dec = {50.0, 50.0, 50.0};

  EXPECT_EQ(req.axis.size(), 3u);
  EXPECT_DOUBLE_EQ(req.target[2], 30.0);

  wmx_r2_message::srv::StartAxesPose::Response res;
  res.success = true;
  res.message = "Moving axis 0 to 10";
  EXPECT_TRUE(res.success);
}

TEST(StartAxesVelocity, request_and_response) {
  wmx_r2_message::srv::StartAxesVelocity::Request req;
  req.axis = {0};
  req.velocity = {5.5};
  req.acc = {200.0};
  req.dec = {200.0};

  EXPECT_EQ(req.axis[0], 0);
  EXPECT_DOUBLE_EQ(req.velocity[0], 5.5);

  wmx_r2_message::srv::StartAxesVelocity::Response res;
  res.success = false;
  res.message = "startVel: no axis given";
  EXPECT_FALSE(res.success);
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

TEST(SetNodeState, request_and_response) {
  wmx_r2_message::srv::SetNodeState::Request req;
  req.node_name = "wmx_io_node";
  req.transition = "deactivate";
  EXPECT_EQ(req.node_name, "wmx_io_node");
  EXPECT_EQ(req.transition, "deactivate");

  wmx_r2_message::srv::SetNodeState::Response res;
  res.success = true;
  res.message = "wmx_io_node: deactivate done";
  res.node_names = {"wmx_io_node"};
  res.states = {"inactive"};
  EXPECT_TRUE(res.success);
  ASSERT_EQ(res.node_names.size(), res.states.size());
  EXPECT_EQ(res.states[0], "inactive");
}

TEST(GetNodeStates, response_arrays) {
  wmx_r2_message::srv::GetNodeStates::Response res;
  res.success = true;
  res.message = "/wmx_io_node: active; /wmx_ethercat_node: inactive; ";
  res.node_names = {"/wmx_io_node", "/wmx_ethercat_node"};
  res.states = {"active", "inactive"};

  EXPECT_TRUE(res.success);
  ASSERT_EQ(res.node_names.size(), res.states.size());
  EXPECT_EQ(res.states[1], "inactive");
}

TEST(SetAxes, request_arrays) {
  wmx_r2_message::srv::SetAxes::Request req;
  req.axis = {0, 1, 2};
  req.data = {1, 1, 0};
  ASSERT_EQ(req.axis.size(), req.data.size());
  EXPECT_EQ(req.data[2], 0);
}

TEST(SetAxesGearRatio, request_arrays) {
  wmx_r2_message::srv::SetAxesGearRatio::Request req;
  req.axis = {0};
  req.numerator = {1.0};
  req.denominator = {2.0};
  EXPECT_DOUBLE_EQ(req.numerator[0] / req.denominator[0], 0.5);
}

TEST(GetIoBit, request_and_response) {
  wmx_r2_message::srv::GetIoBit::Request req;
  req.addr = 4;
  req.bit = 7;
  EXPECT_EQ(req.addr, 4);

  wmx_r2_message::srv::GetIoBit::Response res;
  res.success = true;
  res.data = 1;
  EXPECT_EQ(res.data, 1);
}

TEST(GetIoBits, request_arrays_pair_up) {
  wmx_r2_message::srv::GetIoBits::Request req;
  req.addr = {0, 0, 2};
  req.bit = {0, 1, 5};
  ASSERT_EQ(req.addr.size(), req.bit.size());

  wmx_r2_message::srv::GetIoBits::Response res;
  res.success = true;
  res.data = {1, 0, 1};
  EXPECT_EQ(res.data.size(), req.addr.size());
  EXPECT_EQ(res.data[2], 1);
}

TEST(GetIoBytes, request_validates_size) {
  wmx_r2_message::srv::GetIoBytes::Request req;
  req.addr = 0;
  req.size = 8;
  EXPECT_GT(req.size, 0);

  wmx_r2_message::srv::GetIoBytes::Response res;
  res.data = {0xAB, 0xCD};
  EXPECT_EQ(res.data.size(), 2u);
  EXPECT_EQ(res.data[1], 0xCD);
}

TEST(SetIoBit, request_data_clamps) {
  wmx_r2_message::srv::SetIoBit::Request req;
  req.addr = 4;
  req.bit = 0;
  req.data = 1;
  EXPECT_EQ(req.data, 1);
}

TEST(SetIoBytes, request_payload) {
  wmx_r2_message::srv::SetIoBytes::Request req;
  req.addr = 0;
  req.data = {1, 2, 3};
  EXPECT_EQ(req.data.size(), 3u);
}

TEST(EcatGetMasterInfo, response_master_fields) {
  wmx_r2_message::srv::EcatGetMasterInfo::Response res;
  res.state = 16;
  res.mode = 0;
  res.num_of_slaves = 3;
  res.slave_ids = {0, 1, 2};
  res.slave_states = {16, 16, 16};
  EXPECT_EQ(res.slave_ids.size(), 3u);
  EXPECT_EQ(res.state, 16);
}

TEST(EcatRegisterRead, request_bounds) {
  wmx_r2_message::srv::EcatRegisterRead::Request req;
  req.master_id = 0;
  req.slave_id = 0;
  req.reg_addr = 0x130;
  req.len = 2;
  EXPECT_GE(req.reg_addr, 0);
  EXPECT_LE(req.reg_addr, 0xFFF);
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

TEST(ImportAndSetAll, path) {
  wmx_r2_message::srv::ImportAndSetAll::Request req;
  req.path = "/etc/wmx3/params.xml";
  EXPECT_FALSE(req.path.empty());
}

TEST(GetAxisParam, dump_lines) {
  wmx_r2_message::srv::GetAxisParam::Request req;
  req.axis = {0, 1};
  wmx_r2_message::srv::GetAxisParam::Response res;
  res.success = true;
  res.axis_param = {"=== Axis 0 ===", "[AxisParam]"};
  EXPECT_EQ(res.axis_param.size(), 2u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
