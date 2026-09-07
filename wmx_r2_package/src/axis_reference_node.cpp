// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// axis_reference_node
//
// One service, wmx/axis/reference (wmx_r2_message/srv/SetAxisPosition):
// "axis i is at position[i] right now". No motion. It is a HomeType CurrentPos
// homing whose HomePosition comes from the request instead of the parameter
// file, so a robot-side tool can declare the machine position in its own
// coordinates (movensys_cartesian_motion/home_reference does this in mm).
//
// Kept out of wmx_core_motion_node on purpose: that node stays a thin wrapper
// of the WMX3 CoreMotion basics, and referencing is a bring-up policy of a
// particular machine. Launch this node from the machine's launch file
// (wmx_r2_cartesian.launch.py) next to the other controllers.
//
// On absolute-encoder drives (AbsoluteEncoderMode 1) WMX3 turns the result
// into AbsoluteEncoderHomeOffset; export the parameters afterwards to keep it
// across a restart. The HomePosition of the axis is updated to the same value
// so that export carries it as well.

#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include "WMX3Api.h"
#include "CoreMotionApi.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "wmx_r2_message/srv/set_axis_position.hpp"

using wmx3Api::Config;
using wmx3Api::CoreMotion;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::WMX3Api;

class AxisReferenceNode : public rclcpp::Node
{
public:
  AxisReferenceNode();
  ~AxisReferenceNode();

private:
  bool initialized_ = false;
  int err_ = 0;
  char errString_[256];
  char buffer_[256];

  WMX3Api wmx3Lib_;
  CoreMotion wmx3LibCm_;
  Config::HomeParam homeParam_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;
  rclcpp::Service<wmx_r2_message::srv::SetAxisPosition>::SharedPtr referenceService_;

  void onEngineReady(std_msgs::msg::Bool::ConstSharedPtr msg);
  void reference(
    const std::shared_ptr<wmx_r2_message::srv::SetAxisPosition::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxisPosition::Response> response);
};

AxisReferenceNode::AxisReferenceNode()
: Node("axis_reference_node")
{
  // Same latched flag the other controllers wait on before attaching.
  auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
  engineReadySub_ = this->create_subscription<std_msgs::msg::Bool>(
    "wmx/engine/ready", ready_qos,
    std::bind(&AxisReferenceNode::onEngineReady, this, std::placeholders::_1));

  // The service exists from the start and answers "not ready" until attached,
  // so a client can wait_for_service() right after launch.
  referenceService_ = this->create_service<wmx_r2_message::srv::SetAxisPosition>(
    "wmx/axis/reference",
    std::bind(&AxisReferenceNode::reference, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "axis_reference_node waiting for engine...");
}

AxisReferenceNode::~AxisReferenceNode()
{
  if (initialized_) {
    err_ = wmx3Lib_.CloseDevice();
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(this->get_logger(), "Failed to close device. Error=%d (%s)", err_, errString_);
    }
  }
  RCLCPP_INFO(this->get_logger(), "axis_reference_node is stopped");
}

void AxisReferenceNode::onEngineReady(std_msgs::msg::Bool::ConstSharedPtr msg)
{
  if (!msg->data || initialized_) {
    return;
  }

  unsigned int timeout = 10000;
  err_ = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout);
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    if (err_ == ErrorCode::StartProcessLockError) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to attach to device (lock busy, will retry on next signal).");
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to attach to device. Error=%d (%s)", err_, errString_);
    }
    return;
  }

  wmx3Lib_.SetDeviceName("axis_reference_node");
  wmx3LibCm_ = CoreMotion(&wmx3Lib_);

  initialized_ = true;
  engineReadySub_.reset();
  RCLCPP_INFO(this->get_logger(), "axis_reference_node is ready (wmx/axis/reference)");
}

void AxisReferenceNode::reference(
  const std::shared_ptr<wmx_r2_message::srv::SetAxisPosition::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxisPosition::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "CoreMotion not initialized. Engine not ready.";
    return;
  }
  if (request->index.size() != request->position.size()) {
    response->success = false;
    response->message = "index and position must have the same length";
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    const int axis = request->index[i];
    err_ = wmx3LibCm_.config->GetHomeParam(axis, &homeParam_);
    if (err_ == ErrorCode::None) {
      homeParam_.homeType = Config::HomeType::CurrentPos;
      homeParam_.homePosition = request->position[i];
      err_ = wmx3LibCm_.config->SetHomeParam(axis, &homeParam_);
    }
    if (err_ == ErrorCode::None) {
      err_ = wmx3LibCm_.home->StartHome(axis);
    }
    if (err_ == ErrorCode::None) {
      err_ = wmx3LibCm_.motion->Wait(axis);
    }

    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      snprintf(
        buffer_, sizeof(buffer_), "Failed to reference axis %d at %.6f. Error=%d (%s)",
        axis, request->position[i], err_, errString_);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
      all_success = false;
    } else {
      snprintf(
        buffer_, sizeof(buffer_), "Referenced axis %d at %.6f", axis, request->position[i]);
      RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    }
    msg_stream << buffer_ << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AxisReferenceNode>());
  rclcpp::shutdown();
  return 0;
}
