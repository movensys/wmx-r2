// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_R2_CONTROL__PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP_
#define WMX_R2_CONTROL__PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"

namespace wmx_r2_control
{

// Transfer states shared with the hardware through trajectory_passthrough/transfer_state.
// Format follows the ros2 passthrough pattern.
enum TransferState : int
{
  TRANSFER_STATE_IDLE = 0,
  TRANSFER_STATE_WAITING_FOR_POINT = 1,
  TRANSFER_STATE_TRANSFERRING = 2,
  TRANSFER_STATE_TRANSFER_DONE = 3,
  TRANSFER_STATE_IN_MOTION = 4,
  TRANSFER_STATE_DONE = 5,
  TRANSFER_STATE_NEW_TRAJECTORY = 6,
};

// Forwards a complete FollowJointTrajectory goal point by point to the hardware, which
// executes it as one motion (WMX CSpline). The controller does no interpolation itself and
// claims no state interfaces.
class PassthroughTrajectoryController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
  override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJTrajAction>;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJTrajAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;

  rclcpp_action::GoalResponse goalReceived(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJTrajAction::Goal> goal);
  rclcpp_action::CancelResponse goalCancelled(std::shared_ptr<GoalHandle> goal_handle);
  void goalAccepted(std::shared_ptr<GoalHandle> goal_handle);
  bool validateGoal(const FollowJTrajAction::Goal & goal, std::string & message) const;

  // RT helpers
  bool startTrajectory(const FollowJTrajAction::Goal & goal);
  bool writeNextPoint(const trajectory_msgs::msg::JointTrajectory & traj);
  void finishTrajectory(const RealtimeGoalHandlePtr & active_goal);
  double readCommand(size_t index) const;
  bool writeCommand(size_t index, double value);

  // Parameters
  std::vector<std::string> joint_names_;
  double goal_time_param_ = 0.0;

  // Command interface indices (order of command_interface_configuration()).
  size_t time_from_start_index_ = 0;
  size_t transfer_state_index_ = 0;
  size_t abort_index_ = 0;
  size_t trajectory_size_index_ = 0;

  // Action handling
  rclcpp_action::Server<FollowJTrajAction>::SharedPtr action_server_;
  realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr> rt_active_goal_;
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  std::atomic<bool> trajectory_active_{false};
  std::atomic<bool> cancel_requested_{false};  // set by the cancel callback, served in update()

  // RT-only state of the trajectory being executed
  RealtimeGoalHandlePtr last_goal_;        // goal seen in the previous update()
  bool trajectory_started_ = false;
  bool goal_time_violated_ = false;
  size_t current_index_ = 0;
  std::vector<size_t> joint_map_;          // controller joint i -> index in goal joint_names
  double goal_time_tolerance_ = 0.0;       // effective goal time tolerance (0 = off)
  double max_trajectory_time_ = 0.0;
  double elapsed_time_ = 0.0;
};

}  // namespace wmx_r2_control

#endif  // WMX_R2_CONTROL__PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP_
