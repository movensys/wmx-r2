// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_r2_control/passthrough_trajectory_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace wmx_r2_control
{

namespace
{
constexpr auto kActionMonitorPeriod = std::chrono::milliseconds(50);
constexpr double kNoValue = -1.0;
// Same limit as the native joint_trajectory_controller and the hardware spline buffer.
constexpr size_t kMaxTrajectoryPoints = 1000;
const char kInterfacePrefix[] = "trajectory_passthrough/";

int toState(double value)
{
  return std::isfinite(value) ? static_cast<int>(std::lround(value)) : -1;
}
}  // namespace

controller_interface::CallbackReturn PassthroughTrajectoryController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<double>("constraints.goal_time", 0.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PassthroughTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  goal_time_param_ = get_node()->get_parameter("constraints.goal_time").as_double();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(logger, "Parameter 'joints' is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  const size_t n = joint_names_.size();
  joint_map_.assign(n, 0);

  // Command interfaces: setpoint_positions_0..n-1 first, then the handshake interfaces.
  time_from_start_index_ = n;
  transfer_state_index_ = n + 1;
  abort_index_ = n + 2;
  trajectory_size_index_ = n + 3;

  action_server_.reset();
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    std::bind(
      &PassthroughTrajectoryController::goalReceived, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&PassthroughTrajectoryController::goalCancelled, this, std::placeholders::_1),
    std::bind(&PassthroughTrajectoryController::goalAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger, "Configured %zu joints, goal_time=%.3f s", n, goal_time_param_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PassthroughTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const std::string prefix = kInterfacePrefix;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    conf.names.push_back(prefix + "setpoint_positions_" + std::to_string(i));
  }
  conf.names.push_back(prefix + "time_from_start");
  conf.names.push_back(prefix + "transfer_state");
  conf.names.push_back(prefix + "abort");
  conf.names.push_back(prefix + "trajectory_size");
  return conf;
}

controller_interface::InterfaceConfiguration
PassthroughTrajectoryController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn PassthroughTrajectoryController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const size_t n = joint_names_.size();
  if (command_interfaces_.size() != n + 4) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu", n + 4,
      command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  last_goal_.reset();
  trajectory_started_ = false;
  current_index_ = 0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PassthroughTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  const RealtimeGoalHandlePtr active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal) {
    auto result = std::make_shared<FollowJTrajAction::Result>();
    if (cancel_requested_) {
      active_goal->setCanceled(result);
    } else {
      result->error_code = FollowJTrajAction::Result::INVALID_GOAL;
      result->error_string = "Controller deactivated while executing the trajectory";
      active_goal->setAborted(result);
      RCLCPP_WARN(get_node()->get_logger(), "%s", result->error_string.c_str());
    }
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
  trajectory_active_ = false;
  cancel_requested_ = false;
  trajectory_started_ = false;
  last_goal_.reset();

  // Stop a motion that is still being transferred or executed by the hardware.
  const int state = toState(readCommand(transfer_state_index_));
  if (state != TRANSFER_STATE_IDLE && state != TRANSFER_STATE_DONE) {
    if (!writeCommand(abort_index_, 1.0)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to write abort on deactivate");
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PassthroughTrajectoryController::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const RealtimeGoalHandlePtr active_goal = *rt_active_goal_.readFromRT();
  const int state = toState(readCommand(transfer_state_index_));
  bool ok = true;

  // A different goal (or none) since the last cycle: never continue the previous transfer.
  if (active_goal != last_goal_) {
    last_goal_ = active_goal;
    trajectory_started_ = false;
    current_index_ = 0;
  }

  if (active_goal && trajectory_active_) {
    const auto & goal = *active_goal->gh_->get_goal();

    if (!trajectory_started_) {
      if (cancel_requested_) {
        // Nothing of this goal reached the hardware yet.
        active_goal->setCanceled(std::make_shared<FollowJTrajAction::Result>());
        rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
        cancel_requested_ = false;
        trajectory_active_ = false;
      } else if (state == TRANSFER_STATE_IDLE) {
        ok = startTrajectory(goal);
      } else if (state == TRANSFER_STATE_DONE) {
        // Leftover of a previous trajectory: acknowledge before starting the new one.
        ok &= writeCommand(transfer_state_index_, TRANSFER_STATE_IDLE);
        ok &= writeCommand(abort_index_, 0.0);
      } else {
        // A previous trajectory is still running on the hardware: stop it, wait for DONE.
        ok &= writeCommand(abort_index_, 1.0);
      }
    } else if (state == TRANSFER_STATE_DONE) {
      finishTrajectory(active_goal);
    } else if (cancel_requested_) {
      // Reported as canceled once the hardware has stopped the axes (DONE).
      ok &= writeCommand(abort_index_, 1.0);
    } else if (state == TRANSFER_STATE_WAITING_FOR_POINT) {
      ok = writeNextPoint(goal.trajectory);
    } else if (state == TRANSFER_STATE_IN_MOTION) {
      elapsed_time_ += period.seconds();
      if (goal_time_tolerance_ > 0.0 && !goal_time_violated_ &&
        elapsed_time_ > max_trajectory_time_ + goal_time_tolerance_)
      {
        // Overrun: the hardware stops the axes and answers DONE.
        goal_time_violated_ = true;
        ok &= writeCommand(abort_index_, 1.0);
      }
    }
  } else {
    if (state == TRANSFER_STATE_DONE) {
      // Hardware finished (or stopped after a deactivate): back to idle.
      ok &= writeCommand(transfer_state_index_, TRANSFER_STATE_IDLE);
      ok &= writeCommand(abort_index_, 0.0);
    } else if (state != TRANSFER_STATE_IDLE) {
      // No active goal but the hardware is busy: stop it.
      ok &= writeCommand(abort_index_, 1.0);
    }
  }

  if (!ok) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Failed to write trajectory_passthrough command interfaces");
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

bool PassthroughTrajectoryController::startTrajectory(const FollowJTrajAction::Goal & goal)
{
  const auto & traj = goal.trajectory;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    for (size_t j = 0; j < traj.joint_names.size(); ++j) {
      if (traj.joint_names[j] == joint_names_[i]) {
        joint_map_[i] = j;
        break;
      }
    }
  }
  const double goal_time = rclcpp::Duration(goal.goal_time_tolerance).seconds();
  goal_time_tolerance_ = goal_time > 0.0 ? goal_time : goal_time_param_;
  goal_time_violated_ = false;
  max_trajectory_time_ = rclcpp::Duration(traj.points.back().time_from_start).seconds();
  elapsed_time_ = 0.0;
  current_index_ = 0;
  trajectory_started_ = true;

  bool ok = writeCommand(abort_index_, 0.0);
  ok &= writeCommand(trajectory_size_index_, static_cast<double>(traj.points.size()));
  ok &= writeCommand(transfer_state_index_, TRANSFER_STATE_NEW_TRAJECTORY);
  return ok;
}

bool PassthroughTrajectoryController::writeNextPoint(
  const trajectory_msgs::msg::JointTrajectory & traj)
{
  if (current_index_ >= traj.points.size()) {
    return writeCommand(transfer_state_index_, TRANSFER_STATE_TRANSFER_DONE);
  }
  const auto & point = traj.points[current_index_];
  bool ok = writeCommand(
    time_from_start_index_, rclcpp::Duration(point.time_from_start).seconds());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    ok &= writeCommand(i, point.positions[joint_map_[i]]);
  }
  ok &= writeCommand(transfer_state_index_, TRANSFER_STATE_TRANSFERRING);
  ++current_index_;
  return ok;
}

void PassthroughTrajectoryController::finishTrajectory(const RealtimeGoalHandlePtr & active_goal)
{
  auto logger = get_node()->get_logger();
  auto result = active_goal->preallocated_result_;
  const bool overran = goal_time_violated_ ||
    (goal_time_tolerance_ > 0.0 && elapsed_time_ > max_trajectory_time_ + goal_time_tolerance_);

  if (cancel_requested_) {
    result->error_code = FollowJTrajAction::Result::SUCCESSFUL;
    result->error_string = "";
    active_goal->setCanceled(result);
    RCLCPP_INFO(logger, "Trajectory canceled, axes stopped");
  } else if (overran) {
    result->error_code = FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED;
    result->error_string = "Goal not reached within goal_time tolerance";
    active_goal->setAborted(result);
    RCLCPP_ERROR(
      logger, "Goal time violated: %.3f s elapsed, %.3f s planned + %.3f s tolerance",
      elapsed_time_, max_trajectory_time_, goal_time_tolerance_);
  } else if (readCommand(abort_index_) == 1.0) {
    result->error_code = FollowJTrajAction::Result::INVALID_GOAL;
    result->error_string = "Trajectory aborted by the hardware";
    active_goal->setAborted(result);
    RCLCPP_ERROR(logger, "Trajectory aborted by the hardware");
  } else {
    result->error_code = FollowJTrajAction::Result::SUCCESSFUL;
    result->error_string = "";
    active_goal->setSucceeded(result);
    RCLCPP_INFO(
      logger, "Trajectory done in %.3f s (planned %.3f s)", elapsed_time_,
      max_trajectory_time_);
  }

  rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  trajectory_started_ = false;
  cancel_requested_ = false;
  trajectory_active_ = false;
  if (!writeCommand(transfer_state_index_, TRANSFER_STATE_IDLE) ||
    !writeCommand(abort_index_, 0.0))
  {
    RCLCPP_ERROR(logger, "Failed to reset trajectory_passthrough interfaces");
  }
}

double PassthroughTrajectoryController::readCommand(size_t index) const
{
  return command_interfaces_[index].get_optional().value_or(kNoValue);
}

bool PassthroughTrajectoryController::writeCommand(size_t index, double value)
{
  return command_interfaces_[index].set_value(value);
}

rclcpp_action::GoalResponse PassthroughTrajectoryController::goalReceived(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJTrajAction::Goal>)
{
  auto logger = get_node()->get_logger();
  if (get_lifecycle_id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(logger, "Rejected goal: controller is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (trajectory_active_) {
    RCLCPP_ERROR(logger, "Rejected goal: a trajectory is already executing");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // Trajectory content is checked in goalAccepted(), so a bad goal ends with INVALID_GOAL
  // (a plain REJECT carries no error code), like the native controller.
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

bool PassthroughTrajectoryController::validateGoal(
  const FollowJTrajAction::Goal & goal, std::string & message) const
{
  const auto & traj = goal.trajectory;
  if (traj.joint_names.size() != joint_names_.size()) {
    message = std::to_string(traj.joint_names.size()) + " joints given, controller has " +
      std::to_string(joint_names_.size());
    return false;
  }
  for (const auto & joint : joint_names_) {
    if (std::find(traj.joint_names.begin(), traj.joint_names.end(), joint) ==
      traj.joint_names.end())
    {
      message = "joint " + joint + " missing";
      return false;
    }
  }
  if (traj.points.empty() || traj.points.size() > kMaxTrajectoryPoints) {
    message = std::to_string(traj.points.size()) + " points, must be in [1, " +
      std::to_string(kMaxTrajectoryPoints) + "]";
    return false;
  }
  for (size_t i = 0; i < traj.points.size(); ++i) {
    if (traj.points[i].positions.size() != joint_names_.size()) {
      message = "point " + std::to_string(i) + " has " +
        std::to_string(traj.points[i].positions.size()) + " positions, expected " +
        std::to_string(joint_names_.size());
      return false;
    }
    if (i > 0 &&
      rclcpp::Duration(traj.points[i].time_from_start) <=
      rclcpp::Duration(traj.points[i - 1].time_from_start))
    {
      message = "time_from_start of point " + std::to_string(i) + " is not after point " +
        std::to_string(i - 1);
      return false;
    }
  }
  return true;
}

rclcpp_action::CancelResponse PassthroughTrajectoryController::goalCancelled(
  std::shared_ptr<GoalHandle> goal_handle)
{
  const RealtimeGoalHandlePtr active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle) {
    // update() aborts the hardware and reports canceled once the axes have stopped.
    RCLCPP_INFO(get_node()->get_logger(), "Cancelling active trajectory");
    cancel_requested_ = true;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PassthroughTrajectoryController::goalAccepted(std::shared_ptr<GoalHandle> goal_handle)
{
  std::string message;
  if (!validateGoal(*goal_handle->get_goal(), message)) {
    auto result = std::make_shared<FollowJTrajAction::Result>();
    result->error_code = FollowJTrajAction::Result::INVALID_GOAL;
    result->error_string = "Invalid goal: " + message;
    goal_handle->execute();  // ACCEPTED -> EXECUTING; abort is only valid from EXECUTING
    goal_handle->abort(result);
    RCLCPP_ERROR(get_node()->get_logger(), "%s", result->error_string.c_str());
    return;
  }

  RCLCPP_INFO(
    get_node()->get_logger(), "Accepted trajectory with %zu points",
    goal_handle->get_goal()->trajectory.points.size());

  auto rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->execute();
  cancel_requested_ = false;
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Goal state changes requested from update() are published from this non-RT timer.
  goal_handle_timer_.reset();
  goal_handle_timer_ = get_node()->create_wall_timer(
    kActionMonitorPeriod, std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
  trajectory_active_ = true;
}

}  // namespace wmx_r2_control

PLUGINLIB_EXPORT_CLASS(
  wmx_r2_control::PassthroughTrajectoryController, controller_interface::ControllerInterface)
