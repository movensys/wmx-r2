// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_LIFECYCLE_MANAGER_NODE_HPP_
#define WMX_LIFECYCLE_MANAGER_NODE_HPP_

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "wmx_r2_message/srv/get_node_states.hpp"
#include "wmx_r2_message/srv/set_node_state.hpp"

class LifecycleManager
{
public:
  LifecycleManager(rclcpp::Node * node, const std::vector<std::string> & managedNodes);

  std::vector<std::string> discover();

  std::string state(const std::string & node);
  bool changeState(const std::string & node, uint8_t transition, std::string & message);
  bool bringUp(const std::string & node, std::string & message);
  bool bringDown(const std::string & node, std::string & message);
  bool shutdown(const std::string & node, std::string & message);

  static bool isKnownTransition(const std::string & transition);
  static std::string knownTransitions();

  bool applyTransition(
    const std::string & node, const std::string & transition, std::string & message);
  bool applyTransitionToAll(
    const std::string & transition, std::vector<std::string> & nodes, std::string & message);

  void bringUpDiscovered();
  void bringDownDiscovered(bool cleanup);

  std::string resolveNodeName(const std::string & name) const;

  void markHandled(const std::string & node) {handledNodes_.insert(node);}

  void clearHandled() {handledNodes_.clear();}

private:
  struct Clients
  {
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeState;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr getState;
  };

  const Clients & clientsFor(const std::string & node);

  rclcpp::Node * node_;
  rclcpp::Logger logger_;
  rclcpp::CallbackGroup::SharedPtr clientCbGroup_;

  std::vector<std::string> managedNodes_;
  std::unordered_map<std::string, Clients> clients_;
  std::set<std::string> handledNodes_;
};

class WmxLifecycleManagerNode : public rclcpp::Node
{
public:
  WmxLifecycleManagerNode();

private:
  std::unique_ptr<LifecycleManager> lifecycle_;

  std::string engineStatusService_;
  bool requireEngine_ = true;
  bool nodesAreUp_ = false;

  rclcpp::CallbackGroup::SharedPtr managerCbGroup_;
  rclcpp::CallbackGroup::SharedPtr clientCbGroup_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr engineStatusClient_;
  rclcpp::TimerBase::SharedPtr discoveryTimer_;

  rclcpp::Service<wmx_r2_message::srv::SetNodeState>::SharedPtr setNodeStateService_;
  rclcpp::Service<wmx_r2_message::srv::GetNodeStates>::SharedPtr getNodeStatesService_;

  bool isEngineCommunicating();
  void discoveryStep();

  void setNodeStateCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetNodeState::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetNodeState::Response> response);
  void getNodeStatesCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetNodeStates::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetNodeStates::Response> response);
};

#endif  // WMX_LIFECYCLE_MANAGER_NODE_HPP_
