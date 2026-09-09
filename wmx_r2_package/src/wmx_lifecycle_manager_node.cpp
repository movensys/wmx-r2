// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_lifecycle_manager_node.hpp"

#include <algorithm>

#include "wmx_qos_compat.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using lifecycle_msgs::msg::Transition;

namespace
{
constexpr const char * kEngineStatusService = "wmx/engine/get_engine_status";

constexpr std::chrono::seconds kServiceWaitTimeout{5};
constexpr std::chrono::seconds kTransitionTimeout{30};
constexpr std::chrono::seconds kQueryServiceWaitTimeout{1};
constexpr std::chrono::seconds kQueryTimeout{5};

constexpr int kEngineMissLimit = 3;

bool isDeviceLevelNode(const std::string & fullName)
{
  const std::size_t start = fullName.find_last_of('/');
  const std::string name =
    (start == std::string::npos) ? fullName : fullName.substr(start + 1);
  return name.rfind("wmx_", 0) == 0;
}

const char * transitionLabel(uint8_t transition)
{
  switch (transition) {
    case Transition::TRANSITION_CONFIGURE:             return "configure";
    case Transition::TRANSITION_CLEANUP:               return "cleanup";
    case Transition::TRANSITION_ACTIVATE:              return "activate";
    case Transition::TRANSITION_DEACTIVATE:            return "deactivate";
    case Transition::TRANSITION_UNCONFIGURED_SHUTDOWN: return "shutdown";
    case Transition::TRANSITION_INACTIVE_SHUTDOWN:     return "shutdown";
    case Transition::TRANSITION_ACTIVE_SHUTDOWN:       return "shutdown";
    default:                                           return "unknown transition";
  }
}
}  // namespace

LifecycleManager::LifecycleManager(
  rclcpp::Node * node, const std::vector<std::string> & managedNodes)
: node_(node), logger_(node->get_logger()), managedNodes_(managedNodes)
{
  clientCbGroup_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
}

std::string LifecycleManager::resolveNodeName(const std::string & name) const
{
  if (name.empty() || name.front() == '/') {
    return name;
  }

  const std::string ns = node_->get_effective_namespace();
  return (ns == "/") ? "/" + name : ns + "/" + name;
}

const LifecycleManager::Clients & LifecycleManager::clientsFor(const std::string & node)
{
  const auto cached = clients_.find(node);
  if (cached != clients_.end()) {
    return cached->second;
  }

  Clients clients;
  clients.changeState = node_->create_client<lifecycle_msgs::srv::ChangeState>(
    node + "/change_state", servicesQos(), clientCbGroup_);
  clients.getState = node_->create_client<lifecycle_msgs::srv::GetState>(
    node + "/get_state", servicesQos(), clientCbGroup_);

  return clients_.emplace(node, std::move(clients)).first->second;
}

std::vector<std::string> LifecycleManager::discover()
{
  std::vector<std::string> found;

  const auto nodes = node_->get_node_graph_interface()->get_node_names_and_namespaces();

  for (const auto & entry : nodes) {
    const std::string & name = entry.first;
    const std::string & ns = entry.second;
    const std::string fullName = (ns == "/") ? "/" + name : ns + "/" + name;

    if (fullName == node_->get_fully_qualified_name()) {
      continue;
    }

    std::map<std::string, std::vector<std::string>> services;
    try {
      services = node_->get_service_names_and_types_by_node(name, ns);
    } catch (const std::runtime_error &) {
      continue;
    }

    const auto service = services.find(fullName + "/change_state");
    if (service == services.end()) {
      continue;
    }
    if (std::find(
        service->second.begin(), service->second.end(),
        "lifecycle_msgs/srv/ChangeState") == service->second.end())
    {
      continue;
    }

    found.push_back(fullName);
  }

  if (!managedNodes_.empty()) {
    std::vector<std::string> managed;
    for (const std::string & name : managedNodes_) {
      const std::string fullName = resolveNodeName(name);
      if (std::find(found.begin(), found.end(), fullName) != found.end()) {
        managed.push_back(fullName);
      }
    }
    return managed;
  }

  std::sort(
    found.begin(), found.end(),
    [](const std::string & a, const std::string & b) {
      const bool aIsDeviceLevel = isDeviceLevelNode(a);
      const bool bIsDeviceLevel = isDeviceLevelNode(b);
      if (aIsDeviceLevel != bIsDeviceLevel) {
        return aIsDeviceLevel;
      }
      return a < b;
    });

  return found;
}

std::string LifecycleManager::state(const std::string & node)
{
  const Clients & clients = clientsFor(node);

  if (!clients.getState->wait_for_service(kQueryServiceWaitTimeout)) {
    return "unavailable";
  }

  auto future = clients.getState->async_send_request(
    std::make_shared<lifecycle_msgs::srv::GetState::Request>());

  if (future.wait_for(kQueryTimeout) != std::future_status::ready) {
    clients.getState->remove_pending_request(future);
    return "unknown";
  }

  return future.get()->current_state.label;
}

bool LifecycleManager::changeState(
  const std::string & node, uint8_t transition, std::string & message)
{
  const char * label = transitionLabel(transition);
  const Clients & clients = clientsFor(node);

  if (!clients.changeState->wait_for_service(kServiceWaitTimeout)) {
    message = node + ": " + clients.changeState->get_service_name() + " is not available";
    return false;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  auto future = clients.changeState->async_send_request(request);
  if (future.wait_for(kTransitionTimeout) != std::future_status::ready) {
    clients.changeState->remove_pending_request(future);
    message = node + ": " + label + " timed out";
    return false;
  }

  if (!future.get()->success) {
    message = node + ": " + label + " was rejected (state is now '" +
      state(node) + "')";
    return false;
  }

  message = node + ": " + label + " done";
  return true;
}

bool LifecycleManager::bringUp(const std::string & node, std::string & message)
{
  std::string label = state(node);

  if (label == "unconfigured") {
    if (!changeState(node, Transition::TRANSITION_CONFIGURE, message)) {
      return false;
    }
    label = state(node);
  }

  if (label == "inactive") {
    return changeState(node, Transition::TRANSITION_ACTIVATE, message);
  }

  if (label == "active") {
    message = node + ": already active";
    return true;
  }

  message = node + ": cannot be brought up from state '" + label + "'";
  return false;
}

bool LifecycleManager::bringDown(const std::string & node, std::string & message)
{
  const std::string label = state(node);

  if (label == "active") {
    return changeState(node, Transition::TRANSITION_DEACTIVATE, message);
  }

  if (label == "inactive" || label == "unconfigured" || label == "finalized") {
    message = node + ": already down (state '" + label + "')";
    return true;
  }

  message = node + ": cannot be brought down from state '" + label + "'";
  return false;
}

bool LifecycleManager::shutdown(const std::string & node, std::string & message)
{
  const std::string label = state(node);

  if (label == "finalized") {
    message = node + ": already finalized";
    return true;
  }

  uint8_t transition = Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
  if (label == "active") {
    transition = Transition::TRANSITION_ACTIVE_SHUTDOWN;
  } else if (label == "inactive") {
    transition = Transition::TRANSITION_INACTIVE_SHUTDOWN;
  } else if (label != "unconfigured") {
    message = node + ": cannot be shut down from state '" + label + "'";
    return false;
  }

  return changeState(node, transition, message);
}

bool LifecycleManager::isKnownTransition(const std::string & transition)
{
  return transition == "configure" || transition == "activate" ||
         transition == "deactivate" || transition == "cleanup" ||
         transition == "shutdown" || transition == "bringup" ||
         transition == "bringdown";
}

bool LifecycleManager::isDownwardTransition(const std::string & transition)
{
  return transition == "deactivate" || transition == "cleanup" ||
         transition == "shutdown" || transition == "bringdown";
}

void LifecycleManager::markManual(const std::string & node, bool goingDown)
{
  handledNodes_.insert(node);

  if (!goingDown) {
    return;
  }

  RCLCPP_WARN(
    logger_,
    "%s was taken down by hand. Automatic bring-up stays off for it until it is brought up "
    "by hand or the engine restarts.", node.c_str());
}

std::string LifecycleManager::knownTransitions()
{
  return "configure, activate, deactivate, cleanup, shutdown, bringup or bringdown";
}

bool LifecycleManager::applyTransition(
  const std::string & node, const std::string & transition, std::string & message)
{
  if (transition == "configure") {
    return changeState(node, Transition::TRANSITION_CONFIGURE, message);
  }
  if (transition == "activate") {
    return changeState(node, Transition::TRANSITION_ACTIVATE, message);
  }
  if (transition == "deactivate") {
    return changeState(node, Transition::TRANSITION_DEACTIVATE, message);
  }
  if (transition == "cleanup") {
    return changeState(node, Transition::TRANSITION_CLEANUP, message);
  }
  if (transition == "shutdown") {
    return shutdown(node, message);
  }
  if (transition == "bringup") {
    return bringUp(node, message);
  }
  if (transition == "bringdown") {
    return bringDown(node, message);
  }

  message = node + ": unknown transition '" + transition + "'";
  return false;
}

bool LifecycleManager::applyTransitionToAll(
  const std::string & transition, std::vector<std::string> & nodes, std::string & message)
{
  nodes = discover();

  const bool goingDown = isDownwardTransition(transition);
  if (goingDown) {
    std::reverse(nodes.begin(), nodes.end());
  }

  bool allSucceeded = true;
  message.clear();

  for (const std::string & node : nodes) {
    std::string nodeMessage;

    if (applyTransition(node, transition, nodeMessage)) {
      RCLCPP_INFO(logger_, "%s", nodeMessage.c_str());
      markManual(node, goingDown);
    } else {
      RCLCPP_ERROR(logger_, "%s", nodeMessage.c_str());
      allSucceeded = false;
    }

    message += nodeMessage + "; ";
  }

  if (nodes.empty()) {
    message = "No lifecycle nodes found.";
  }

  return allSucceeded;
}

void LifecycleManager::bringDownDiscovered(bool cleanup)
{
  const std::vector<std::string> nodes = discover();

  for (auto it = nodes.rbegin(); it != nodes.rend(); ++it) {
    const std::string & node = *it;
    std::string message;

    if (bringDown(node, message)) {
      RCLCPP_INFO(logger_, "%s", message.c_str());
    } else {
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      continue;
    }

    if (cleanup && state(node) == "inactive") {
      if (changeState(node, Transition::TRANSITION_CLEANUP, message)) {
        RCLCPP_INFO(logger_, "%s", message.c_str());
      } else {
        RCLCPP_ERROR(logger_, "%s", message.c_str());
      }
    }
  }
}

void LifecycleManager::bringUpDiscovered()
{
  const std::vector<std::string> discovered = discover();

  for (auto it = handledNodes_.begin(); it != handledNodes_.end(); ) {
    if (std::find(discovered.begin(), discovered.end(), *it) == discovered.end()) {
      RCLCPP_INFO(logger_, "%s left the graph", it->c_str());
      it = handledNodes_.erase(it);
    } else {
      ++it;
    }
  }

  for (const std::string & node : discovered) {
    if (handledNodes_.count(node) > 0) {
      continue;
    }

    std::string message;
    if (bringUp(node, message)) {
      RCLCPP_INFO(logger_, "%s", message.c_str());
      handledNodes_.insert(node);
    } else {
      RCLCPP_ERROR_THROTTLE(
        logger_, *node_->get_clock(), 10000, "%s", message.c_str());
    }
  }
}

WmxLifecycleManagerNode::WmxLifecycleManagerNode()
: Node("wmx_lifecycle_manager_node")
{
  const auto managedNodes = this->declare_parameter<std::vector<std::string>>(
    "managed_nodes", std::vector<std::string>{});
  const double period = this->declare_parameter<double>("discovery_period", 2.0);

  lifecycle_ = std::make_unique<LifecycleManager>(this, managedNodes);

  oneCbOnlyGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  clientCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  engineStatusClient_ = this->create_client<std_srvs::srv::Trigger>(
    kEngineStatusService, servicesQos(), clientCbGroup_);

  setNodeStateService_ = this->create_service<wmx_r2_message::srv::SetNodeState>(
    "wmx/lifecycle/set_node_state",
    std::bind(&WmxLifecycleManagerNode::setNodeStateCallback, this, _1, _2),
    servicesQos(), oneCbOnlyGroup_);

  getNodeStatesService_ = this->create_service<wmx_r2_message::srv::GetNodeStates>(
    "wmx/lifecycle/get_node_states",
    std::bind(&WmxLifecycleManagerNode::getNodeStatesCallback, this, _1, _2),
    servicesQos(), oneCbOnlyGroup_);

  discoveryTimer_ = this->create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&WmxLifecycleManagerNode::discoveryStep, this),
    oneCbOnlyGroup_);

  RCLCPP_INFO(this->get_logger(), "wmx_lifecycle_manager_node is ready");
}

bool WmxLifecycleManagerNode::isEngineCommunicating()
{
  if (!engineStatusClient_->wait_for_service(std::chrono::seconds(1))) {
    return false;
  }

  auto future = engineStatusClient_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());

  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    engineStatusClient_->remove_pending_request(future);
    return false;
  }

  const auto result = future.get();
  return result->success && result->message == "Communicating";
}

void WmxLifecycleManagerNode::discoveryStep()
{
  if (isEngineCommunicating()) {
    engineMissCount_ = 0;
    nodesAreUp_ = true;
    lifecycle_->bringUpDiscovered();
    return;
  }

  if (!nodesAreUp_) {
    return;
  }

  ++engineMissCount_;
  if (engineMissCount_ < kEngineMissLimit) {
    RCLCPP_WARN(
      this->get_logger(), "Engine status query failed (%d of %d). Holding the nodes up.",
      engineMissCount_, kEngineMissLimit);
    return;
  }

  RCLCPP_WARN(this->get_logger(), "Engine stopped; taking the lifecycle nodes down");
  lifecycle_->bringDownDiscovered(true);
  lifecycle_->clearHandled();
  nodesAreUp_ = false;
  engineMissCount_ = 0;
}

void WmxLifecycleManagerNode::setNodeStateCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetNodeState::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetNodeState::Response> response)
{
  const std::string & transition = request->transition;

  if (!LifecycleManager::isKnownTransition(transition)) {
    response->success = false;
    response->message = "Unknown transition '" + transition + "'. Use " +
      LifecycleManager::knownTransitions() + ".";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  std::string message;

  if (request->node_name.empty()) {
    std::vector<std::string> nodes;
    response->success = lifecycle_->applyTransitionToAll(transition, nodes, message);
    response->node_names = nodes;
  } else {
    const std::string node = lifecycle_->resolveNodeName(request->node_name);
    response->success = lifecycle_->applyTransition(node, transition, message);

    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
      lifecycle_->markManual(node, LifecycleManager::isDownwardTransition(transition));
    } else {
      RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    }

    response->node_names.push_back(node);
  }

  response->message = message;
  for (const std::string & node : response->node_names) {
    response->states.push_back(lifecycle_->state(node));
  }
}

void WmxLifecycleManagerNode::getNodeStatesCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetNodeStates::Request>,
  std::shared_ptr<wmx_r2_message::srv::GetNodeStates::Response> response)
{
  std::string message;

  for (const std::string & node : lifecycle_->discover()) {
    const std::string label = lifecycle_->state(node);
    response->node_names.push_back(node);
    response->states.push_back(label);
    message += node + ": " + label + "; ";
  }

  response->success = true;
  response->message = message.empty() ? "No lifecycle nodes found." : message;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxLifecycleManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
