// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_ENGINE_NODE_HPP_
#define WMX_ENGINE_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "wmx_r2_message/srv/get_wmx_params.hpp"
#include "wmx_r2_message/srv/load_wmx_params.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

class WmxEngineNodeApi
{
public:
  struct Config
  {
    std::string paramFilePath;
    int core = -1;
    int64_t affinityMask = 0;
  };

  WmxEngineNodeApi(const rclcpp::Logger & logger, const Config & config);
  ~WmxEngineNodeApi();

  int startEngine(std::string & message);
  int stopEngine(std::string & message);
  int getEngineStatus(std::string & message);

  int startCommunication(std::string & message);
  int stopCommunication(std::string & message);

  int loadWmxParams(const std::string & path, std::string & message);
  int getWmxParams(
    const std::vector<int32_t> & axes, std::vector<std::string> & dump,
    std::string & message);

  bool isDeviceOpen() const {return deviceOpen_;}
  bool isEngineStarted() const {return isEngineStarted_;}
  bool isCommStarted() const {return isCommStarted_;}

private:
  std::string errorText(int err);

  int engineState(wmx3Api::EngineState::T & state);

  rclcpp::Logger logger_;
  Config config_;

  const char * deviceName_ = "wmx_engine_node";
  unsigned int timeout_ = 10000;
  int maxRetries_ = 5;
  int retryDelay_ = 2000;

  std::recursive_mutex deviceMutex_;

  wmx3Api::WMX3Api wmx3Lib_;
  std::unique_ptr<wmx3Api::CoreMotion> cm_;

  std::atomic<bool> deviceOpen_{false};
  std::atomic<bool> isEngineStarted_{false};
  std::atomic<bool> isCommStarted_{false};
};

class WmxEngineNode : public rclcpp::Node
{
public:
  WmxEngineNode();
  ~WmxEngineNode();

private:
  std::unique_ptr<WmxEngineNodeApi> api_;
  std::thread startThread_;

  rclcpp::CallbackGroup::SharedPtr managerCbGroup_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setEngineService_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setCommService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getEngineStatusService_;
  rclcpp::Service<wmx_r2_message::srv::LoadWmxParams>::SharedPtr loadWmxParamsService_;
  rclcpp::Service<wmx_r2_message::srv::GetWmxParams>::SharedPtr getWmxParamsService_;

  void setEngineCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void setCommCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void getEngineStatusCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void loadWmxParamsCallback(
    const std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Request> request,
    std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Response> response);
  void getWmxParamsCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Response> response);
};

#endif  // WMX_ENGINE_NODE_HPP_
