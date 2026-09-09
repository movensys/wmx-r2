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

#include "wmx_r2_message/srv/get_axis_param.hpp"
#include "wmx_r2_message/srv/import_and_set_all.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

class WmxEngineNodeApi
{
public:
  struct Config
  {
    std::string wmxParamFilePath;
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

  int importAndSetAll(const std::string & path, std::string & message);
  int getAxisParam(
    const std::vector<int32_t> & axis, std::vector<std::string> & axisParam,
    std::string & message);

private:
  std::string errorToString(int err);

  bool isEngineStarted();
  bool isEngineCommunicating();

  rclcpp::Logger logger_;
  Config config_;

  const char * deviceName_ = "wmx_engine_node";
  unsigned int timeout_ = 10000;
  int maxRetries_ = 5;
  int retryDelay_ = 2000;

  std::recursive_mutex deviceMutex_;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::CoreMotion cm_;
};

class WmxEngineNode : public rclcpp::Node
{
public:
  WmxEngineNode();
  ~WmxEngineNode();

private:
  std::unique_ptr<WmxEngineNodeApi> api_;
  std::thread startEngineThread_;

  rclcpp::CallbackGroup::SharedPtr oneCbOnlyGroup_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setEngineService_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setCommunicationService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getEngineStatusService_;
  rclcpp::Service<wmx_r2_message::srv::ImportAndSetAll>::SharedPtr importAndSetAllService_;
  rclcpp::Service<wmx_r2_message::srv::GetAxisParam>::SharedPtr getAxisParamService_;

  void setEngineCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void setCommunicationCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void getEngineStatusCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void importAndSetAllCallback(
    const std::shared_ptr<wmx_r2_message::srv::ImportAndSetAll::Request> request,
    std::shared_ptr<wmx_r2_message::srv::ImportAndSetAll::Response> response);
  void getAxisParamCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetAxisParam::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetAxisParam::Response> response);
};

#endif  // WMX_ENGINE_NODE_HPP_
