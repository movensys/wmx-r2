// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_QOS_COMPAT_HPP_
#define WMX_QOS_COMPAT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"

inline auto servicesQos()
{
#if RCLCPP_VERSION_GTE(21, 0, 0)
  return rclcpp::ServicesQoS();
#else
  return rmw_qos_profile_services_default;
#endif
}

#endif  // WMX_QOS_COMPAT_HPP_
