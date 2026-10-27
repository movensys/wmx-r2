// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// Guard-clause tests for WmxCyclicStream. Every case here is rejected before the
// class touches the engine, so these run on any machine with the WMX3 SDK
// headers present -- no engine, no EtherCAT, no hardware. Behaviour that needs a
// live engine (open/start/push/abort against real axes) is covered by the SIL
// bring-up ladder, not here.
//
// Not built when the WMX3 SDK is absent; see CMakeLists.txt.
#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "wmx_r2_control/wmx_cyclic_stream.hpp"

using wmx_r2_control::WmxCyclicStream;

namespace
{

WmxCyclicStream::Config makeConfig()
{
  WmxCyclicStream::Config cfg;
  cfg.axes = {0, 1, 2, 3, 4, 5};
  return cfg;
}

}  // namespace

TEST(WmxCyclicStream, StartsClosedAndIdle)
{
  const WmxCyclicStream stream;
  EXPECT_FALSE(stream.isOpen());
  EXPECT_FALSE(stream.isRunning());
  EXPECT_FALSE(stream.faulted());
  EXPECT_EQ(stream.depth(), 0u);
  EXPECT_EQ(stream.rejected(), 0u);
  EXPECT_STREQ(stream.stateName(), "Stopped");
}

TEST(WmxCyclicStream, OpenRejectsNullHandles)
{
  WmxCyclicStream stream;
  std::string error;
  EXPECT_FALSE(stream.open(nullptr, nullptr, makeConfig(), 10.0, &error));
  EXPECT_NE(error.find("null"), std::string::npos) << error;
  EXPECT_FALSE(stream.isOpen());
}

TEST(WmxCyclicStream, OpenRejectsEmptyAxisList)
{
  WmxCyclicStream stream;
  WmxCyclicStream::Config cfg;  // axes intentionally empty
  std::string error;
  EXPECT_FALSE(stream.open(nullptr, nullptr, cfg, 10.0, &error));
  EXPECT_NE(error.find("axis list"), std::string::npos) << error;
}

TEST(WmxCyclicStream, OpenRejectsOutOfRangeAxis)
{
  WmxCyclicStream stream;
  std::string error;

  WmxCyclicStream::Config negative = makeConfig();
  negative.axes = {0, -1};
  EXPECT_FALSE(stream.open(nullptr, nullptr, negative, 10.0, &error));

  WmxCyclicStream::Config too_high = makeConfig();
  too_high.axes = {0, wmx3Api::constants::maxAxes};
  error.clear();
  EXPECT_FALSE(stream.open(nullptr, nullptr, too_high, 10.0, &error));
}

// Config is validated before the handles, so the whole config surface is
// reachable without an engine -- and a caller who got both wrong is told about
// the axis list, which is the one that comes from the ros2_control xacro.
TEST(WmxCyclicStream, OpenReportsAxisProblemsBeforeNullHandles)
{
  WmxCyclicStream stream;
  WmxCyclicStream::Config cfg;
  cfg.axes = {-7};
  std::string error;
  EXPECT_FALSE(stream.open(nullptr, nullptr, cfg, 10.0, &error));
  EXPECT_NE(error.find("out of range"), std::string::npos) << error;
}

TEST(WmxCyclicStream, PushBeforeStartIsRejected)
{
  WmxCyclicStream stream;
  std::string error;
  EXPECT_FALSE(stream.push({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, &error));
  EXPECT_NE(error.find("not running"), std::string::npos) << error;
}

TEST(WmxCyclicStream, RefreshAndStartBeforeOpenAreRejected)
{
  WmxCyclicStream stream;
  std::string error;
  EXPECT_FALSE(stream.refresh(&error));
  EXPECT_NE(error.find("not open"), std::string::npos) << error;

  error.clear();
  EXPECT_FALSE(stream.start(&error));
  EXPECT_NE(error.find("not open"), std::string::npos) << error;
}

// on_deactivate/on_cleanup may run after a failed activation, so these must be
// safe on an unopened stream rather than dereferencing a null buffer.
TEST(WmxCyclicStream, AbortAndCloseAreSafeWhenNotOpen)
{
  WmxCyclicStream stream;
  stream.abort();
  stream.close();
  stream.close();  // idempotent
  EXPECT_FALSE(stream.isOpen());
  EXPECT_FALSE(stream.isRunning());
}

TEST(WmxCyclicStream, ErrorOutParameterIsOptional)
{
  WmxCyclicStream stream;
  // A caller that does not want the message must not crash.
  EXPECT_FALSE(stream.open(nullptr, nullptr, makeConfig(), 10.0, nullptr));
  EXPECT_FALSE(stream.refresh(nullptr));
  EXPECT_FALSE(stream.push({0.0}, nullptr));
}

TEST(WmxCyclicStream, ObservePushPeriodIsSafeBeforeOpen)
{
  WmxCyclicStream stream;
  stream.observePushPeriod(10.0);  // no tracker yet; must be a no-op
  stream.observePushPeriod(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(stream.isOpen());
}
