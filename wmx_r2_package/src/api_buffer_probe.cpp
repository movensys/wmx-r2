// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// api_buffer_probe
//
// Phase 1 bench measurement for streaming MoveIt Servo commands into the WMX3
// API buffer. Run-once node: it attaches to the engine, measures, prints a
// report, and shuts down. It answers three questions that size and shape the
// servo_stream_controller:
//
//   1. How many bytes does one streamed setpoint occupy in the API buffer?
//      (StartLinearIntplPos block + USleep block, measured separately.)
//   2. What is the real resolution of ApiBuffer::USleep? The engine ticks at
//      1 kHz, so sub-millisecond sleeps are expected to quantise to one cycle.
//      The pacing controller in servo_stream_controller depends on this.
//   3. Do the block counters behave such that a locally maintained counter can
//      stand in for GetCumulativeBlockCount, which this SDK does not provide?
//
// SAFETY: this program commands no motion.
//   - Measurement 1 records StartLinearIntplPos blocks into a channel that is
//     left in the Stop state and is Cleared before anything is ever executed.
//   - Every recorded target is the axis' CURRENT commanded position, so even an
//     erroneous execute would be a no-op.
//   - Measurements 2 and 3 execute buffers containing USleep blocks only.
// It is safe to run with the servos off, and that is the recommended way.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "ApiBufferApi.h"

using std::placeholders::_1;
using wmx3Api::ApiBuffer;
using wmx3Api::ApiBufferOptions;
using wmx3Api::ApiBufferState;
using wmx3Api::ApiBufferStatus;
using wmx3Api::AxisSelection;
using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::Motion;
using wmx3Api::ProfileType;
using wmx3Api::SizeUnit;
using wmx3Api::WMX3Api;

namespace
{
// GetStatus is cyclic: it reflects a snapshot from the end of the previous
// communication cycle. Poll rather than sleep-and-hope when waiting on it.
constexpr auto kPollInterval = std::chrono::microseconds(200);
constexpr auto kDrainTimeout = std::chrono::seconds(10);

// A Clear, or a freshly recorded block, is not visible in GetStatus until the
// engine has ticked. Sampling immediately after a write reads a half-applied
// state — which silently corrupts any measurement built on freeSize deltas.
// Poll at whole-cycle granularity and require several identical readings.
constexpr auto kSettlePoll = std::chrono::milliseconds(1);
constexpr auto kSettleTimeout = std::chrono::seconds(2);
constexpr int kSettleSamples = 5;
}  // namespace

class ApiBufferProbe : public rclcpp::Node
{
public:
  ApiBufferProbe();
  ~ApiBufferProbe();

private:
  // --- WMX3 handles -------------------------------------------------------
  // Two devices, mirroring the servo_stream_controller design.
  // StartRecordBufferChannel is DEVICE-wide: once recording, every WMX3 call
  // made through that device is recorded rather than executed. The probe needs
  // CoreMotion::GetStatus (which WOULD be recorded) while blocks are being
  // recorded, so the recording device is kept separate from the control device.
  WMX3Api wmxRec_;
  WMX3Api wmxCtl_;
  std::unique_ptr<CoreMotion> cmRec_;
  std::unique_ptr<CoreMotion> cmCtl_;
  std::unique_ptr<ApiBuffer> abRec_;
  std::unique_ptr<ApiBuffer> abCtl_;

  bool recAttached_ = false;
  bool ctlAttached_ = false;
  bool bufferCreated_ = false;

  int err_ = 0;
  char errString_[256];

  // --- parameters ---------------------------------------------------------
  std::vector<int64_t> jointAxes_;
  int channel_ = 0;
  int bufferSizeMb_ = 5;
  int sampleCount_ = 100;
  int sleepSampleCount_ = 50;

  AxisSelection axisSel_;
  Motion::LinearIntplCommand intpl_;

  std::atomic<bool> initializing_{false};
  std::thread probeThread_;

  // Engine readiness is polled through wmx/engine/get_engine_status, the same
  // way wmx_lifecycle_manager_node does it.
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr engineStatusClient_;
  rclcpp::TimerBase::SharedPtr engineProbeTimer_;
  bool engineProbePending_ = false;

  // --- helpers ------------------------------------------------------------
  void setRosParameter();
  void probeEngine();
  void onEngineReady();
  bool attachDevice(WMX3Api * lib, const char * name);
  bool check(int err, const char * what);
  bool getStatus(ApiBufferStatus * st);
  bool settle(ApiBufferStatus * st);
  bool resetChannel();
  bool drain(int * cycles_waited);
  void seedNeutralCommand();

  // --- measurements -------------------------------------------------------
  void runProbe();
  bool measureBlockSize(int * motion_bytes, int * sleep_bytes, int * buffer_size);
  void measureSleepResolution();
  void measureBlockAccounting();
};

ApiBufferProbe::ApiBufferProbe()
: Node("api_buffer_probe")
{
  setRosParameter();

  engineStatusClient_ =
    this->create_client<std_srvs::srv::Trigger>("wmx/engine/get_engine_status");
  engineProbeTimer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&ApiBufferProbe::probeEngine, this));

  RCLCPP_INFO(this->get_logger(), "api_buffer_probe waiting for engine...");
}

ApiBufferProbe::~ApiBufferProbe()
{
  if (probeThread_.joinable()) {
    probeThread_.join();
  }

  // Leave the channel empty and stopped regardless of how we got here, so a
  // later run (or another node) does not inherit our blocks.
  if (bufferCreated_ && abCtl_) {
    abCtl_->Halt(channel_);
    abCtl_->Clear(channel_);
    abCtl_->FreeApiBuffer(channel_);
  }
  if (recAttached_) {
    wmxRec_.CloseDevice();
  }
  if (ctlAttached_) {
    wmxCtl_.CloseDevice();
  }

  RCLCPP_INFO(this->get_logger(), "api_buffer_probe is stopped");
}

void ApiBufferProbe::setRosParameter()
{
  this->declare_parameter<std::vector<int64_t>>(
    "joint_axes", std::vector<int64_t>{0, 1, 2, 3, 4, 5});
  this->declare_parameter<int>("api_buffer_channel", 0);
  this->declare_parameter<int>("api_buffer_size_mb", 5);
  this->declare_parameter<int>("sample_count", 100);
  this->declare_parameter<int>("sleep_sample_count", 50);

  this->get_parameter("joint_axes", jointAxes_);
  this->get_parameter("api_buffer_channel", channel_);
  this->get_parameter("api_buffer_size_mb", bufferSizeMb_);
  this->get_parameter("sample_count", sampleCount_);
  this->get_parameter("sleep_sample_count", sleepSampleCount_);

  axisSel_.axisCount = jointAxes_.size();
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    axisSel_.axis[i] = static_cast<int>(jointAxes_[i]);
  }

  std::string axes_str;
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    if (i > 0) {axes_str += ", ";}
    axes_str += std::to_string(jointAxes_[i]);
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", axes_str.c_str());
  RCLCPP_INFO(this->get_logger(), "api_buffer_channel: %d", channel_);
  RCLCPP_INFO(this->get_logger(), "api_buffer_size_mb: %d", bufferSizeMb_);
  RCLCPP_INFO(this->get_logger(), "sample_count: %d", sampleCount_);
  RCLCPP_INFO(this->get_logger(), "sleep_sample_count: %d", sleepSampleCount_);
  RCLCPP_INFO(this->get_logger(), "===========================");
}

void ApiBufferProbe::probeEngine()
{
  if (initializing_ || engineProbePending_ || !engineStatusClient_->service_is_ready()) {
    return;
  }

  engineProbePending_ = true;
  engineStatusClient_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      engineProbePending_ = false;
      const auto result = future.get();
      if (result->success && result->message == "Communicating") {
        onEngineReady();
      }
    });
}

void ApiBufferProbe::onEngineReady()
{
  if (initializing_.exchange(true)) {
    return;
  }
  engineProbeTimer_->cancel();
  RCLCPP_INFO(this->get_logger(), "Engine ready — starting probe on dedicated thread...");
  probeThread_ = std::thread(&ApiBufferProbe::runProbe, this);
}

bool ApiBufferProbe::check(int err, const char * what)
{
  if (err == ErrorCode::None) {
    return true;
  }
  err_ = err;
  ApiBuffer::ErrorToString(err, errString_, sizeof(errString_));
  RCLCPP_ERROR(this->get_logger(), "%s failed. Error=%d (%s)", what, err, errString_);
  return false;
}

bool ApiBufferProbe::attachDevice(WMX3Api * lib, const char * name)
{
  static constexpr int kMaxDeviceRetries = 30;
  unsigned int timeout = 10000;
  int err = ErrorCode::None;

  for (int attempt = 1; attempt <= kMaxDeviceRetries; ++attempt) {
    err = lib->CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout);
    if (err == ErrorCode::None) {
      break;
    }
    lib->ErrorToString(err, errString_, sizeof(errString_));
    if (err == ErrorCode::StartProcessLockError) {
      RCLCPP_WARN(
        this->get_logger(), "Device lock busy for '%s', retrying in 1s... (%d/%d)",
        name, attempt, kMaxDeviceRetries);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to attach device '%s'. Error=%d (%s)", name, err, errString_);
      return false;
    }
  }

  if (err != ErrorCode::None) {
    RCLCPP_FATAL(
      this->get_logger(), "Device '%s' lock busy after %d retries", name, kMaxDeviceRetries);
    return false;
  }

  lib->SetDeviceName(name);
  RCLCPP_INFO(this->get_logger(), "Attached WMX3 device '%s'", name);
  return true;
}

bool ApiBufferProbe::getStatus(ApiBufferStatus * st)
{
  return check(abCtl_->GetStatus(channel_, st), "ApiBuffer::GetStatus");
}

// Wait until the cyclic status stops moving, so callers never measure against a
// counter that is still catching up with a Clear or a burst of recorded blocks.
bool ApiBufferProbe::settle(ApiBufferStatus * st)
{
  const auto deadline = std::chrono::steady_clock::now() + kSettleTimeout;
  int64_t prev_blocks = -1;
  int prev_free = -1;
  int stable = 0;

  while (rclcpp::ok()) {
    if (!getStatus(st)) {
      return false;
    }
    if (st->blockCount == prev_blocks && st->freeSize == prev_free) {
      if (++stable >= kSettleSamples) {
        return true;
      }
    } else {
      stable = 0;
      prev_blocks = st->blockCount;
      prev_free = st->freeSize;
    }
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_WARN(
        this->get_logger(), "Status never settled (blockCount=%lld freeSize=%d)",
        st->blockCount, st->freeSize);
      return false;
    }
    std::this_thread::sleep_for(kSettlePoll);
  }
  return false;
}

// Halt then Clear. Order matters: Halt controls the execution state
// (Active -> Stop), Clear controls the content (empties, counters -> 0).
// Clearing an Active channel leaves it Active, which is not what we want
// between measurements.
//
// The Clear is asynchronous: it lands a cycle or more later. Wait for it, or
// the next measurement's baseline freeSize still counts the blocks being freed.
bool ApiBufferProbe::resetChannel()
{
  abCtl_->Halt(channel_);      // may legitimately fail if already stopped
  if (!check(abCtl_->Clear(channel_), "ApiBuffer::Clear")) {
    return false;
  }

  ApiBufferStatus st;
  if (!settle(&st)) {
    return false;
  }
  if (st.blockCount != 0 || st.remainingBlockCount != 0) {
    RCLCPP_WARN(
      this->get_logger(), "Channel not empty after Clear (blockCount=%lld remaining=%lld)",
      st.blockCount, st.remainingBlockCount);
    return false;
  }
  return true;
}

// Wait until every block has been consumed. Returns false on timeout.
bool ApiBufferProbe::drain(int * cycles_waited)
{
  const auto deadline = std::chrono::steady_clock::now() + kDrainTimeout;
  int polls = 0;

  while (rclcpp::ok()) {
    ApiBufferStatus st;
    if (!getStatus(&st)) {
      return false;
    }
    if (st.remainingBlockCount == 0) {
      if (cycles_waited != nullptr) {
        *cycles_waited = polls;
      }
      return true;
    }
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(
        this->get_logger(), "Timed out draining buffer (remaining=%lld)", st.remainingBlockCount);
      return false;
    }
    ++polls;
    std::this_thread::sleep_for(kPollInterval);
  }
  return false;
}

// Build a LinearIntplCommand whose targets are the axes' CURRENT commanded
// positions, so the recorded blocks are motionless no-ops. The axis array is
// filled in joint_axes order: LinearIntplProfileCalcMode is read from axis[0],
// so this ordering is part of the command's semantics, not a detail.
void ApiBufferProbe::seedNeutralCommand()
{
  CoreMotionStatus cmStatus;
  cmCtl_->GetStatus(&cmStatus);

  intpl_.axisCount = jointAxes_.size();
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    const int axis = static_cast<int>(jointAxes_[i]);
    intpl_.axis[i] = axis;
    intpl_.target[i] = cmStatus.axesStatus[axis].posCmd;   // no-op target
    intpl_.maxVelocity[i] = 1.0;
    intpl_.maxAcc[i] = 10.0;
    intpl_.maxDec[i] = 10.0;
  }
  intpl_.profile.type = ProfileType::Trapezoidal;
  intpl_.profile.velocity = 0.0;
  intpl_.profile.acc = 0.0;
  intpl_.profile.dec = 0.0;
  intpl_.profile.startingVelocity = 0.0;
  intpl_.profile.endVelocity = 0.0;
}

// --- Measurement 1: bytes per block ---------------------------------------
// Records into a channel that is never executed, then Clears. Motion and sleep
// blocks are measured in separate passes so the two costs are known
// independently — the pacing design adds exactly one of each per setpoint.
bool ApiBufferProbe::measureBlockSize(int * motion_bytes, int * sleep_bytes, int * buffer_size)
{
  ApiBufferStatus st;

  // ---- pass A: StartLinearIntplPos blocks ----
  if (!resetChannel() || !settle(&st)) {
    return false;
  }
  *buffer_size = st.bufferSize;
  const int free_before_motion = st.freeSize;

  seedNeutralCommand();

  int recorded_motion = 0;
  for (int i = 0; i < sampleCount_; ++i) {
    if (!check(abRec_->StartRecordBufferChannel(channel_), "StartRecordBufferChannel")) {
      return false;
    }
    const int rec_err = cmRec_->motion->StartLinearIntplPos(&intpl_);
    abRec_->EndRecordBufferChannel();

    if (rec_err != ErrorCode::None) {
      wmxRec_.ErrorToString(rec_err, errString_, sizeof(errString_));
      RCLCPP_WARN(
        this->get_logger(), "Recording stopped at sample %d: Error=%d (%s)",
        i, rec_err, errString_);
      break;
    }
    ++recorded_motion;
  }

  if (recorded_motion == 0) {
    RCLCPP_ERROR(this->get_logger(), "Recorded no motion blocks; cannot measure");
    return false;
  }

  if (!settle(&st)) {
    return false;
  }
  *motion_bytes = (free_before_motion - st.freeSize) / recorded_motion;
  RCLCPP_INFO(
    this->get_logger(), "  motion pass: %d blocks consumed %d bytes",
    recorded_motion, free_before_motion - st.freeSize);

  // ---- pass B: USleep blocks ----
  // Clear FIRST: the motion blocks above must never reach the execution engine.
  if (!resetChannel() || !settle(&st)) {
    return false;
  }
  const int free_before_sleep = st.freeSize;

  int recorded_sleep = 0;
  for (int i = 0; i < sampleCount_; ++i) {
    if (!check(abRec_->StartRecordBufferChannel(channel_), "StartRecordBufferChannel")) {
      return false;
    }
    const int rec_err = abRec_->USleep(25000);
    abRec_->EndRecordBufferChannel();

    if (rec_err != ErrorCode::None) {
      break;
    }
    ++recorded_sleep;
  }

  if (recorded_sleep == 0) {
    RCLCPP_ERROR(this->get_logger(), "Recorded no sleep blocks; cannot measure");
    return false;
  }

  if (!settle(&st)) {
    return false;
  }
  *sleep_bytes = (free_before_sleep - st.freeSize) / recorded_sleep;
  RCLCPP_INFO(
    this->get_logger(), "  sleep pass:  %d blocks consumed %d bytes",
    recorded_sleep, free_before_sleep - st.freeSize);

  return resetChannel();
}

// --- Measurement 2: USleep resolution --------------------------------------
// Executes sleep-only buffers. If USleep quantises to the 1 ms engine cycle,
// 1500 us will behave as 1000 or 2000 us and the pacing controller must work in
// whole cycles.
//
// remainingBlockCount is decremented when a sleep BEGINS, not when it ends, so
// a trailing dummy USleep(0) is appended: remaining then reaches 0 only once the
// last real sleep has completed. This also verifies that documented behaviour.
void ApiBufferProbe::measureSleepResolution()
{
  // Deliberately NOT all multiples of a round number: 1100/1333/1750 are the
  // values that distinguish "honours the request" from "quantises to 250/500 us
  // and happens to agree on the round ones". 250 probes the sub-cycle floor.
  const unsigned int test_us[] = {250, 750, 1000, 1100, 1333, 1500, 1750, 2500, 10000, 25000};

  RCLCPP_INFO(this->get_logger(), "--- Measurement 2: USleep resolution ---");
  RCLCPP_INFO(
    this->get_logger(), "  %10s %10s %12s %12s %10s",
    "req [us]", "blocks", "expect [ms]", "actual [ms]", "per-blk [us]");

  for (unsigned int t : test_us) {
    if (!resetChannel()) {
      return;
    }

    int recorded = 0;
    bool record_ok = true;
    for (int i = 0; i < sleepSampleCount_ && record_ok; ++i) {
      record_ok = check(abRec_->StartRecordBufferChannel(channel_), "StartRecordBufferChannel");
      if (record_ok && abRec_->USleep(t) == ErrorCode::None) {
        ++recorded;
      }
      abRec_->EndRecordBufferChannel();
    }
    if (!record_ok || recorded == 0) {
      RCLCPP_WARN(this->get_logger(), "  %10u  (recording failed)", t);
      continue;
    }

    // Trailing dummy so remainingBlockCount==0 means the last real sleep ended.
    abRec_->StartRecordBufferChannel(channel_);
    abRec_->USleep(0);
    abRec_->EndRecordBufferChannel();

    const auto t0 = std::chrono::steady_clock::now();
    if (!check(abCtl_->Execute(channel_), "ApiBuffer::Execute")) {
      return;
    }
    if (!drain(nullptr)) {
      return;
    }
    const auto t1 = std::chrono::steady_clock::now();

    const double actual_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
    const double expect_ms = recorded * t / 1000.0;
    const double per_block_us = (actual_ms * 1000.0) / recorded;

    RCLCPP_INFO(
      this->get_logger(), "  %10u %10d %12.2f %12.2f %10.1f",
      t, recorded, expect_ms, actual_ms, per_block_us);

    abCtl_->Halt(channel_);
  }

  resetChannel();
  RCLCPP_INFO(
    this->get_logger(),
    "  NOTE: Execute blocks ~1ms and GetStatus is 1 cycle stale, so short totals "
    "carry ~1-2ms of fixed overhead.");
}

// --- Measurement 3: block accounting ---------------------------------------
// servo_stream_controller cannot use GetCumulativeBlockCount (absent from this
// SDK), so it tracks blocks added with a local counter and relies on that
// matching errorLog[].execBlockNumber. This checks the counters move as assumed.
void ApiBufferProbe::measureBlockAccounting()
{
  RCLCPP_INFO(this->get_logger(), "--- Measurement 3: block accounting ---");

  if (!resetChannel()) {
    return;
  }

  constexpr int kSetpoints = 5;
  int localBlocks = 0;

  for (int i = 0; i < kSetpoints; ++i) {
    abRec_->StartRecordBufferChannel(channel_);
    abRec_->USleep(2000);
    abRec_->USleep(2000);   // two blocks per "setpoint", as the controller adds
    abRec_->EndRecordBufferChannel();
    localBlocks += 2;
  }

  ApiBufferStatus st;
  if (!settle(&st)) {
    return;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "  after recording: local=%d blockCount=%lld remaining=%lld exec=%lld",
    localBlocks, st.blockCount, st.remainingBlockCount, st.execBlockCount);
  if (st.blockCount != localBlocks || st.remainingBlockCount != localBlocks) {
    RCLCPP_WARN(
      this->get_logger(),
      "  MISMATCH: local counter does not track blockCount — revisit error correlation");
  }

  abRec_->StartRecordBufferChannel(channel_);
  abRec_->USleep(0);        // trailing dummy, see measurement 2
  abRec_->EndRecordBufferChannel();

  if (!check(abCtl_->Execute(channel_), "ApiBuffer::Execute") || !drain(nullptr)) {
    return;
  }
  abCtl_->Halt(channel_);

  if (!settle(&st)) {
    return;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "  after draining:  local=%d blockCount=%lld remaining=%lld exec=%lld errors=%lld",
    localBlocks, st.blockCount, st.remainingBlockCount, st.execBlockCount, st.errorCount);

  resetChannel();
}

void ApiBufferProbe::runProbe()
{
  if (!attachDevice(&wmxRec_, "api_buffer_probe_rec") ||
    !attachDevice(&wmxCtl_, "api_buffer_probe_ctl"))
  {
    rclcpp::shutdown();
    return;
  }
  recAttached_ = true;
  ctlAttached_ = true;

  cmRec_ = std::make_unique<CoreMotion>(&wmxRec_);
  cmCtl_ = std::make_unique<CoreMotion>(&wmxCtl_);
  abRec_ = std::make_unique<ApiBuffer>(&wmxRec_);
  abCtl_ = std::make_unique<ApiBuffer>(&wmxCtl_);

  int major = 0, minor = 0, revision = 0, fix = 0;
  ApiBuffer::GetLibVersion(&major, &minor, &revision, &fix);
  RCLCPP_INFO(
    this->get_logger(), "ApiBuffer lib version: %d.%d.%d.%d", major, minor, revision, fix);

  if (!check(
      abCtl_->CreateApiBuffer(channel_, bufferSizeMb_, SizeUnit::Megabyte),
      "CreateApiBuffer"))
  {
    rclcpp::shutdown();
    return;
  }
  bufferCreated_ = true;

  // stopOnLastBlock=false matches how servo_stream_controller will run the
  // channel: it stays Active and picks up blocks as they are added.
  ApiBufferOptions opt;
  opt.stopOnError = true;
  opt.autoRewind = false;
  opt.stopOnLastBlock = false;
  check(abCtl_->SetOptions(channel_, &opt), "SetOptions");

  RCLCPP_INFO(this->get_logger(), "--- Measurement 1: block sizes ---");
  int motion_bytes = 0, sleep_bytes = 0, buffer_size = 0;
  const bool size_ok = measureBlockSize(&motion_bytes, &sleep_bytes, &buffer_size);

  measureSleepResolution();
  measureBlockAccounting();

  // ---- report ----
  const int requested = bufferSizeMb_ * 1024 * 1024;
  RCLCPP_INFO(this->get_logger(), "==================== PROBE REPORT ====================");
  RCLCPP_INFO(
    this->get_logger(), " SDK ApiBuffer lib   : %d.%d.%d.%d", major, minor, revision, fix);
  RCLCPP_INFO(
    this->get_logger(), " buffer requested    : %d bytes (%d MB)", requested, bufferSizeMb_);
  RCLCPP_INFO(
    this->get_logger(), " buffer usable       : %d bytes (delta %d)",
    buffer_size, requested - buffer_size);

  if (size_ok) {
    const int per_setpoint = motion_bytes + sleep_bytes;
    RCLCPP_INFO(this->get_logger(), " StartLinearIntplPos : %d bytes/block", motion_bytes);
    RCLCPP_INFO(this->get_logger(), " USleep              : %d bytes/block", sleep_bytes);
    RCLCPP_INFO(this->get_logger(), " per setpoint        : %d bytes", per_setpoint);
    if (per_setpoint > 0) {
      // Servo runs at 40 Hz (publish_period 0.025).
      const double bytes_per_sec = per_setpoint * 40.0;
      RCLCPP_INFO(
        this->get_logger(), " at 40 Hz            : %.1f KB/s, %d MB holds %.1f s of stream",
        bytes_per_sec / 1024.0, bufferSizeMb_, buffer_size / bytes_per_sec);
      RCLCPP_INFO(
        this->get_logger(), " capacity            : %d setpoints resident",
        buffer_size / per_setpoint);
    }
  } else {
    RCLCPP_WARN(this->get_logger(), " block size measurement FAILED — see errors above");
  }
  RCLCPP_INFO(this->get_logger(), "======================================================");

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApiBufferProbe>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
