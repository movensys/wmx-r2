// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// servo_stream_controller
//
// Streams MoveIt Servo joint commands into the WMX3 API buffer, which replays
// them under the real-time OS at communication-cycle boundaries.
//
// This is the buffered alternative to joint_position_controller. Both issue one
// StartLinearIntplPos per setpoint from the ROS subscription callback; what this
// node adds is the API buffer, whose watch function is a hardware-side safety
// net and whose blocks execute under the RT OS at communication-cycle
// boundaries. Recording is also non-blocking with respect to motion start,
// where the direct node's call inherits APIWaitUntilMotionStart.
//
// The two nodes subscribe to the same topic and MUST NOT run together.
//
//   Servo topic (QoS depth 1, latest-wins) -> callback -> API buffer -> engine
//
// Each setpoint becomes ONE block:
//   StartLinearIntplPos(6 axes)   coordinated, lands as a FastBlending override
//
// There is no pacing block. The WMX3 manual is explicit that buffered API
// functions "will be executed as fast as possible" unless a Wait or Sleep is
// recorded between them, and that motion APIs go at one per communication
// cycle. So the channel drains at ~1 ms per setpoint against a 25 ms arrival
// interval, and remainingBlockCount sits at 0-1 in steady state. The API buffer
// is a real-time mailbox here, not a queue: it still moves the hand-off onto
// the RT side and still carries the watch trigger, but it no longer stores
// latency.
//
// That is deliberate. The earlier design recorded USleep(T) after each
// interpolation, which held the channel for the whole period -- so every
// segment ran to its target, decelerated to rest under endVelocity = 0, and sat
// there until the sleep expired. FastBlending never engaged. The motion came
// out stop-and-go (measured: ~8x peak velocity, ~24 velocity sign changes per
// second), and the pacing loop could only cover the per-block setup cost by
// holding a standing depth error of ~7 setpoints, which cost 225-250 ms of
// latency instead of the intended 50 ms. Dropping the sleep lets each command
// override its predecessor mid-flight, which is what FastBlending is for.
//
// The pump thread went the same way, and for the same reason. With depth pinned
// at 0-1 the buffer stores no latency, so a pump could not absorb ROS jitter --
// it could only add its own: a condition-variable wakeup on a non-RT thread and
// a cyclic GetStatus per setpoint, inserted into a path whose entire error
// budget is one accel ramp (accel_ratio * period). Recording straight from the
// subscription callback removes both. Latest-wins still holds, one level down:
// the subscription is QoS depth 1, so an unread setpoint is overwritten by the
// newer one in the middleware instead of in a ring of our own.
//
// Ride-through no longer comes from queue depth. Each block carries a full
// period of programmed travel, so a late setpoint continues the previous segment
// rather than leaving a gap.
//
// What a dead publisher does NOT do any more is coast to a stop on its own. That
// was true while endVelocity was 0; with end_velocity_ratio > 0 the final
// segment ends in a velocity STEP, so stopping the arm is now the starvation
// timer's job and it has a real deadline -- setRosParameter checks that the
// timeout lands before the segment completes, and says so at startup.
//
// Cost is 320 bytes per setpoint, so a 1 MB channel holds ~3200. The channel is
// a ring and reclaims space as blocks execute, so stream length is not bounded
// by buffer size.
//
// Notes on the motion model:
//   - LinearIntplOverrideType is FastBlending, so consecutive interpolations
//     blend rather than stop-and-go.
//   - profile.startingVelocity = 0 lets the override inherit the current
//     velocity; continuity comes from the blend. endVelocity is NOT 0: a segment
//     planning a decel to rest puts that decel exactly where the next setpoint
//     lands, and an override issued during the final deceleration segment does
//     not blend at all (see recordSetpoint). The arm is brought to rest by the
//     starvation timer instead.
//   - LinearIntplProfileCalcMode is read from axis[0], so the axis array is
//     always filled in joint_axes order. That ordering is semantics, not style.
//   - Segment distance is measured from the CURRENT COMMANDED POSITION, not
//     from the previous setpoint. That is the reverse of the buffered design,
//     and it follows from the queue being gone. With commands overriding
//     mid-flight, posCmd never reaches the previous target, so a setpoint-to-
//     setpoint delta understates the distance still to cover and the lag grows
//     without bound -- nothing in the loop would ever notice. Measuring from
//     posCmd is self-correcting: the further behind the axis is, the higher the
//     velocity it asks for. Depth 0-1 is what makes posCmd trustworthy again;
//     it is one block plus one cycle stale, ~1-2 ms.
//   - The subscription is QoS depth 1, so only the newest setpoint survives.
//     Raising it would hand the callback a backlog, each entry recorded and
//     overriding the last, leaving the survivor to cover the whole backlog's
//     distance in one period -- a velocity spike proportional to the backlog.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "ApiBufferApi.h"

using std::placeholders::_1;
using wmx3Api::ApiBuffer;
using wmx3Api::ApiBufferOptions;
using wmx3Api::ApiBufferState;
using wmx3Api::ApiBufferStatus;
using wmx3Api::ApiBufferWatch;
using wmx3Api::AxisSelection;
using wmx3Api::CoreMotion;
using wmx3Api::Config;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::Motion;
using wmx3Api::ProfileType;
using wmx3Api::SizeUnit;
using wmx3Api::WMX3Api;

namespace
{
// remainingBlockCount should sit at 0-1 with no pacing block, so anything
// standing above this means the engine is not keeping up with arrivals.
constexpr int kDepthWarnBlocks = 3;

// Consecutive status polls with cumulativeBlockCount unchanged while streaming
// before the engine is called stalled. Polls are 100 ms and arrivals are 40 Hz,
// so a healthy poll sees ~4 blocks consumed and three empty ones is not noise.
constexpr int kStalledPollCount = 3;
constexpr auto kStatusPollPeriod = std::chrono::milliseconds(100);   // ~10 Hz

// Starvation is checked on its own timer rather than with the status poll,
// because once end_velocity_ratio > 0 it is a safety deadline and not just
// bookkeeping: a segment that completes drops the axes from speed to zero in one
// step, and the controlled stop has to land first. The callback does nothing but
// compare two timestamps -- no engine call -- so it is cheap enough to run here.
constexpr auto kStarvationPollPeriod = std::chrono::milliseconds(10);

// Backoff after the motion channel stops on a fault. Without this the callback
// re-Executes on the very next setpoint, so a PERSISTENT fault (servo off, for
// one) becomes a 40 Hz Clear/Execute/Stop storm against the engine that never
// gives up. Doubles per consecutive fault, capped, and resets the moment a
// setpoint records successfully — so a transient trip still recovers promptly.
constexpr auto kFaultBackoffBase = std::chrono::milliseconds(500);
constexpr auto kFaultBackoffMax = std::chrono::milliseconds(5000);
constexpr int kFaultBackoffMaxShift = 4;

// How long a stream must survive before the fault history is forgiven. A
// recorded block is NOT evidence of health: recordSetpoint only writes into the
// buffer and succeeds even when every block is failing at execution time. Only
// elapsed time with the channel still Active proves the engine is consuming.
constexpr auto kHealthyStreamTime = std::chrono::seconds(1);

// A Stop state read within this window of Execute is not trusted. GetStatus is
// cyclic, so the first poll after Execute can still be reporting the Stop that
// the preceding Halt/Clear left behind. Long enough to cover several engine
// cycles and the 25 ms setpoint cadence, short enough that a genuine
// error-free Stop (an external Halt, say) is still caught promptly.
constexpr auto kStatusSettleTime = std::chrono::milliseconds(50);
}  // namespace

class ServoStreamController : public rclcpp::Node
{
public:
  ServoStreamController();
  ~ServoStreamController();

private:
  // --- WMX3 handles -------------------------------------------------------
  // Two devices. StartRecordBufferChannel is DEVICE-wide: while recording,
  // every WMX3 call through that device is recorded instead of executed. The
  // control path needs CoreMotion::GetStatus and Motion::Stop (both of which
  // WOULD be recorded), so the recording device is kept strictly separate.
  // Nothing on the ctl device may be called inside a record window.
  WMX3Api wmxRec_;
  WMX3Api wmxCtl_;
  std::unique_ptr<CoreMotion> cmRec_;
  std::unique_ptr<CoreMotion> cmCtl_;
  std::unique_ptr<ApiBuffer> abRec_;
  std::unique_ptr<ApiBuffer> abCtl_;

  bool recAttached_ = false;
  bool ctlAttached_ = false;
  bool bufferCreated_ = false;
  bool initialized_ = false;

  int err_ = 0;
  char errString_[256];

  // --- parameters ---------------------------------------------------------
  std::vector<int64_t> jointAxes_;
  std::vector<std::string> jointNames_;
  std::string jointTrajectoryTopic_;
  int channelMotion_ = 0;
  int channelEstop_ = 1;
  int bufferSizeMb_ = 5;
  int nominalPeriodUs_ = 25000;      // time each segment is given to cover its step
  int starvationTimeoutMs_ = 75;
  double accelRatio_ = 0.3;
  double maxJointVelocity_ = 3.0;    // rad/s ceiling, applied as a uniform scale
  double minStepRad_ = 1e-6;         // below this the setpoint is a hold, not a move
  double endVelocityRatio_ = 1.0;    // endVelocity as a fraction of path velocity
  bool stopOnError_ = true;
  int setpointLogEvery_ = 0;         // 0 = off; N = trace every Nth recorded setpoint

  std::map<std::string, int> axisByName_;
  AxisSelection axisSel_;
  Motion::LinearIntplCommand intpl_;

  // --- streaming state ----------------------------------------------------
  // Everything here is touched only from the executor thread: the trajectory
  // callback, onExecActive and the status timer all run on it, and rclcpp::spin
  // is single-threaded. That is what the pump's removal bought -- no ring, no
  // mutex, no atomics on the streaming path.
  std::vector<double> step_;                  // scratch: target - posCmd, per axis
  uint64_t setpointSeq_ = 0;                  // recorded setpoints, for the trace log

  bool streaming_ = false;                    // buffer Active and being fed

  // Last setpoint HANDLED, holds included -- a hold is a deliberate no-record,
  // not a gap in the stream, so it must keep starvation quiet.
  std::chrono::steady_clock::time_point lastSetpointAt_{};

  int64_t lastErrorCount_ = 0;               // mirrors ApiBufferStatus::errorCount
  int64_t overrideRejects_ = 0;              // refused overrides, a floor (log samples 10/poll)

  // Liveness. remainingBlockCount is 0-1 by design now, so depth no longer
  // shows whether the engine is consuming; the cumulative count does.
  int64_t lastCumulativeBlocks_ = 0;
  uint64_t lastSeqSeen_ = 0;                 // setpointSeq_ at the previous poll
  int stalledPolls_ = 0;

  int consecutiveFaults_ = 0;                       // channel stops without progress
  std::chrono::steady_clock::time_point faultUntil_{};   // suppress restarts until
  std::chrono::steady_clock::time_point streamStartedAt_{};

  bool inExecution_ = false;                  // move_group owns the arm

  // Atomic: written by the executor thread in onEngineReady, read by the init
  // thread. The only cross-thread flag left.
  std::atomic<bool> initializing_{false};

  std::thread initThread_;

  // Engine readiness is polled through wmx/engine/get_engine_status, the same
  // way wmx_lifecycle_manager_node does it. engineProbePending_ is only touched
  // on the executor thread (timer callback and service response).
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr engineStatusClient_;
  rclcpp::TimerBase::SharedPtr engineProbeTimer_;
  bool engineProbePending_ = false;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execActiveSub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr jointTrajectorySub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr depthPub_;
  rclcpp::TimerBase::SharedPtr statusTimer_;
  rclcpp::TimerBase::SharedPtr starvationTimer_;

  // --- helpers ------------------------------------------------------------
  void setRosParameter();
  void probeEngine();
  void onEngineReady();
  void onExecActive(const std_msgs::msg::Bool::SharedPtr msg);
  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void runInitSequence();
  bool attachDevice(WMX3Api * lib, const char * name);
  bool createBuffer(int channel, int size_mb, const char * what);
  void teardownPartialInit();
  bool check(int err, const char * what);
  void blockErrorToString(int code, char * buf, size_t size);
  bool setupBuffers();
  void recordEstopRoutine();

  bool beginStream();
  void endStream(const char * reason, bool controlled_stop);
  bool recordSetpoint(const std::vector<double> & target);
  void logSetpoint(const CoreMotionStatus & before, double scale);
  void logMotionParams();
  void pollStatus();
  void checkStarvation();
};

ServoStreamController::ServoStreamController()
: Node("servo_stream_controller")
{
  setRosParameter();

  depthPub_ = this->create_publisher<std_msgs::msg::Int32>("~/queue_depth", 10);

  engineStatusClient_ =
    this->create_client<std_srvs::srv::Trigger>("wmx/engine/get_engine_status");
  engineProbeTimer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&ServoStreamController::probeEngine, this));

  execActiveSub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local(),
    std::bind(&ServoStreamController::onExecActive, this, _1));

  RCLCPP_INFO(this->get_logger(), "servo_stream_controller waiting for engine...");
}

ServoStreamController::~ServoStreamController()
{
  if (initThread_.joinable()) {
    initThread_.join();
  }

  if (initialized_) {
    // Stop issuing blocks, drop what is queued, then bring the axes to rest
    // with the configured stop profile. This is endStream's teardown and more,
    // so there is nothing to gain by routing through it on the way out.
    if (abCtl_) {
      abCtl_->Halt(channelMotion_);
      abCtl_->Clear(channelMotion_);
    }
    if (cmCtl_) {
      cmCtl_->motion->Stop(&axisSel_);
      // Bounded. The no-timeout Wait can block indefinitely if an axis never
      // reaches rest (servo off, alarm), and a shutdown that hangs here never
      // reaches FreeApiBuffer below — leaving the channels allocated in the
      // engine so the NEXT run fails with "Queue ID is already used".
      cmCtl_->motion->Wait(&axisSel_, 2000);
    }
    if (abCtl_ && bufferCreated_) {
      abCtl_->FreeApiBuffer(channelMotion_);
      abCtl_->FreeApiBuffer(channelEstop_);
    }
  }

  if (recAttached_) {
    wmxRec_.CloseDevice();
  }
  if (ctlAttached_) {
    wmxCtl_.CloseDevice();
  }

  RCLCPP_INFO(this->get_logger(), "servo_stream_controller is stopped");
}

void ServoStreamController::setRosParameter()
{
  this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  this->declare_parameter<std::vector<std::string>>(
    "joint_name", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
  this->declare_parameter<std::string>(
    "joint_trajectory_topic", "/joint_trajectory_topic/no_param");
  this->declare_parameter<int>("api_buffer_channel", 0);
  this->declare_parameter<int>("estop_buffer_channel", 1);
  this->declare_parameter<int>("api_buffer_size_mb", 5);
  this->declare_parameter<int>("nominal_period_us", 25000);
  this->declare_parameter<int>("starvation_timeout_ms", 75);
  this->declare_parameter<double>("accel_ratio", 0.3);
  this->declare_parameter<double>("max_joint_velocity", 3.0);
  this->declare_parameter<double>("min_step_rad", 1e-6);
  this->declare_parameter<double>("end_velocity_ratio", 1.0);
  this->declare_parameter<bool>("stop_on_error", true);
  this->declare_parameter<int>("setpoint_log_every", 0);

  this->get_parameter("joint_axes", jointAxes_);
  this->get_parameter("joint_name", jointNames_);
  this->get_parameter("joint_trajectory_topic", jointTrajectoryTopic_);
  this->get_parameter("api_buffer_channel", channelMotion_);
  this->get_parameter("estop_buffer_channel", channelEstop_);
  this->get_parameter("api_buffer_size_mb", bufferSizeMb_);
  this->get_parameter("nominal_period_us", nominalPeriodUs_);
  this->get_parameter("starvation_timeout_ms", starvationTimeoutMs_);
  this->get_parameter("accel_ratio", accelRatio_);
  this->get_parameter("max_joint_velocity", maxJointVelocity_);
  this->get_parameter("min_step_rad", minStepRad_);
  this->get_parameter("end_velocity_ratio", endVelocityRatio_);
  this->get_parameter("stop_on_error", stopOnError_);
  this->get_parameter("setpoint_log_every", setpointLogEvery_);

  for (size_t i = 0; i < jointNames_.size() && i < jointAxes_.size(); ++i) {
    axisByName_[jointNames_[i]] = static_cast<int>(jointAxes_[i]);
  }

  // Fixed order: axis[0] determines LinearIntplProfileCalcMode.
  axisSel_.axisCount = jointAxes_.size();
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    axisSel_.axis[i] = static_cast<int>(jointAxes_[i]);
  }

  step_.assign(jointAxes_.size(), 0.0);

  // These three feed a division or a velocity directly, so a bad value is a
  // divide-by-zero or an unbounded command rather than a mis-tuned stream.
  if (accelRatio_ <= 0.0) {
    RCLCPP_ERROR(
      this->get_logger(), "accel_ratio must be > 0 (got %.3f); using 0.3", accelRatio_);
    accelRatio_ = 0.3;
  }
  if (nominalPeriodUs_ <= 0) {
    RCLCPP_ERROR(
      this->get_logger(), "nominal_period_us must be > 0 (got %d); using 25000", nominalPeriodUs_);
    nominalPeriodUs_ = 25000;
  }
  if (maxJointVelocity_ <= 0.0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "max_joint_velocity must be > 0 (got %.3f); using 3.0", maxJointVelocity_);
    maxJointVelocity_ = 3.0;
  }
  if (endVelocityRatio_ < 0.0 || endVelocityRatio_ > 1.0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "end_velocity_ratio must be in [0, 1] (got %.3f); using 1.0", endVelocityRatio_);
    endVelocityRatio_ = 1.0;
  }

  // A segment runs for T*(1 + r*(1 - e + e*e/2)) before it completes: accel r*T,
  // cruise, then a decel from the path velocity down to e times it. At e = 1
  // that last phase vanishes and the segment simply ends at speed.
  //
  // Completion is the dangerous moment once e > 0, because the engine drops the
  // axis from endVelocity to 0 in one step rather than ramping. The starvation
  // timer has to get there first with a controlled stop, so its timeout must sit
  // BELOW the completion time -- and above the worst-case gap between setpoints,
  // or it will stop the arm mid-stream.
  if (endVelocityRatio_ > 0.0) {
    const double e = endVelocityRatio_;
    const double completion_ms = (nominalPeriodUs_ / 1000.0) *
      (1.0 + accelRatio_ * (1.0 - e + 0.5 * e * e));
    if (starvationTimeoutMs_ >= completion_ms) {
      RCLCPP_ERROR(
        this->get_logger(),
        "starvation_timeout_ms %d is at or past the %.1f ms segment completion time; "
        "with end_velocity_ratio %.2f the engine will step the axes from speed to zero "
        "before the controlled stop runs. Lower the timeout or raise nominal_period_us.",
        starvationTimeoutMs_, completion_ms, e);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "segment completes at %.1f ms; controlled stop armed at %d ms",
        completion_ms, starvationTimeoutMs_);
    }
  }

  if (channelMotion_ == channelEstop_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "estop_buffer_channel must differ from api_buffer_channel; watch trigger disabled");
  }

  std::string axes_str;
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    if (i > 0) {axes_str += ", ";}
    axes_str += std::to_string(jointAxes_[i]);
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", axes_str.c_str());
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_topic: %s", jointTrajectoryTopic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "api_buffer_channel: %d, estop_buffer_channel: %d",
    channelMotion_, channelEstop_);
  RCLCPP_INFO(this->get_logger(), "api_buffer_size_mb: %d", bufferSizeMb_);
  RCLCPP_INFO(
    this->get_logger(), "nominal_period_us: %d (time given to each segment)", nominalPeriodUs_);
  RCLCPP_INFO(this->get_logger(), "starvation_timeout_ms: %d", starvationTimeoutMs_);
  RCLCPP_INFO(this->get_logger(), "accel_ratio: %.2f", accelRatio_);
  RCLCPP_INFO(this->get_logger(), "max_joint_velocity: %.3f rad/s", maxJointVelocity_);
  RCLCPP_INFO(this->get_logger(), "min_step_rad: %.2e", minStepRad_);
  RCLCPP_INFO(
    this->get_logger(), "end_velocity_ratio: %.2f (%s)", endVelocityRatio_,
    endVelocityRatio_ > 0.0 ? "carry velocity" : "decelerate to rest each segment");
  RCLCPP_INFO(this->get_logger(), "stop_on_error: %s", stopOnError_ ? "true" : "false");
  RCLCPP_INFO(
    this->get_logger(), "setpoint_log_every: %d (%s)", setpointLogEvery_,
    setpointLogEvery_ > 0 ? "per-setpoint trace ON" : "off");
  RCLCPP_INFO(this->get_logger(), "===========================");
}

bool ServoStreamController::check(int err, const char * what)
{
  if (err == ErrorCode::None) {
    return true;
  }
  err_ = err;
  ApiBuffer::ErrorToString(err, errString_, sizeof(errString_));
  RCLCPP_ERROR(this->get_logger(), "%s failed. Error=%d (%s)", what, err, errString_);
  return false;
}

// A block error comes from the recorded command itself (StartLinearIntplPos),
// so it carries a CoreMotion error code, NOT an ApiBuffer one. Decoding it with
// ApiBuffer::ErrorToString yields an empty string — on the bench, code 65556
// printed as "()". Try CoreMotion first, fall back to ApiBuffer, then admit
// defeat rather than printing an empty pair of brackets.
void ServoStreamController::blockErrorToString(int code, char * buf, size_t size)
{
  const auto n = static_cast<unsigned int>(size);
  buf[0] = '\0';
  CoreMotion::ErrorToString(code, buf, n);
  if (buf[0] == '\0') {
    ApiBuffer::ErrorToString(code, buf, n);
  }
  if (buf[0] == '\0') {
    snprintf(buf, size, "unrecognised code");
  }
}

void ServoStreamController::probeEngine()
{
  if (initialized_) {
    engineProbeTimer_->cancel();
    return;
  }
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

void ServoStreamController::onEngineReady()
{
  if (initialized_ || initializing_.exchange(true)) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Engine ready — starting init on dedicated thread...");
  if (initThread_.joinable()) {
    initThread_.join();
  }
  initThread_ = std::thread(&ServoStreamController::runInitSequence, this);
}

bool ServoStreamController::attachDevice(WMX3Api * lib, const char * name)
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
        this->get_logger(), "Failed to attach device '%s'. Error=%d (%s)",
        name, err, errString_);
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

// The watch trigger routine: a pre-recorded Stop on every joint axis, living in
// its own channel. If any watched axis goes servo-off, offline, amp-alarm or
// hits a limit, the engine halts the motion channel and runs this — entirely
// inside the RT side, with no dependency on ROS or Linux still being alive.
void ServoStreamController::recordEstopRoutine()
{
  if (channelMotion_ == channelEstop_) {
    return;
  }
  abRec_->StartRecordBufferChannel(channelEstop_);
  cmRec_->motion->Stop(&axisSel_);
  abRec_->EndRecordBufferChannel();
  RCLCPP_INFO(this->get_logger(), "Recorded e-stop routine into channel %d", channelEstop_);
}

// API buffers are engine-side and outlive the client process, so a controller
// that died without freeing them leaves the channel allocated and the next run
// gets "Queue ID is already used". Reclaim the channel instead of making the
// operator restart the whole engine.
bool ServoStreamController::createBuffer(int channel, int size_mb, const char * what)
{
  int err = abCtl_->CreateApiBuffer(channel, size_mb, SizeUnit::Megabyte);
  if (err == ErrorCode::None) {
    return true;
  }

  ApiBuffer::ErrorToString(err, errString_, sizeof(errString_));
  RCLCPP_WARN(
    this->get_logger(),
    "%s failed (Error=%d %s); channel %d appears stale, reclaiming",
    what, err, errString_, channel);

  abCtl_->Halt(channel);
  abCtl_->Clear(channel);
  abCtl_->FreeApiBuffer(channel);

  return check(abCtl_->CreateApiBuffer(channel, size_mb, SizeUnit::Megabyte), what);
}

// Undo whatever runInitSequence managed before failing. Without this, a failure
// part-way through leaks engine-side buffers AND device handles, and the 1 Hz
// engine-ready timer then retries forever — leaking two more devices a second.
void ServoStreamController::teardownPartialInit()
{
  if (abCtl_) {
    // Unconditional: bufferCreated_ is only set once BOTH channels exist, so it
    // cannot tell us whether the first one was already allocated. Freeing an
    // unallocated channel is harmless.
    abCtl_->Halt(channelMotion_);
    abCtl_->Clear(channelMotion_);
    abCtl_->FreeApiBuffer(channelMotion_);
    if (channelMotion_ != channelEstop_) {
      abCtl_->FreeApiBuffer(channelEstop_);
    }
  }
  bufferCreated_ = false;

  abRec_.reset();
  abCtl_.reset();
  cmRec_.reset();
  cmCtl_.reset();

  if (recAttached_) {
    wmxRec_.CloseDevice();
    recAttached_ = false;
  }
  if (ctlAttached_) {
    wmxCtl_.CloseDevice();
    ctlAttached_ = false;
  }
}

bool ServoStreamController::setupBuffers()
{
  if (!createBuffer(channelMotion_, bufferSizeMb_, "CreateApiBuffer(motion)")) {
    return false;
  }
  if (channelMotion_ != channelEstop_) {
    if (!createBuffer(channelEstop_, 1, "CreateApiBuffer(estop)")) {
      return false;
    }
  }
  bufferCreated_ = true;

  recordEstopRoutine();

  ApiBufferOptions opt;
  opt.stopOnError = stopOnError_;
  opt.autoRewind = false;      // the ring wraps within seconds; rewind is unusable here
  opt.stopOnLastBlock = false;  // MUST be false: an Active channel picks up new blocks
  if (!check(abCtl_->SetOptions(channelMotion_, &opt), "SetOptions")) {
    return false;
  }

  if (channelMotion_ != channelEstop_) {
    ApiBufferWatch watch;
    watch.enableWatch = true;
    watch.watchAxes = axisSel_;
    watch.enableWatchTriggerRoutine = true;
    watch.watchTriggerRoutineChannel = channelEstop_;
    if (!check(abCtl_->SetWatch(channelMotion_, &watch), "SetWatch")) {
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(), "Watch enabled on %u axes, trigger routine on channel %d",
      axisSel_.axisCount, channelEstop_);
  }

  return true;
}

// Neither of these is set by this node — both are inherited from the axis
// configuration — and both decide whether the motion model in this file's
// header actually holds. linearIntplProfileCalcMode governs whether the
// per-axis velocity limits recordSetpoint computes are in force at all;
// linearIntplOverrideType decides whether consecutive setpoints blend or
// stop-and-go, which is the whole premise of dropping the pacing block. Read
// them once and put them in the log so a run can be diagnosed after the fact.
void ServoStreamController::logMotionParams()
{
  if (jointAxes_.empty()) {
    return;
  }

  // SystemParam carries per-axis arrays for all 128 axes; too big for the stack.
  auto param = std::make_unique<Config::SystemParam>();
  if (cmCtl_->config->GetParam(param.get()) != ErrorCode::None) {
    RCLCPP_WARN(this->get_logger(), "Could not read motion params; calc mode unknown");
    return;
  }

  static const char * const kCalcMode[] = {"AxisLimit", "MatchSlowestAxis", "MatchFarthestAxis"};
  static const char * const kOverride[] = {"Smoothing", "Blending", "FastBlending"};
  const auto name = [](const char * const * table, int size, int value) {
      return (value >= 0 && value < size) ? table[value] : "unknown";
    };

  const auto & mp = param->motionParam[jointAxes_[0]];
  RCLCPP_INFO(
    this->get_logger(),
    "axis %d motion params: calcMode=%s, overrideType=%s, apiWaitUntilMotionStart=%s",
    static_cast<int>(jointAxes_[0]),
    name(kCalcMode, 3, mp.linearIntplProfileCalcMode),
    name(kOverride, 3, mp.linearIntplOverrideType),
    mp.apiWaitUntilMotionStart ? "true" : "false");

  if (mp.linearIntplOverrideType == Config::LinearIntplOverrideType::Smoothing) {
    RCLCPP_WARN(
      this->get_logger(),
      "linearIntplOverrideType is Smoothing; streaming needs Blending or FastBlending "
      "for consecutive setpoints to override rather than run to completion");
  }
  if (mp.linearIntplProfileCalcMode != Config::LinearIntplProfileCalcMode::AxisLimit) {
    RCLCPP_WARN(
      this->get_logger(),
      "linearIntplProfileCalcMode is %s, not AxisLimit; the per-axis velocity and "
      "acceleration limits this node computes may not govern the profile",
      name(kCalcMode, 3, mp.linearIntplProfileCalcMode));
  }
}

void ServoStreamController::runInitSequence()
{
  // Flags are set per-device, not after both: if the second attach fails the
  // first device still needs closing, or the retry leaks it.
  if (!attachDevice(&wmxRec_, "servo_stream_rec")) {
    teardownPartialInit();
    initializing_ = false;
    return;
  }
  recAttached_ = true;

  if (!attachDevice(&wmxCtl_, "servo_stream_ctl")) {
    teardownPartialInit();
    initializing_ = false;
    return;
  }
  ctlAttached_ = true;

  cmRec_ = std::make_unique<CoreMotion>(&wmxRec_);
  cmCtl_ = std::make_unique<CoreMotion>(&wmxCtl_);
  abRec_ = std::make_unique<ApiBuffer>(&wmxRec_);
  abCtl_ = std::make_unique<ApiBuffer>(&wmxCtl_);

  if (!setupBuffers()) {
    teardownPartialInit();
    initializing_ = false;
    return;
  }

  logMotionParams();

  statusTimer_ = this->create_wall_timer(
    kStatusPollPeriod, std::bind(&ServoStreamController::pollStatus, this));

  starvationTimer_ = this->create_wall_timer(
    kStarvationPollPeriod, std::bind(&ServoStreamController::checkStarvation, this));

  // Before the subscription, not after: the callback drops setpoints until this
  // is set, and the subscription can start delivering the moment it exists.
  initialized_ = true;

  // Depth 1 IS the drop policy now -- see the note at the top of this file.
  // Raising it hands the callback a backlog and buys a velocity spike.
  jointTrajectorySub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    jointTrajectoryTopic_, 1,
    std::bind(&ServoStreamController::jointTrajectoryCallback, this, _1));

  RCLCPP_INFO(this->get_logger(), "servo_stream_controller is ready");
}

void ServoStreamController::onExecActive(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (inExecution_ == msg->data) {
    return;
  }
  inExecution_ = msg->data;

  RCLCPP_INFO(
    this->get_logger(), "Servo stream %s (move_group execution %s)",
    msg->data ? "blocked" : "allowed", msg->data ? "started" : "finished");

  if (msg->data) {
    // move_group is about to move the arm somewhere else. Stop the channel; the
    // callback discards setpoints for as long as inExecution_ holds, so there
    // is nothing else to drop. No Motion::Stop here — the trajectory controller
    // owns the axes from this point.
    endStream("move_group execution started", false);
  }
}

void ServoStreamController::jointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->points.empty() || inExecution_ || !initialized_) {
    return;
  }

  const auto & pt = msg->points.back();
  const size_t count = jointAxes_.size();

  std::vector<double> target(count, 0.0);
  std::vector<bool> filled(count, false);

  if (msg->joint_names.empty()) {
    // Unnamed: positional, assumed already in joint_axes order.
    if (pt.positions.size() < count) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dropped trajectory: %zu positions for %zu axes", pt.positions.size(), count);
      return;
    }
    for (size_t i = 0; i < count; ++i) {
      target[i] = pt.positions[i];
      filled[i] = true;
    }
  } else {
    // Iterate the MESSAGE's joints and pick out the ones this node drives.
    // Doing it this way round lets a subset configuration (e.g. joint_axes: [0]
    // for single-axis bring-up) consume Servo's full six-joint output, and is
    // robust to a publisher that orders joints differently than we do.
    const size_t n = std::min(msg->joint_names.size(), pt.positions.size());
    for (size_t i = 0; i < n; ++i) {
      const auto it = axisByName_.find(msg->joint_names[i]);
      if (it == axisByName_.end()) {
        continue;                       // a joint we are not responsible for
      }
      const auto pos = std::find(jointAxes_.begin(), jointAxes_.end(), it->second);
      if (pos == jointAxes_.end()) {
        continue;
      }
      const size_t slot = static_cast<size_t>(std::distance(jointAxes_.begin(), pos));
      target[slot] = pt.positions[i];
      filled[slot] = true;
    }
  }

  // Every axis we drive must be commanded: a partial setpoint would hold some
  // joints and move others, bending the path Servo computed.
  for (size_t i = 0; i < count; ++i) {
    if (!filled[i]) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dropped trajectory: no command for axis %d (check joint_name vs publisher)",
        static_cast<int>(jointAxes_[i]));
      return;
    }
  }

  // Holding off after a fault. Discard setpoints rather than restarting the
  // channel into a condition that is still present.
  const auto now = std::chrono::steady_clock::now();
  if (now < faultUntil_) {
    return;
  }

  if (!streaming_) {
    if (!beginStream()) {
      // Without a holdoff this retries at the arrival rate, which is a 40 Hz
      // Clear/Execute storm against an engine that is already refusing.
      faultUntil_ = now + kFaultBackoffBase;
      return;
    }
  } else {
    // Time between overrides is what predicts smoothness. Each segment starts
    // decelerating exactly one period in (accel r*T + cruise (1-r)*T = T, for
    // any accel_ratio), so an override that lands late arrives into the decel
    // ramp and the axis has already given up speed. Nothing else reports this:
    // middleware drops are invisible from here, and the buffer is empty by
    // design, so depth cannot show it either.
    const auto gap = std::chrono::duration_cast<std::chrono::microseconds>(
      now - lastSetpointAt_).count();
    if (gap > (3LL * nominalPeriodUs_) / 2) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Override %.1f ms late (period %.1f ms); the segment was already decelerating",
        static_cast<double>(gap) / 1000.0, nominalPeriodUs_ / 1000.0);
    }
  }

  if (recordSetpoint(target)) {
    lastSetpointAt_ = std::chrono::steady_clock::now();
    // Forgive the fault history only once the stream has actually survived a
    // while. Resetting on a successful record instead would pin the backoff at
    // its first step forever, since recording succeeds regardless of whether
    // the engine can execute what was recorded.
    if (lastSetpointAt_ - streamStartedAt_ > kHealthyStreamTime) {
      consecutiveFaults_ = 0;
    }
  }
}

bool ServoStreamController::beginStream()
{
  // No origin to seed: every segment reads posCmd for itself at record time.
  if (!check(abCtl_->Clear(channelMotion_), "ApiBuffer::Clear")) {
    return false;
  }
  if (!check(abCtl_->Execute(channelMotion_), "ApiBuffer::Execute")) {
    return false;
  }

  // lastErrorCount_ is deliberately NOT reset here. errorCount is cumulative
  // for the life of the channel — the manual is explicit that channel statuses
  // are cleared by FreeApiBuffer, not by Clear — so zeroing it would make the
  // next pollStatus re-report the entire error history on every stream restart.
  // It belongs to pollStatus alone.
  streaming_ = true;
  streamStartedAt_ = std::chrono::steady_clock::now();
  // Arm the starvation clock here, not on the first record. Left at its old
  // value it can already be seconds stale, and the very next status poll would
  // stop the axes before a single setpoint had been recorded.
  lastSetpointAt_ = streamStartedAt_;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Stream started (channel %d Active)", channelMotion_);
  return true;
}

// controlled_stop: bring the axes to rest with the configured stop profile.
// Halt alone does not stop motion already commanded — the last interpolation
// would run to its target with that segment's (deliberately brisk) deceleration.
void ServoStreamController::endStream(const char * reason, bool controlled_stop)
{
  if (!streaming_) {
    return;
  }
  streaming_ = false;

  abCtl_->Halt(channelMotion_);
  abCtl_->Clear(channelMotion_);

  if (controlled_stop) {
    cmCtl_->motion->Stop(&axisSel_);
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Stream ended: %s%s", reason, controlled_stop ? " (axes stopped)" : "");
}

bool ServoStreamController::recordSetpoint(const std::vector<double> & target)
{
  const size_t count = jointAxes_.size();
  const double period_s = nominalPeriodUs_ / 1000000.0;

  // Origin is the live commanded position, not the previous setpoint — see the
  // motion-model note at the top of this file. Costs one extra cyclic read per
  // setpoint, which at 40 Hz is nothing, and it is what keeps lag from
  // accumulating now that every command lands as a mid-flight override.
  // Not check(): that decodes with ApiBuffer::ErrorToString, which yields an
  // empty string for a CoreMotion code. Same reason blockErrorToString exists.
  CoreMotionStatus cmStatus;
  const int status_err = cmCtl_->GetStatus(&cmStatus);
  if (status_err != ErrorCode::None) {
    char buf[256];
    blockErrorToString(status_err, buf, sizeof(buf));
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "CoreMotion::GetStatus failed, cannot place segment origin. Error=%d (%s)",
      status_err, buf);
    return false;
  }

  double peak = 0.0;                  // largest per-axis step, in radians
  for (size_t i = 0; i < count; ++i) {
    step_[i] = std::fabs(target[i] - cmStatus.axesStatus[jointAxes_[i]].posCmd);
    peak = std::max(peak, step_[i]);
  }

  // A hold, not a move. Recording it would ask for maxVelocity = 0 on every
  // axis, which the engine rejects — and with stop_on_error that stops the
  // channel, so holding a pose would read as a fault. Servo publishes the held
  // position whenever its input is centred, so this is a normal case.
  if (peak < minStepRad_) {
    return true;
  }

  // Uniform scale rather than a per-axis clamp: clamping one axis while the
  // others run free would bend the coordinated path. Slowing the whole move
  // keeps it straight and only costs time.
  const double peak_velocity = peak / period_s;
  const double scale = (peak_velocity > maxJointVelocity_) ?
    peak_velocity / maxJointVelocity_ : 1.0;
  if (scale > 1.0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Setpoint needs %.3f rad/s, over max_joint_velocity %.3f; scaling the move by %.2f "
      "(the arm will lag the stream)", peak_velocity, maxJointVelocity_, 1.0 / scale);
  }

  // Floor so an axis with nothing to do still carries a positive limit. A zero
  // limit on one axis of a coordinated move is the same rejection risk as the
  // all-zero case above, and since that axis has no distance to cover the floor
  // never binds.
  const double min_velocity = minStepRad_ / period_s;

  intpl_.axisCount = count;
  double path_sq = 0.0;               // |step| along the interpolation trajectory
  for (size_t i = 0; i < count; ++i) {
    const double velocity = std::max(step_[i] / period_s / scale, min_velocity);
    const double accel = velocity / (accelRatio_ * period_s);
    path_sq += step_[i] * step_[i];

    intpl_.axis[i] = static_cast<int>(jointAxes_[i]);
    intpl_.target[i] = target[i];
    intpl_.maxVelocity[i] = velocity;
    intpl_.maxAcc[i] = accel;
    intpl_.maxDec[i] = accel;
  }

  // Velocity along the trajectory. Every axis limit above is step_[i]/(T*scale)
  // and the path direction IS the step vector, so all axes reach their limit at
  // the same instant and the implied path velocity is exactly |step|/(T*scale).
  // The manual allows omitting profile.velocity when per-axis limits are given
  // (the trajectory velocity is then derived from them), and this reproduces
  // that derived value rather than constraining it -- but stating it explicitly
  // is what makes endVelocity well defined, because endVelocity is documented as
  // clamped to the profile velocity, and a profile velocity of 0 would clamp it
  // away to nothing.
  const double path_velocity = std::sqrt(path_sq) / period_s / scale;

  intpl_.profile.type = ProfileType::Trapezoidal;
  intpl_.profile.velocity = path_velocity;
  intpl_.profile.acc = 0.0;           // per-axis maxAcc/maxDec govern under AxisLimit
  intpl_.profile.dec = 0.0;
  intpl_.profile.startingVelocity = 0.0;   // override inherits the current velocity

  // The point of the whole exercise. With endVelocity = 0 the segment plans a
  // final deceleration to rest beginning at exactly t = T, and setpoints arrive
  // at T -- so every override landed on that boundary, where the manual is
  // explicit that FastBlending does NOT blend: "if the override interpolation is
  // executed while the current interpolation is in the final deceleration
  // segment, the current interpolation will run to completion instead of
  // transitioning to a stop motion." Completion means rest, so the arm
  // stop-and-goes at the stream rate. At ratio 1.0 there is no deceleration
  // segment left to collide with.
  //
  // The cost, and it is real: "If this value is not set to 0, the axis will
  // suddenly decelerate from the end velocity to 0 velocity at the end of
  // motion." A segment that COMPLETES now ends in a velocity step rather than a
  // ramp. Segments only complete when the stream stops feeding them, which is
  // why the starvation timer must command a controlled stop before the engine
  // gets there -- see the guard in setRosParameter.
  intpl_.profile.endVelocity = endVelocityRatio_ * path_velocity;

  if (!check(abRec_->StartRecordBufferChannel(channelMotion_), "StartRecordBufferChannel")) {
    return false;
  }
  const int motion_err = cmRec_->motion->StartLinearIntplPos(&intpl_);
  abRec_->EndRecordBufferChannel();

  if (motion_err != ErrorCode::None) {
    char buf[256];
    wmxRec_.ErrorToString(motion_err, buf, sizeof(buf));
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Failed to record setpoint. Error=%d (%s)", motion_err, buf);
    return false;
  }

  // Trace only blocks the engine accepted. cmStatus was read before this block
  // was recorded, so its posCmd/velocityCmd/cmdAcc are this segment's start
  // conditions -- and equally, the PREVIOUS segment's end conditions.
  ++setpointSeq_;
  if (setpointLogEvery_ > 0 && (setpointSeq_ % setpointLogEvery_) == 0) {
    logSetpoint(cmStatus, scale);
  }

  return true;
}

// Per-setpoint trace. Off by default: this runs on the pump thread at the stream
// rate, and formatting six axes is not free -- setpoint_log_every throttles it.
//
// "start" is read from the engine, "end" is what we programmed. The start
// velocity of the NEXT setpoint is the measurement of what the end velocity
// actually was; if it keeps reading ~0 while endVel is not 0, the blend is not
// engaging and the segments are running to completion.
void ServoStreamController::logSetpoint(const CoreMotionStatus & before, double scale)
{
  const double period_s = nominalPeriodUs_ / 1000000.0;
  char line[512];

  std::snprintf(
    line, sizeof(line),
    "setpoint #%" PRIu64 " | period %.2f ms | accel_ratio %.2f | scale %.3f | "
    "Trapezoidal, startVel inherited, endVel %.4f",
    setpointSeq_, period_s * 1000.0, accelRatio_, scale, intpl_.profile.endVelocity);
  std::string out(line);

  for (unsigned int i = 0; i < intpl_.axisCount; ++i) {
    const auto & a = before.axesStatus[intpl_.axis[i]];
    std::snprintf(
      line, sizeof(line),
      "\n  ax%-2d start[pos %+.6f vel %+.6f acc %+.4f%s%s] "
      "end[pos %+.6f vel %+.6f] "
      "cmd[step %.3e maxVel %.6f maxAcc %.4f maxDec %.4f]",
      intpl_.axis[i], a.posCmd, a.velocityCmd, a.cmdAcc,
      a.accFlag ? " ACC" : "", a.decFlag ? " DEC" : "",
      intpl_.target[i], intpl_.profile.endVelocity,
      step_[static_cast<size_t>(i)], intpl_.maxVelocity[i],
      intpl_.maxAcc[i], intpl_.maxDec[i]);
    out += line;
  }

  RCLCPP_INFO(this->get_logger(), "%s", out.c_str());
}

// The controlled stop for a stream that has gone quiet. This is a deadline, not
// a poll: with end_velocity_ratio > 0 the last segment ends in a velocity STEP
// rather than a ramp, so this has to reach the axes before the segment completes.
// setRosParameter checks the two times against each other at startup.
//
// A hold refreshes lastSetpointAt_ without recording anything, which is what
// keeps a deliberately stationary teleop input from being read as a dead stream.
void ServoStreamController::checkStarvation()
{
  if (!streaming_) {
    return;
  }
  if (std::chrono::steady_clock::now() - lastSetpointAt_ <=
    std::chrono::milliseconds(starvationTimeoutMs_))
  {
    return;
  }
  // Motion::Stop ONLY when the segment would not come to rest by itself. At
  // end_velocity_ratio 0 the in-flight interpolation already decelerates to
  // endVelocity = 0, so Halt/Clear alone lets it finish its own ramp -- which is
  // exactly what joint_position_controller does with a gap, and why the direct
  // path glides through a teleop pause that this one used to punctuate with a
  // commanded stop. Above 0 the segment ends at speed and the Stop is the whole
  // point of the deadline.
  const bool needs_stop = endVelocityRatio_ > 0.0;
  RCLCPP_WARN(
    this->get_logger(), "Servo stream starved (>%d ms); %s",
    starvationTimeoutMs_,
    needs_stop ? "stopping axes" : "letting the last segment ramp down");
  endStream("stream starved", needs_stop);
}

// Everything that watches the channel rather than feeds it. This used to be
// split between the pump (fault detection, starvation, depth) and here; with the
// pump gone it all lives on this timer, which keeps the one cyclic GetStatus off
// the setpoint path entirely -- the callback's only engine read is the posCmd
// origin it cannot do without.
//
// Surfaces the error log too. The log holds only 10 entries with index 0 newest,
// so errorCount (not the array length) is what reveals how many were missed.
void ServoStreamController::pollStatus()
{
  if (!initialized_) {
    return;
  }

  ApiBufferStatus st;
  if (abCtl_->GetStatus(channelMotion_, &st) != ErrorCode::None) {
    return;
  }

  // One block per setpoint, so this is blocks and setpoints at once. Read from
  // the status every poll rather than cached from a record, so it stays honest
  // when the stream is down. It sits at 0-1 in steady state and is a "can the
  // engine keep up" signal, not a latency dial.
  const int depth = static_cast<int>(st.remainingBlockCount);
  std_msgs::msg::Int32 depth_msg;
  depth_msg.data = depth;
  depthPub_->publish(depth_msg);

  if (streaming_ && depth > kDepthWarnBlocks) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "API buffer not draining: %d blocks in flight (expected 0-1)", depth);
  }

  if (streaming_ && st.state == ApiBufferState::Stop) {
    // A Stop with nothing recorded against it is not a fault: since GetStatus
    // is one cycle stale, the first poll after Execute reads back the Stop that
    // endStream's Halt left behind. Faulting on that tears down the stream just
    // after starting it, Halt/Clear/Stops the axes on nothing, and escalates
    // the backoff -- so the next attempt is delayed on evidence that was never
    // there. Let the status catch up instead.
    const bool recorded_error = st.watchError || st.errorCount > 0;
    const bool settled =
      std::chrono::steady_clock::now() - streamStartedAt_ >= kStatusSettleTime;

    if (recorded_error || settled) {
      ++consecutiveFaults_;
      const auto backoff = std::min(
        kFaultBackoffMax,
        kFaultBackoffBase * (1 << std::min(consecutiveFaults_ - 1, kFaultBackoffMaxShift)));
      faultUntil_ = std::chrono::steady_clock::now() + backoff;

      // watchErrorCode only means anything when watchError is set, and
      // "stopOnError" was a guess at the cause rather than a reading of it.
      // Name what the device recorded: the watch, the block that failed (error
      // log index 0 is newest), or nothing at all.
      char cause[320];
      if (st.watchError) {
        wmxCtl_.ErrorToString(st.watchErrorCode, errString_, sizeof(errString_));
        snprintf(
          cause, sizeof(cause), "watch fired on axis %d, code %d (%s)",
          st.watchErrorAxis, st.watchErrorCode, errString_);
      } else if (st.errorCount > 0) {
        // blockErrorToString, NOT the device decoder: a block error is a
        // CoreMotion/ApiBuffer code, and running it through WMX3Api::
        // ErrorToString yields a plausible-looking string for an unrelated
        // error. 65630 came out as "Failed to obtain lock for starting process"
        // when it actually means "Currently processing override".
        blockErrorToString(st.errorLog[0].errorCode, errString_, sizeof(errString_));
        snprintf(
          cause, sizeof(cause), "block %lld failed, code %d (%s)",
          st.errorLog[0].execBlockNumber,
          st.errorLog[0].errorCode, errString_);
      } else {
        snprintf(
          cause, sizeof(cause), "no error recorded, channel never left Stop after Execute");
      }
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Motion channel stopped (%s; errors=%lld); fault %d, holding off %d ms",
        cause, st.errorCount, consecutiveFaults_, static_cast<int>(backoff.count()));

      endStream("channel stopped", true);
    }
  }

  // Liveness. remainingBlockCount is 0-1 by design, so an empty buffer proves
  // nothing either way — what proves the engine is still consuming is the
  // cumulative count advancing. A channel that is Active but not executing
  // (an axis stuck mid-interpolation, say) shows up here and nowhere else.
  //
  // It also has to know whether anything was OFFERED. setpointSeq_ counts
  // recorded blocks only -- a hold returns before the record -- so a stationary
  // teleop input advances neither counter, and without this the warning fired
  // through every deliberate pause and cried stall at a perfectly healthy
  // engine.
  const int64_t consumed = st.cumulativeBlockCount - lastCumulativeBlocks_;
  const bool recorded_any = setpointSeq_ != lastSeqSeen_;
  lastCumulativeBlocks_ = st.cumulativeBlockCount;
  lastSeqSeen_ = setpointSeq_;
  if (streaming_ && recorded_any && consumed == 0) {
    if (++stalledPolls_ >= kStalledPollCount) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Streaming but no blocks consumed in %d ms; engine may be stalled",
        stalledPolls_ * static_cast<int>(kStatusPollPeriod.count()));
    }
  } else {
    stalledPolls_ = 0;
  }

  if (st.errorCount == lastErrorCount_) {
    return;
  }

  const int64_t missed = st.errorCount - lastErrorCount_;
  lastErrorCount_ = st.errorCount;

  const int reportable = static_cast<int>(
    std::min<int64_t>(missed, wmx3Api::constants::maxApiBufferErrorLog));

  // ProcessingOverride is not a fault once stop_on_error is false: it is the
  // engine saying the previous override is still in its smoothing segment, so
  // this setpoint was skipped and the axes stayed on the segment already in
  // flight. That is the intended degradation, and at stream rate it can happen
  // many times a second -- logging each one would put a hundred ERROR lines a
  // second on the executor thread that also runs the setpoint callback, which
  // would corrupt the very timing being measured. Count them, report a rate.
  int rejects = 0;
  for (int i = 0; i < reportable; ++i) {
    const int code = st.errorLog[i].errorCode;
    if (code == wmx3Api::CoreMotionErrorCode::ProcessingOverride) {
      ++rejects;
      continue;
    }
    blockErrorToString(code, errString_, sizeof(errString_));
    RCLCPP_ERROR(
      this->get_logger(),
      "API buffer error at block %lld (one block per setpoint): %d (%s)",
      st.errorLog[i].execBlockNumber, code, errString_);
  }

  if (rejects > 0) {
    overrideRejects_ += rejects;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Engine refused at least %" PRId64 " override(s) so far (previous override still "
      "smoothing); those setpoints were skipped and the arm continued the segment "
      "in flight. The error log holds %d entries per poll, so this is a floor.",
      overrideRejects_, static_cast<int>(wmx3Api::constants::maxApiBufferErrorLog));
  }

  if (missed > reportable) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "%" PRId64 " further block errors overflowed the log between polls",
      missed - reportable);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoStreamController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
