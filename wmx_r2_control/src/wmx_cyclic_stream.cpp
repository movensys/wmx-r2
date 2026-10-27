// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_r2_control/wmx_cyclic_stream.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

// [local 2026-10-13 진단] 마지막 GetStatus 의 축별 상태, 고장 난 축 (스트림은 프로세스에 하나)
static std::string g_last_buffer_diag;
static int g_fault_axis = -1;
// [local 2026-10-13] CR3A_R2_VELINT=1: 1 ms 점을 JTC 속도의 구간 안 직선 보간을 적분해 만든다 (위치는 JTC 끝점 쪽으로 천천히 보정).
// 3차 Hermite (위치·속도 끝점 맞춤) 는 JTC 가 궤적 시각을 넘긴 간격과 여기서 쓰는 구간 시간이 ~15-20 us 만 달라도
// v x dt 의 위치 어긋남이 끝 가속 +-3..7 rad/s^2 로 커졌다 (3 rad/s 에서 maxAcc 15 도 넘음). 속도 적분은 1 ms 가속이
// 구간 평균 가속 (|v1 - v0| / h) + 보정분을 넘지 않는다.
static std::vector<double> g_vi_p;      // 마지막으로 넣은 점의 위치 (축 순서 = cfg_.axes)
static const double kViGain = 0.2;      // 구간마다 JTC 위치와의 차의 이만큼을 그 구간 동안 보정
static bool velIntEnabled()
{
  static const bool on = [] {
      const char * e = std::getenv("CR3A_R2_VELINT");
      return e != nullptr && e[0] == '1';
    }();
  return on;
}

namespace wmx_r2_control
{

using wmx3Api::AxisCommandMode;
using wmx3Api::CyclicBufferCommandType;
using wmx3Api::CyclicBufferState;
using wmx3Api::ErrorCode;

WmxCyclicStream::~WmxCyclicStream()
{
  close();
}

void WmxCyclicStream::setError(std::string * error, const char * what, int code) const
{
  if (error == nullptr) {
    return;
  }
  char buf[256] = {0};
  wmx3Api::CyclicBuffer::ErrorToString(code, buf, sizeof(buf));
  *error = std::string(what) + ": error=" + std::to_string(code) + " (" + buf + ")";
}

bool WmxCyclicStream::open(
  wmx3Api::WMX3Api * wmx, wmx3Api::CoreMotion * cm, const Config & cfg,
  double seed_push_period_ms, std::string * error)
{
  if (open_) {
    if (error != nullptr) {*error = "cyclic stream is already open";}
    return false;
  }
  // Config first: it is the pure check, needing neither handles nor an engine,
  // and a bad axis list is the likelier real-world mistake (it comes from the
  // ros2_control xacro) so it deserves the more specific diagnostic.
  if (cfg.axes.empty() || cfg.axes.size() > static_cast<size_t>(wmx3Api::constants::maxAxes)) {
    if (error != nullptr) {
      *error = "axis list must hold between 1 and " +
        std::to_string(wmx3Api::constants::maxAxes) + " axes";
    }
    return false;
  }
  for (const int axis : cfg.axes) {
    if (axis < 0 || axis >= wmx3Api::constants::maxAxes) {
      if (error != nullptr) {*error = "axis " + std::to_string(axis) + " is out of range";}
      return false;
    }
  }
  if (wmx == nullptr || cm == nullptr) {
    if (error != nullptr) {*error = "null WMX3Api or CoreMotion handle";}
    return false;
  }

  wmx_ = wmx;
  cm_ = cm;
  cfg_ = cfg;

  sel_.axisCount = static_cast<int>(cfg_.axes.size());
  for (size_t i = 0; i < cfg_.axes.size(); ++i) {
    sel_.axis[i] = cfg_.axes[i];
  }

  // AbsolutePos commands require Position command mode; anything else aborts
  // execution into the latching CommandModeMismatchError state at the first
  // command. Check it here so a misconfigured axis fails at activation with a
  // clear message instead of going limp mid-motion.
  for (const int axis : cfg_.axes) {
    AxisCommandMode::T mode = AxisCommandMode::T::Position;
    const int err = cm_->axisControl->GetAxisCommandMode(axis, &mode);
    if (err != ErrorCode::None) {
      setError(error, "GetAxisCommandMode failed", err);
      return false;
    }
    if (mode != AxisCommandMode::T::Position) {
      if (error != nullptr) {
        *error = "axis " + std::to_string(axis) + " is not in Position command mode (mode=" +
          std::to_string(static_cast<int>(mode)) + "); AbsolutePos cyclic commands require it";
      }
      return false;
    }
  }

  // The engine communication cycle is not configured anywhere in this workspace
  // -- it belongs to the WMX engine. Read it; a hardcoded 0.5 ms would silently
  // scale every commanded velocity on a differently configured engine.
  wmx3Api::CoreMotionStatus status;
  const int err = cm_->GetStatus(&status);
  if (err != ErrorCode::None) {
    setError(error, "CoreMotion GetStatus failed", err);
    return false;
  }
  timing_ = cyclic::StreamTiming();
  timing_.cycle_ms = status.cycleTimeMilliseconds[0];
  timing_.push_period_ms = seed_push_period_ms;
  if (!timing_.valid()) {
    if (error != nullptr) {
      *error = "invalid stream timing (engine cycle=" + std::to_string(timing_.cycle_ms) +
        " ms, push period=" + std::to_string(timing_.push_period_ms) +
        " ms); is the engine communicating?";
    }
    return false;
  }
  period_ = std::make_unique<cyclic::PeriodTracker>(seed_push_period_ms);
  target_depth_ = timing_.targetDepth(cfg_.target_periods);

  cb_ = std::make_unique<wmx3Api::CyclicBuffer>(wmx_);

  // Allocates on the engine: activation only, never from the control loop.
  unsigned int slots = timing_.bufferCommands(cfg_.buffer_horizon_s);
  if (cfg_.hermite) {
    // [local 2026-10-13] one-cycle commands: size and depth are in cycles
    const double h = cfg_.buffer_horizon_s;
    if (!std::isfinite(h) || h <= 0.0 || h > 600.0) {
      if (error != nullptr) {*error = "cyclic_buffer_horizon_s must be in (0, 600] s";}
      return false;
    }
    slots = static_cast<unsigned int>(std::max<long>(1, std::lround(h * 1000.0 / timing_.cycle_ms)));
    target_cycles_ = target_depth_ * timing_.nominalInterval();
    // Largest single append: priming (target_cycles_) or one segment after a loop
    // stall of up to 8 push periods (a longer one is a fault, see pushSamples()).
    const unsigned int seg_max = 8u * timing_.nominalInterval();
    taus_.assign(seg_max, 0.0);
    block_.assign(std::max(target_cycles_, seg_max), wmx3Api::CyclicBufferMultiAxisCommands());
    seg_p_.assign(cfg_.axes.size(), 0.0);
    seg_v_.assign(cfg_.axes.size(), 0.0);
    v1_scratch_.assign(cfg_.axes.size(), 0.0);
  }
  const int open_err = cb_->OpenCyclicBuffer(&sel_, slots);
  if (open_err != ErrorCode::None) {
    setError(error, "OpenCyclicBuffer failed", open_err);
    cb_.reset();
    return false;
  }

  open_ = true;
  return true;
}

bool WmxCyclicStream::holdCommands(
  wmx3Api::CyclicBufferMultiAxisCommands * cmds, unsigned int interval, std::string * error)
{
  for (const int axis : cfg_.axes) {
    // The *command* position, not the feedback position. Seeding from feedback
    // (CoreMotionStatus::actualPos, which read() correctly uses for
    // /joint_states) would command a step equal to the servo following error on
    // the first cycle, simultaneously on every axis.
    double commanded = 0.0;
    const int err = cm_->axisControl->GetPosCommand(axis, &commanded);
    if (err != ErrorCode::None) {
      setError(error, "GetPosCommand failed", err);
      return false;
    }
    // Indexed by axis number, not by position in the selection: cmd[], option[]
    // and status[] are all maxAxes-wide arrays keyed on the axis.
    cmds->cmd[axis].type = CyclicBufferCommandType::T::AbsolutePos;
    cmds->cmd[axis].command = commanded;
    cmds->cmd[axis].intervalCycles = interval;
  }
  return true;
}

bool WmxCyclicStream::start(std::string * error)
{
  if (!open_) {
    if (error != nullptr) {*error = "cyclic stream is not open";}
    return false;
  }
  if (running_) {
    return true;
  }

  const unsigned int interval = timing_.nominalInterval();

  if (cfg_.hermite) {
    // [local 2026-10-13] prime target_cycles_ one-cycle holds in one call and
    // start the first segment from the held pose at rest.
    wmx3Api::CyclicBufferMultiAxisCommands hold;
    if (!holdCommands(&hold, 1u, error)) {
      return false;
    }
    // target_cycles_ can grow after open() (observePushPeriod), block_ cannot: clamp.
    const unsigned int prime = std::min<unsigned int>(target_cycles_, static_cast<unsigned int>(block_.size()));
    for (unsigned int k = 0; k < prime; ++k) {
      block_[k] = hold;
    }
    for (size_t i = 0; i < cfg_.axes.size(); ++i) {
      seg_p_[i] = hold.cmd[cfg_.axes[i]].command;
      seg_v_[i] = 0.0;
    }
    next_tau_ = timing_.cycle_ms / 1000.0;
    pending_s_ = 0.0;
    stalled_ = false;
  g_vi_p.clear();
    reg_.reset();
    const int err = cb_->AddCommand(&sel_, block_.data(), static_cast<int>(prime));
    if (err != ErrorCode::None) {
      setError(error, "AddCommand failed while priming the buffer", err);
      return false;
    }
  }

  // Prime the setpoint's worth of runway holding the current pose, so the first
  // real command lands into a buffer that is already at depth rather than
  // starting from an underrun.
  for (unsigned int i = 0; i < (cfg_.hermite ? 0u : target_depth_); ++i) {
    wmx3Api::CyclicBufferMultiAxisCommands hold;
    if (!holdCommands(&hold, interval, error)) {
      return false;
    }
    const int err = cb_->AddCommand(&sel_, &hold);
    if (err != ErrorCode::None) {
      setError(error, "AddCommand failed while priming the buffer", err);
      return false;
    }
  }

  wmx3Api::CyclicBufferMultiAxisOption option;
  for (const int axis : cfg_.axes) {
    option.option[axis].maxAcc = cfg_.max_acc;
  }

  // Execute is called once. When the buffer drains, the state becomes
  // WaitingForCommand and execution resumes automatically on the next
  // AddCommand -- there is no need (and no way) to re-Execute per command.
  const int err = cb_->Execute(&sel_, &option);
  if (err != ErrorCode::None) {
    setError(error, "Execute failed", err);
    return false;
  }

  running_ = true;
  return refresh(error);
}

bool WmxCyclicStream::queryBuffer(
  unsigned int * min_free, unsigned int * min_depth, std::string * error)
{
  wmx3Api::CyclicBufferMultiAxisStatus status;
  const int err = cb_->GetStatus(&sel_, &status);
  if (err != ErrorCode::None) {
    setError(error, "CyclicBuffer GetStatus failed", err);
    return false;
  }

  unsigned int free_slots = std::numeric_limits<unsigned int>::max();
  unsigned int depth = std::numeric_limits<unsigned int>::max();
  g_last_buffer_diag.clear();   // [local 2026-10-13 진단] 거절 때 축별 값을 같이 남김
  for (const int axis : cfg_.axes) {
    const auto & axis_status = status.status[axis];
    g_last_buffer_diag += " ax" + std::to_string(axis) + "(st" + std::to_string(static_cast<int>(axis_status.state)) +
      " remain " + std::to_string(axis_status.remainCount) + " avail " + std::to_string(axis_status.availableCount) + ")";
    // [local 2026-10-13] availableCount IS the free space (measured on 3.7.0 simu: remain
    // 20 / available 4980 on a 5000 buffer, 2500 / 2500 on a stopped axis). The old
    // capacity reading (free = available - remain) only under-counted by remain.
    const unsigned int axis_free = axis_status.availableCount;
    free_slots = std::min(free_slots, axis_free);
    depth = std::min(depth, axis_status.remainCount);
  }

  // [local 2026-10-13] The first axis' state is not enough: a latching error is per
  // axis. Measured (rt_20261013_194708 / 201655 / 202157): only joint5 hit MaxAccError
  // (state 3, its queue filling to 2500/5000) while the other five kept moving; it
  // surfaced 2.5 s later as "buffer full". Any axis in a latching error faults the stream.
  state_ = status.status[cfg_.axes.front()].state;
  for (const int axis : cfg_.axes) {
    const auto st = status.status[axis].state;
    if (st == CyclicBufferState::T::MaxAccError || st == CyclicBufferState::T::CommandModeMismatchError ||
      st == CyclicBufferState::T::ServoOff)
    {
      state_ = st;
      g_fault_axis = axis;
      break;
    }
  }
  *min_free = free_slots;
  *min_depth = depth;
  return true;
}

bool WmxCyclicStream::refresh(std::string * error)
{
  if (!open_) {
    if (error != nullptr) {*error = "cyclic stream is not open";}
    return false;
  }
  unsigned int free_slots = 0u;
  unsigned int depth = 0u;
  if (!queryBuffer(&free_slots, &depth, error)) {
    return false;
  }
  depth_ = depth;
  return true;
}

bool WmxCyclicStream::push(const std::vector<double> & positions, std::string * error)
{
  if (!running_) {
    if (error != nullptr) {*error = "cyclic stream is not running";}
    return false;
  }
  if (positions.size() != cfg_.axes.size()) {
    if (error != nullptr) {
      *error = "expected " + std::to_string(cfg_.axes.size()) + " positions, got " +
        std::to_string(positions.size());
    }
    return false;
  }
  for (const double q : positions) {
    if (!std::isfinite(q)) {
      ++rejected_;
      if (error != nullptr) {*error = "refusing to command a non-finite position";}
      return false;
    }
  }

  unsigned int free_slots = 0u;
  unsigned int depth = 0u;
  if (!queryBuffer(&free_slots, &depth, error)) {
    return false;
  }
  depth_ = depth;

  if (faulted()) {
    ++rejected_;
    if (error != nullptr) {
      *error = std::string("cyclic buffer is in a latching error state (") + stateName() +
        (g_fault_axis >= 0 ? " on axis " + std::to_string(g_fault_axis) : std::string()) +
        "); abort and restart the stream to recover";
    }
    return false;
  }

  // Check every axis has room before committing to any of them. The multi-axis
  // AddCommand applies *partially* on BufferSizeIsNotEnough -- axes ordered
  // before the failing one keep their command -- which on a 6-DOF arm desyncs
  // the joints rather than merely dropping a frame.
  if (free_slots < 1u) {
    ++rejected_;
    if (error != nullptr) {
      *error = "cyclic buffer full (depth=" + std::to_string(depth_) + "); dropping command";
    }
    return false;
  }

  const unsigned int interval = cyclic::nextInterval(timing_, target_depth_, depth_);

  wmx3Api::CyclicBufferMultiAxisCommands cmds;
  for (size_t i = 0; i < cfg_.axes.size(); ++i) {
    const int axis = cfg_.axes[i];
    cmds.cmd[axis].type = CyclicBufferCommandType::T::AbsolutePos;
    cmds.cmd[axis].command = positions[i];
    cmds.cmd[axis].intervalCycles = interval;
  }

  const int err = cb_->AddCommand(&sel_, &cmds);
  if (err != ErrorCode::None) {
    ++rejected_;
    setError(error, "AddCommand failed", err);
    return false;
  }
  return true;
}

void WmxCyclicStream::observePushPeriod(double measured_ms)
{
  if (period_ == nullptr) {
    return;
  }
  period_->observe(measured_ms);
  timing_.push_period_ms = period_->periodMs();
  target_depth_ = timing_.targetDepth(cfg_.target_periods);
  if (cfg_.hermite) {
    target_cycles_ = target_depth_ * timing_.nominalInterval();
  }
}

bool WmxCyclicStream::pushSamples(
  const std::vector<double> & positions, const std::vector<double> & velocities,
  double segment_s, std::string * error)
{
  if (!running_ || !cfg_.hermite) {
    if (error != nullptr) {*error = "cyclic stream is not running in hermite mode";}
    return false;
  }
  if (positions.size() != cfg_.axes.size() || velocities.size() != cfg_.axes.size()) {
    if (error != nullptr) {*error = "positions/velocities do not match the axis count";}
    return false;
  }
  if (!std::isfinite(segment_s) || segment_s <= 0.0) {
    segment_s = timing_.push_period_ms / 1000.0;
  }
  // A rejected push must not lose its time: the next cubic spans every period
  // since the last appended segment (review 2026-10-13: otherwise two periods of
  // motion land in one period of samples, ~2x velocity).
  const double h = pending_s_ + segment_s;
  for (const double q : positions) {
    if (!std::isfinite(q)) {
      ++rejected_;
      pending_s_ = h;
      if (error != nullptr) {*error = "refusing to command a non-finite position";}
      return false;
    }
  }

  unsigned int free_slots = 0u;
  unsigned int depth = 0u;
  if (!queryBuffer(&free_slots, &depth, error)) {
    pending_s_ = h;
    return false;
  }
  depth_ = depth;
  if (faulted()) {
    ++rejected_;
    if (error != nullptr) {
      *error = std::string("cyclic buffer is in a latching error state (") + stateName() +
        (g_fault_axis >= 0 ? " on axis " + std::to_string(g_fault_axis) : std::string()) +
        "); abort and restart the stream to recover";
    }
    return false;
  }

  const double step = timing_.cycle_ms / 1000.0 * reg_.update(target_cycles_, depth_);
  const unsigned int max_n = static_cast<unsigned int>(taus_.size());
  // A segment longer than one push can hold means the loop stalled well past the
  // queued runway: the engine has already underrun and held. Compressing the
  // stall into fewer samples would replay it faster with fixed end slopes, so it
  // is a latching fault instead (review 2026-10-13).
  if (h > step * static_cast<double>(max_n)) {
    stalled_ = true;
    ++rejected_;
    if (error != nullptr) {
      *error = "control loop stalled for " + std::to_string(h * 1000.0) +
        " ms (more than one push can hold); stream stopped";
    }
    return false;
  }
  double next_tau = next_tau_;
  const unsigned int n = cyclic::gridSamples(next_tau, h, step, taus_.data(), max_n);
  // Same partial-add hazard as push(): check every axis has room for all n first.
  if (free_slots < n) {
    ++rejected_;
    pending_s_ = h;
    if (error != nullptr) {
      *error = "cyclic buffer full (depth=" + std::to_string(depth_) + ", free " + std::to_string(free_slots) +
        " < n " + std::to_string(n) + ", h " + std::to_string(h * 1000.0) + " ms," + g_last_buffer_diag + "); deferring segment";
    }
    return false;
  }

  // End slopes: the controller's velocity command; if it is missing (interface
  // not claimed -> NaN, see WmxSystemHardware) use the chord (p1 - p0) / h rather
  // than 0, which would bend the cubic into a stop at every boundary.
  for (size_t i = 0; i < cfg_.axes.size(); ++i) {
    v1_scratch_[i] = std::isfinite(velocities[i]) ? velocities[i] : (positions[i] - seg_p_[i]) / h;
  }
  std::vector<double> vi_next;
  if (velIntEnabled() && n > 0u) {
    if (g_vi_p.size() != cfg_.axes.size()) {
      g_vi_p.assign(seg_p_.begin(), seg_p_.end());
    }
    vi_next = g_vi_p;
    const double step_tau = (n > 1u) ? (taus_[1] - taus_[0]) : taus_[0];
    for (size_t i = 0; i < cfg_.axes.size(); ++i) {
      const double v0 = seg_v_[i];
      const double v1 = v1_scratch_[i];
      auto vel = [&](double tau) {return v0 + (v1 - v0) * std::min(std::max(tau, 0.0), h) / h;};
      const double tau0 = taus_[0] - step_tau;        // 지난 점의 시각 (이 구간 시작 기준, 0 아래면 지난 구간)
      const double ta = std::max(tau0, 0.0);
      const double to_end = (tau0 < 0.0 ? v0 * (-tau0) : 0.0) + 0.5 * (vel(ta) + v1) * (h - ta);
      const double corr = kViGain * (positions[i] - (vi_next[i] + to_end)) / h;   // 보정 속도 (이 구간 동안)
      double prev = tau0;
      double p = vi_next[i];
      for (unsigned int k = 1; k <= n; ++k) {
        const double tau = taus_[k - 1];
        p += 0.5 * (vel(prev) + vel(tau)) * (tau - prev) + corr * (tau - prev);
        prev = tau;
        auto & cmds = block_[k - 1];
        const int axis = cfg_.axes[i];
        cmds.cmd[axis].type = CyclicBufferCommandType::T::AbsolutePos;
        cmds.cmd[axis].command = p;
        cmds.cmd[axis].intervalCycles = 1u;
      }
      vi_next[i] = p;
    }
  } else {
    for (unsigned int k = 1; k <= n; ++k) {
      const double s = taus_[k - 1] / h;
      auto & cmds = block_[k - 1];
      for (size_t i = 0; i < cfg_.axes.size(); ++i) {
        const int axis = cfg_.axes[i];
        cmds.cmd[axis].type = CyclicBufferCommandType::T::AbsolutePos;
        cmds.cmd[axis].command = cyclic::hermite(seg_p_[i], seg_v_[i], positions[i], v1_scratch_[i], h, s);
        cmds.cmd[axis].intervalCycles = 1u;
      }
    }
  }

  if (n > 0u) {
    const int err = cb_->AddCommand(&sel_, block_.data(), static_cast<int>(n));
    if (err != ErrorCode::None) {
      ++rejected_;
      pending_s_ = h;
      setError(error, "AddCommand failed", err);
      return false;
    }
  }
  next_tau_ = next_tau;
  pending_s_ = 0.0;
  if (!vi_next.empty()) {
    g_vi_p = vi_next;
  }
  for (size_t i = 0; i < cfg_.axes.size(); ++i) {
    seg_p_[i] = positions[i];
    seg_v_[i] = v1_scratch_[i];
  }
  last_count_ = n;
  return true;
}

void WmxCyclicStream::abort()
{
  if (!open_ || cb_ == nullptr) {
    running_ = false;
    return;
  }
  // Abort, not ExecQuickStop: Abort stops immediately and can be re-armed on the
  // very next cycle, whereas ExecQuickStop blocks AddCommand and Execute until
  // the state returns to Stopped, silently swallowing the first commands of the
  // next engage.
  cb_->Abort(&sel_);
  running_ = false;
  stalled_ = false;
  g_vi_p.clear();
  pending_s_ = 0.0;
  depth_ = 0u;
  state_ = CyclicBufferState::T::Stopped;
}

void WmxCyclicStream::close()
{
  if (!open_) {
    return;
  }
  abort();
  if (cb_ != nullptr) {
    cb_->CloseCyclicBuffer(&sel_);
    cb_.reset();
  }
  period_.reset();
  open_ = false;
}

bool WmxCyclicStream::faulted() const
{
  if (stalled_) {
    return true;   // [local 2026-10-13] hermite: loop stall longer than one push (pushSamples)
  }
  switch (state_) {
    // Both clear the buffer and latch: another push cannot recover them.
    case CyclicBufferState::T::MaxAccError:
    case CyclicBufferState::T::CommandModeMismatchError:
    // Not in the vendored markdown docs but present in CyclicBufferApi.h. A
    // dropped servo must be treated as a fault, not as a transient, or the
    // stream would keep pushing into an axis that cannot move.
    case CyclicBufferState::T::ServoOff:
      return true;
    default:
      return false;
  }
}

const char * WmxCyclicStream::stateName() const
{
  if (stalled_) {
    return "LoopStalled";
  }
  switch (state_) {
    case CyclicBufferState::T::Stopped:
      return "Stopped";
    case CyclicBufferState::T::ExecutingCommand:
      return "ExecutingCommand";
    case CyclicBufferState::T::WaitingForCommand:
      return "WaitingForCommand";
    case CyclicBufferState::T::MaxAccError:
      return "MaxAccError";
    case CyclicBufferState::T::CommandModeMismatchError:
      return "CommandModeMismatchError";
    case CyclicBufferState::T::Busy:
      return "Busy";
    case CyclicBufferState::T::ServoOff:
      return "ServoOff";
    default:
      return "Unknown";
  }
}

}  // namespace wmx_r2_control
