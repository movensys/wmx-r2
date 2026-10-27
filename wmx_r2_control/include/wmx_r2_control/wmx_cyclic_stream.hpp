// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// Streams per-cycle absolute position commands to a group of WMX3 axes through
// the CyclicBuffer module, so that a continuously updated joint target (MoveIt
// Servo, a teleop device) can drive the arm. Contrast StartCSplinePos, which
// needs the whole trajectory up front and cannot be blended into.
//
// Lifecycle, mapped onto the ros2_control hardware component:
//     open()     on_configure/on_activate  -- allocates; never in the loop
//     start()    on_activate               -- seeds a hold, then Execute
//     push()     write()                   -- one command per cycle
//     abort()    on_deactivate             -- immediate stop, buffer cleared
//     close()    on_cleanup                -- deallocates
//
// push() polls the buffer itself, so a running stream needs exactly one engine
// round-trip per control cycle. Do NOT also call refresh() from read(): that
// would double the polling in the real-time loop for no new information.
//
// All the timing arithmetic lives in cyclic_stream_timing.hpp, which is free of
// WMX types and unit-tested; this class is the WMX contact surface only.
#ifndef WMX_R2_CONTROL__WMX_CYCLIC_STREAM_HPP_
#define WMX_R2_CONTROL__WMX_CYCLIC_STREAM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "CyclicBufferApi.h"

#include "wmx_r2_control/cyclic_stream_timing.hpp"

namespace wmx_r2_control
{

class WmxCyclicStream
{
public:
  struct Config
  {
    /// WMX axis numbers, in the same order as the position values passed to
    /// push(). Must be non-empty and within [0, constants::maxAxes).
    std::vector<int> axes;

    /// How much motion the buffer can hold, used only to size it. Generous is
    /// cheap: the buffer is sized in commands, not cycles (see bufferCommands).
    double buffer_horizon_s = 5.0;

    /// Push-periods of runway to hold queued. Each one buys tolerance to a late
    /// write() and costs a push-period of added latency.
    unsigned int target_periods = 2u;

    /// Acceleration trip [user units/s^2; radians/s^2 on the CR3A]. Exceeding it
    /// quick-stops the axis and clears the buffer, so this is a safety limit and
    /// not a smoother. 0 disables it.
    double max_acc = 0.0;

    /// [local 2026-10-13] false: push() = one command per write(), split linearly
    /// over ~N cycles (branch behaviour). true: pushSamples() = N one-cycle commands
    /// on the cubic between the previous and current (position, velocity), see
    /// cyclic::hermite(). Depth and the buffer size are then counted in cycles.
    bool hermite = false;
  };

  WmxCyclicStream() = default;
  ~WmxCyclicStream();

  // Owns an open cyclic buffer on the engine; copying would double-close it.
  WmxCyclicStream(const WmxCyclicStream &) = delete;
  WmxCyclicStream & operator=(const WmxCyclicStream &) = delete;

  /// Validate the config, read the engine cycle time, and open the buffer.
  /// `wmx` and `cm` are borrowed and must outlive this object.
  /// `seed_push_period_ms` is the expected write() period (1000 / update_rate);
  /// the real one is learned via observePushPeriod().
  bool open(
    wmx3Api::WMX3Api * wmx, wmx3Api::CoreMotion * cm, const Config & cfg,
    double seed_push_period_ms, std::string * error);

  /// Seed a hold at the current *command* position and start execution. After
  /// this the axes are in DirectControl and nothing else may command them.
  bool start(std::string * error);

  /// Refresh depth() and state() from the engine.
  ///
  /// Only needed when you want status *without* pushing -- after start(), or
  /// while stopped, or to inspect a fault. push() refreshes as part of its
  /// free-space precheck, so calling this from read() alongside push() in
  /// write() just polls the engine twice per cycle.
  bool refresh(std::string * error);

  /// Append one command per axis, interpolated over a depth-regulated number of
  /// cycles. `positions` must be parallel to Config::axes, in user units.
  /// Returns false (and counts a rejection) without commanding anything if the
  /// buffer lacks room on any axis -- see the partial-add hazard in the .cpp.
  bool push(const std::vector<double> & positions, std::string * error);

  /// [local 2026-10-13] Hermite mode (Config::hermite): append one-cycle samples of
  /// the cubic from the end of the previous segment to (`positions`, `velocities`)
  /// [user units, user units/s], parallel to Config::axes. The segment lasts
  /// `segment_s` (time since the previous write()) plus the time of any rejected
  /// pushes since the last success; samples sit on the engine-cycle grid with the
  /// remainder carried over (cyclic::gridSamples), the grid step nudged slowly by
  /// the filtered depth (cyclic::GridRegulator). A non-finite velocity (interface
  /// not claimed) falls back to the chord; non-finite positions are rejected. A
  /// segment longer than one push can hold is a latching fault (faulted()).
  bool pushSamples(
    const std::vector<double> & positions, const std::vector<double> & velocities,
    double segment_s, std::string * error);

  /// Feed write()'s measured period so the commanded interval tracks the real
  /// loop rate rather than the configured one.
  void observePushPeriod(double measured_ms);

  /// Stop immediately and clear the buffer. Safe to call when not running.
  /// Preferred over ExecQuickStop, which blocks re-arming for a cycle.
  void abort();

  /// Release the buffer. Aborts first if still running.
  void close();

  bool isOpen() const {return open_;}
  bool isRunning() const {return running_;}

  /// True for the latching error states: pushing again will not recover, the
  /// stream must be aborted and restarted.
  bool faulted() const;

  /// Human-readable current state, for logs.
  const char * stateName() const;

  wmx3Api::CyclicBufferState::T state() const {return state_;}
  unsigned int depth() const {return depth_;}
  unsigned int targetDepth() const {return target_depth_;}
  unsigned int rejected() const {return rejected_;}
  bool hermite() const {return cfg_.hermite;}
  /// Hermite mode: depth setpoint in cycles (target_periods x nominal interval).
  unsigned int targetCycles() const {return target_cycles_;}
  /// Hermite mode: samples appended by the last successful pushSamples().
  unsigned int lastSampleCount() const {return last_count_;}
  /// Hermite mode: longest segment absorbed in one push [cycles] (a longer loop
  /// stall is time-compressed into this many samples).
  unsigned int maxSegmentCycles() const {return static_cast<unsigned int>(taus_.size());}
  const cyclic::StreamTiming & timing() const {return timing_;}

private:
  /// Fill `cmds` with an AbsolutePos hold at each axis' current command
  /// position, spanning `interval` cycles.
  bool holdCommands(
    wmx3Api::CyclicBufferMultiAxisCommands * cmds, unsigned int interval, std::string * error);

  /// Free slots on the axis with the least room, and the shallowest depth.
  bool queryBuffer(unsigned int * min_free, unsigned int * min_depth, std::string * error);

  void setError(std::string * error, const char * what, int code) const;

  wmx3Api::WMX3Api * wmx_ = nullptr;
  wmx3Api::CoreMotion * cm_ = nullptr;
  std::unique_ptr<wmx3Api::CyclicBuffer> cb_;

  Config cfg_;
  wmx3Api::AxisSelection sel_{};
  cyclic::StreamTiming timing_;
  std::unique_ptr<cyclic::PeriodTracker> period_;

  bool open_ = false;
  bool running_ = false;
  unsigned int target_depth_ = 1u;
  unsigned int depth_ = 0u;
  unsigned int rejected_ = 0u;
  wmx3Api::CyclicBufferState::T state_ = wmx3Api::CyclicBufferState::T::Stopped;

  // [local 2026-10-13] Hermite mode
  unsigned int target_cycles_ = 1u;
  unsigned int last_count_ = 0u;
  std::vector<double> seg_p_;   ///< end of the last appended segment, per axis
  std::vector<double> seg_v_;
  double next_tau_ = 0.0;       ///< time of the next grid sample after the segment start [s]
  double pending_s_ = 0.0;      ///< time of rejected pushes not yet covered by samples [s]
  bool stalled_ = false;        ///< latching: a segment exceeded one push (loop stall)
  cyclic::GridRegulator reg_;
  std::vector<double> v1_scratch_;
  std::vector<double> taus_;    ///< preallocated sample times
  std::vector<wmx3Api::CyclicBufferMultiAxisCommands> block_;   ///< preallocated in open()
};

}  // namespace wmx_r2_control

#endif  // WMX_R2_CONTROL__WMX_CYCLIC_STREAM_HPP_
