// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// Unit tests for the WMX-free cyclic-buffer stream timing logic. No WMX3 SDK
// required, so these run on any machine and in CI.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <deque>
#include <limits>
#include <random>
#include <vector>

#include "wmx_r2_control/cyclic_stream_timing.hpp"

using wmx_r2_control::cyclic::StreamTiming;
using wmx_r2_control::cyclic::nextInterval;

namespace
{

/// Default rig: 0.5 ms engine cycle, 100 Hz ros2_control loop.
/// nominal = 10 / 0.5 = 20 cycles, span = 25% = 5, so the band is [15, 25].
constexpr double kCycleMs = 0.5;
constexpr double kPushMs = 10.0;
constexpr unsigned int kNominal = 20u;
constexpr unsigned int kLo = 15u;
constexpr unsigned int kHi = 25u;

StreamTiming makeTiming(double cycle_ms = kCycleMs, double push_period_ms = kPushMs)
{
  StreamTiming t;
  t.cycle_ms = cycle_ms;
  t.push_period_ms = push_period_ms;
  return t;
}

}  // namespace

// ===========================================================================
// nominalInterval
// ===========================================================================

TEST(StreamTiming, NominalIntervalDividesPushPeriodByCycle)
{
  EXPECT_EQ(makeTiming(0.5, 10.0).nominalInterval(), 20u);   // 100 Hz loop
  EXPECT_EQ(makeTiming(0.5, 25.0).nominalInterval(), 50u);   // Servo's 40 Hz
  EXPECT_EQ(makeTiming(1.0, 10.0).nominalInterval(), 10u);   // 1 ms engine cycle
  EXPECT_EQ(makeTiming(0.25, 10.0).nominalInterval(), 40u);  // 0.25 ms engine cycle
  EXPECT_EQ(makeTiming(0.5, 2.0).nominalInterval(), 4u);     // 500 Hz loop
}

TEST(StreamTiming, NominalIntervalNeverZero)
{
  // A push period shorter than the engine cycle cannot be honoured; the command
  // must still span at least one cycle.
  EXPECT_EQ(makeTiming(10.0, 2.0).nominalInterval(), 1u);
  EXPECT_EQ(makeTiming(0.5, 0.4).nominalInterval(), 1u);
  EXPECT_EQ(makeTiming(0.5, 0.5).nominalInterval(), 1u);
}

// ===========================================================================
// Validation: an unset or nonsense StreamTiming must degrade to a hold, never
// to a plausible-looking wrong interval.
// ===========================================================================

TEST(StreamTiming, DefaultConstructedIsInvalid)
{
  const StreamTiming t;  // periods unset
  EXPECT_FALSE(t.valid());
  EXPECT_EQ(t.nominalInterval(), 1u);
  EXPECT_EQ(t.bufferCommands(5.0), 1u);
}

TEST(StreamTiming, RejectsNonFiniteAndNonPositivePeriods)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(makeTiming(nan, 10.0).valid());
  EXPECT_FALSE(makeTiming(0.5, nan).valid());
  EXPECT_FALSE(makeTiming(inf, 10.0).valid());
  EXPECT_FALSE(makeTiming(0.0, 10.0).valid());   // engine not communicating yet
  EXPECT_FALSE(makeTiming(-0.5, 10.0).valid());
  EXPECT_FALSE(makeTiming(0.5, 0.0).valid());
  EXPECT_TRUE(makeTiming().valid());
}

TEST(StreamTiming, RejectsGainsOutsideTheStableRange)
{
  StreamTiming t = makeTiming();
  t.depth_gain = 0.0;
  EXPECT_FALSE(t.valid());
  t.depth_gain = 2.0;  // loop is marginally stable at 2, unstable above
  EXPECT_FALSE(t.valid());
  t.depth_gain = 1.0;  // deadbeat, still valid
  EXPECT_TRUE(t.valid());

  t = makeTiming();
  t.tolerance = -0.1;
  EXPECT_FALSE(t.valid());
  t.tolerance = 1.5;
  EXPECT_FALSE(t.valid());
  t.tolerance = 0.0;  // no adjustment allowed, but well-defined
  EXPECT_TRUE(t.valid());
}

TEST(StreamTiming, InvalidTimingYieldsSingleCycleCommands)
{
  const StreamTiming t;
  for (unsigned int depth = 0; depth < 10u; ++depth) {
    EXPECT_EQ(nextInterval(t, 2u, depth), 1u) << "depth=" << depth;
  }
}

// ===========================================================================
// targetDepth / bufferCommands
// ===========================================================================

TEST(StreamTiming, TargetDepthIsCommandsOfRunway)
{
  const StreamTiming t = makeTiming();
  EXPECT_EQ(t.targetDepth(), 2u);    // default
  EXPECT_EQ(t.targetDepth(4u), 4u);
  EXPECT_EQ(t.targetDepth(0u), 1u);  // zero runway would underrun by design
}

TEST(StreamTiming, BufferCommandsCountsCommandsNotCycles)
{
  // 5 s at 10 ms per command = 500 commands, whatever the engine cycle is:
  // intervalCycles does not change a command's one-slot footprint.
  EXPECT_EQ(makeTiming(0.5, 10.0).bufferCommands(5.0), 500u);
  EXPECT_EQ(makeTiming(0.25, 10.0).bufferCommands(5.0), 500u);
  EXPECT_EQ(makeTiming(1.0, 10.0).bufferCommands(5.0), 500u);

  // Reasoning in cycles instead would ask the engine for 10x this at 0.25 ms.
  EXPECT_EQ(
    makeTiming(0.25, 10.0).bufferCommands(5.0),
    makeTiming(1.0, 10.0).bufferCommands(5.0));
}

TEST(StreamTiming, BufferCommandsRejectsNonsenseHorizons)
{
  const StreamTiming t = makeTiming();
  EXPECT_EQ(t.bufferCommands(0.0), 1u);
  EXPECT_EQ(t.bufferCommands(-5.0), 1u);
  EXPECT_EQ(t.bufferCommands(std::numeric_limits<double>::quiet_NaN()), 1u);
}

TEST(StreamTiming, RunwayMsScalesWithDepth)
{
  const StreamTiming t = makeTiming();
  EXPECT_DOUBLE_EQ(t.runwayMs(0u), 0.0);
  EXPECT_DOUBLE_EQ(t.runwayMs(2u), 20.0);
  EXPECT_DOUBLE_EQ(t.runwayMs(5u), 50.0);
}

// ===========================================================================
// nextInterval: direction, clamping, monotonicity
// ===========================================================================

TEST(NextInterval, AtSetpointReturnsNominal)
{
  EXPECT_EQ(nextInterval(makeTiming(), 2u, 2u), kNominal);
  EXPECT_EQ(nextInterval(makeTiming(), 5u, 5u), kNominal);
}

// The sign here is the bug that would produce a runaway: stretching when the
// buffer is already long would grow the queue without bound.
TEST(NextInterval, BelowSetpointStretches)
{
  const StreamTiming t = makeTiming();
  EXPECT_GT(nextInterval(t, 2u, 1u), kNominal);
  EXPECT_GT(nextInterval(t, 2u, 0u), kNominal);
  // Saturates at the clamp for a one-command error: 0.5 * 20 * 1 = 10 -> +5.
  EXPECT_EQ(nextInterval(t, 2u, 1u), kHi);
  EXPECT_EQ(nextInterval(t, 2u, 0u), kHi);
}

TEST(NextInterval, AboveSetpointCompresses)
{
  const StreamTiming t = makeTiming();
  EXPECT_LT(nextInterval(t, 2u, 3u), kNominal);
  EXPECT_LT(nextInterval(t, 2u, 50u), kNominal);
  EXPECT_EQ(nextInterval(t, 2u, 3u), kLo);
  EXPECT_EQ(nextInterval(t, 2u, 50u), kLo);
}

TEST(NextInterval, AlwaysWithinToleranceBandAndNeverZero)
{
  const StreamTiming t = makeTiming();
  for (unsigned int depth = 0; depth <= 1000u; ++depth) {
    const unsigned int interval = nextInterval(t, 2u, depth);
    EXPECT_GE(interval, kLo) << "depth=" << depth;
    EXPECT_LE(interval, kHi) << "depth=" << depth;
    EXPECT_GT(interval, 0u) << "depth=" << depth;
  }
}

TEST(NextInterval, HandlesExtremeDepthsWithoutOverflow)
{
  const StreamTiming t = makeTiming();
  EXPECT_EQ(nextInterval(t, 2u, std::numeric_limits<unsigned int>::max()), kLo);
  EXPECT_EQ(nextInterval(t, std::numeric_limits<unsigned int>::max(), 0u), kHi);
}

TEST(NextInterval, IsMonotonicNonIncreasingInDepth)
{
  const StreamTiming t = makeTiming();
  unsigned int previous = nextInterval(t, 3u, 0u);
  for (unsigned int depth = 1u; depth <= 50u; ++depth) {
    const unsigned int interval = nextInterval(t, 3u, depth);
    EXPECT_LE(interval, previous) << "depth=" << depth;
    previous = interval;
  }
}

// When the loop period equals the engine cycle there is no room to interpolate,
// so the regulator has no authority at all. Documented as a test so nobody tunes
// against a knob that cannot move.
TEST(NextInterval, HasNoAuthorityWhenNominalIntervalIsOne)
{
  const StreamTiming t = makeTiming(0.5, 0.5);
  ASSERT_EQ(t.nominalInterval(), 1u);
  EXPECT_EQ(nextInterval(t, 2u, 0u), 1u);
  EXPECT_EQ(nextInterval(t, 2u, 9u), 1u);
}

// ===========================================================================
// Closed loop against a FIFO model of the buffer
// ===========================================================================

namespace
{

struct SimResult
{
  unsigned int min_depth = std::numeric_limits<unsigned int>::max();
  unsigned int max_depth = 0u;
  unsigned int final_depth = 0u;
  int underruns = 0;      ///< pushes after which the buffer ran dry (axis held)
  double mean_depth = 0.0;
};

/// Simulate the real buffer: a FIFO of commands, each with its own remaining
/// cycle count, drained by however many cycles elapse between pushes.
///
/// `initial_depth` is how many commands are queued before the first push, which
/// start() normally primes to `target` but which can be higher after a burst of
/// late writes. `regulate == false` pushes a fixed nominal interval, i.e. the
/// naive implementation with no depth control.
SimResult simulate(
  const StreamTiming & timing, unsigned int target, unsigned int initial_depth, int pushes,
  double jitter_frac, std::uint32_t seed, bool regulate)
{
  const unsigned int nominal = timing.nominalInterval();
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> jitter(1.0 - jitter_frac, 1.0 + jitter_frac);

  std::deque<std::int64_t> queue(initial_depth, static_cast<std::int64_t>(nominal));

  SimResult result;
  double sum = 0.0;

  for (int k = 0; k < pushes; ++k) {
    // push() reads the depth, then appends -- same order as the real call.
    const unsigned int depth = static_cast<unsigned int>(queue.size());
    result.min_depth = std::min(result.min_depth, depth);
    result.max_depth = std::max(result.max_depth, depth);
    sum += depth;

    queue.push_back(
      regulate ? nextInterval(timing, target, depth) : nominal);

    // Consume the cycles that elapse before the next push lands.
    std::int64_t budget = std::llround(nominal * jitter(rng));
    while (budget > 0 && !queue.empty()) {
      const std::int64_t take = std::min(budget, queue.front());
      queue.front() -= take;
      budget -= take;
      if (queue.front() == 0) {queue.pop_front();}
    }
    if (queue.empty()) {++result.underruns;}
  }

  result.final_depth = static_cast<unsigned int>(queue.size());
  result.mean_depth = sum / pushes;
  return result;
}

}  // namespace

TEST(ClosedLoop, HoldsDepthWithoutUnderrunUnderModerateJitter)
{
  const StreamTiming t = makeTiming();
  const unsigned int target = t.targetDepth();  // 2 commands
  const SimResult r = simulate(t, target, target, 2000, 0.20, 1234u, true);

  EXPECT_EQ(r.underruns, 0);
  EXPECT_GE(r.min_depth, 1u);
  // Settles into a one-command band around the setpoint: the deadband means it
  // dithers between target-1 and target+1, averaging the setpoint.
  EXPECT_LE(r.max_depth, target + 1u);
  EXPECT_NEAR(r.mean_depth, static_cast<double>(target), 0.25);
}

TEST(ClosedLoop, SurvivesSevereJitterWithMoreRunway)
{
  const StreamTiming t = makeTiming();
  const unsigned int target = t.targetDepth(4u);
  const SimResult r = simulate(t, target, target, 2000, 0.50, 4321u, true);

  EXPECT_EQ(r.underruns, 0);
  EXPECT_GE(r.min_depth, 1u);
  EXPECT_LE(r.max_depth, target + 4u);
}

// The regulator has to earn its place. With a fixed nominal interval the queued
// cycle count is a zero-drift random walk, so the buffer wanders off in one of
// two directions and which one you get depends on the noise: down to empty (the
// axis stalls, then steps) or up without bound (commanded motion falls further
// and further behind the operator). Both are disqualifying, so assert across
// several seeds rather than pinning one seed to one failure mode.
TEST(ClosedLoop, FixedIntervalDriftsWhereRegulatedHolds)
{
  const StreamTiming t = makeTiming();
  const unsigned int target = t.targetDepth();
  const std::uint32_t seeds[] = {99u, 1234u, 7u, 555u};

  int fixed_underruns = 0;
  unsigned int fixed_worst_depth = 0;
  unsigned int regulated_worst_depth = 0;

  for (const std::uint32_t seed : seeds) {
    const SimResult fixed = simulate(t, target, target, 2000, 0.20, seed, false);
    const SimResult regulated = simulate(t, target, target, 2000, 0.20, seed, true);

    fixed_underruns += fixed.underruns;
    fixed_worst_depth = std::max(fixed_worst_depth, fixed.max_depth);
    regulated_worst_depth = std::max(regulated_worst_depth, regulated.max_depth);

    // The regulated loop must be well-behaved on *every* seed.
    EXPECT_EQ(regulated.underruns, 0) << "seed=" << seed;
    EXPECT_LE(regulated.max_depth, target + 1u) << "seed=" << seed;
  }

  // Underrun failure mode: some seeds drain the buffer dry.
  EXPECT_GT(fixed_underruns, 0);
  // Latency failure mode: others pile up many push-periods of stale commands.
  EXPECT_GE(fixed_worst_depth, 3u * regulated_worst_depth);
}

// A burst of late writes leaves the queue deep, which is pure added latency:
// every queued command must execute before the operator's newest target does.
// The regulator has to shed it, not settle for it.
TEST(ClosedLoop, DrainsAnOverfilledBufferBackToSetpoint)
{
  const StreamTiming t = makeTiming();
  const unsigned int target = t.targetDepth();
  const SimResult r = simulate(t, target, target + 20u, 2000, 0.0, 7u, true);

  EXPECT_EQ(r.underruns, 0);
  EXPECT_LE(r.final_depth, target + 1u) << "regulator failed to shed latency";
  EXPECT_EQ(r.max_depth, target + 20u) << "depth should only ever fall from the start";
}

// The mirror case: starting empty, the stretch must fill the buffer to the
// setpoint rather than limping along one command deep forever.
TEST(ClosedLoop, FillsAnEmptyBufferUpToSetpoint)
{
  const StreamTiming t = makeTiming();
  const unsigned int target = t.targetDepth(4u);
  const SimResult r = simulate(t, target, 0u, 2000, 0.0, 8u, true);

  EXPECT_GE(r.final_depth, target) << "regulator failed to build runway";
}

// ===========================================================================
// PeriodTracker
// ===========================================================================

using wmx_r2_control::cyclic::PeriodTracker;

TEST(PeriodTracker, UsesSeedBeforeAnySample)
{
  const PeriodTracker tracker(10.0);
  EXPECT_DOUBLE_EQ(tracker.periodMs(), 10.0);
  EXPECT_EQ(tracker.samples(), 0u);
  EXPECT_FALSE(tracker.settled());
}

TEST(PeriodTracker, ConvergesTowardTheMeasuredPeriod)
{
  // Seeded at the nominal 10 ms but the loop actually runs at 11 ms.
  PeriodTracker tracker(10.0, 100u, 0.1);
  for (int i = 0; i < 100; ++i) {
    tracker.observe(11.0);
  }
  EXPECT_NEAR(tracker.periodMs(), 11.0, 0.01);
  EXPECT_TRUE(tracker.settled());
}

TEST(PeriodTracker, AveragesOutSymmetricJitter)
{
  PeriodTracker tracker(10.0, 100u, 0.1);
  for (int i = 0; i < 100; ++i) {
    tracker.observe(i % 2 == 0 ? 9.0 : 11.0);
  }
  EXPECT_NEAR(tracker.periodMs(), 10.0, 0.2);
}

// Once settled, a stall must not drag the estimate: stretching every subsequent
// command would silently add latency for the rest of the run.
TEST(PeriodTracker, FreezesAfterWarmup)
{
  PeriodTracker tracker(10.0, 10u, 0.5);
  for (int i = 0; i < 10; ++i) {
    tracker.observe(10.0);
  }
  ASSERT_TRUE(tracker.settled());
  const double settled = tracker.periodMs();

  for (int i = 0; i < 50; ++i) {
    tracker.observe(25.0);
  }
  EXPECT_DOUBLE_EQ(tracker.periodMs(), settled);
  EXPECT_EQ(tracker.samples(), 10u);
}

TEST(PeriodTracker, RejectsOutliersDuringWarmup)
{
  PeriodTracker tracker(10.0, 100u, 0.1);
  tracker.observe(100.0);  // 10x: a stall, not the loop rate
  tracker.observe(0.5);    // 1/20x: a clock glitch
  EXPECT_DOUBLE_EQ(tracker.periodMs(), 10.0);
  EXPECT_EQ(tracker.samples(), 0u);
  EXPECT_EQ(tracker.rejected(), 2u);

  tracker.observe(11.0);  // in band, accepted
  EXPECT_GT(tracker.periodMs(), 10.0);
  EXPECT_EQ(tracker.samples(), 1u);
}

TEST(PeriodTracker, IgnoresNonFiniteAndNonPositiveSamples)
{
  PeriodTracker tracker(10.0);
  tracker.observe(std::numeric_limits<double>::quiet_NaN());
  tracker.observe(std::numeric_limits<double>::infinity());
  tracker.observe(0.0);
  tracker.observe(-5.0);
  EXPECT_DOUBLE_EQ(tracker.periodMs(), 10.0);
  EXPECT_EQ(tracker.samples(), 0u);
}

// The tracked period feeds nominalInterval(), so a drifted loop must change the
// commanded interval -- that is the whole reason the tracker exists.
TEST(PeriodTracker, FeedsNominalIntervalOnceSettled)
{
  PeriodTracker tracker(10.0, 50u, 0.2);
  for (int i = 0; i < 50; ++i) {
    tracker.observe(12.5);
  }
  StreamTiming t = makeTiming();
  EXPECT_EQ(t.nominalInterval(), 20u);  // from the seeded 10 ms
  t.push_period_ms = tracker.periodMs();
  EXPECT_EQ(t.nominalInterval(), 25u);  // 12.5 ms / 0.5 ms
}

// ===========================================================================
// [local 2026-10-13] Hermite sub-sampling: hermite(), gridSamples(), GridRegulator
// ===========================================================================

using wmx_r2_control::cyclic::hermite;
using wmx_r2_control::cyclic::gridSamples;
using wmx_r2_control::cyclic::GridRegulator;

TEST(Hermite, HitsEndpointsAndEndSlopes)
{
  const double p0 = 0.3, v0 = -0.8, p1 = 0.35, v1 = 1.2, h = 0.010;
  EXPECT_DOUBLE_EQ(hermite(p0, v0, p1, v1, h, 0.0), p0);
  EXPECT_DOUBLE_EQ(hermite(p0, v0, p1, v1, h, 1.0), p1);
  const double ds = 1e-6;
  // d/dt = d/ds / h
  EXPECT_NEAR((hermite(p0, v0, p1, v1, h, ds) - p0) / (ds * h), v0, 1e-3);
  EXPECT_NEAR((p1 - hermite(p0, v0, p1, v1, h, 1.0 - ds)) / (ds * h), v1, 1e-3);
}

TEST(Hermite, ConstantVelocityIsAStraightLine)
{
  const double v = 1.5, h = 0.010, p0 = -0.2, p1 = p0 + v * h;
  for (int k = 0; k <= 10; ++k) {
    const double s = k / 10.0;
    EXPECT_NEAR(hermite(p0, v, p1, v, h, s), p0 + v * h * s, 1e-15);
  }
}

// The point of hermite mode: sampled once per engine cycle, a constant
// acceleration shows up as a constant per-cycle velocity change (no staircase).
TEST(Hermite, ConstantAccelerationHasNoVelocityStaircase)
{
  const double a = 6.0, cycle = 0.001;
  const unsigned int n = 10u;
  const double h = n * cycle;
  std::vector<double> samples;
  double t = 0.0, p = 0.0, v = 0.0;
  for (int seg = 0; seg < 5; ++seg) {
    const double p1 = p + v * h + 0.5 * a * h * h, v1 = v + a * h;
    for (unsigned int k = (seg == 0 ? 0u : 1u); k <= n; ++k) {
      samples.push_back(hermite(p, v, p1, v1, h, static_cast<double>(k) / n));
    }
    p = p1; v = v1; t += h;
  }
  // second difference / cycle^2 = acceleration seen per cycle
  for (size_t i = 2; i < samples.size(); ++i) {
    const double acc = (samples[i] - 2.0 * samples[i - 1] + samples[i - 2]) / (cycle * cycle);
    EXPECT_NEAR(acc, a, 1e-6) << "at sample " << i;
  }
}

TEST(GridSamples, CarriesTheRemainderAcrossSegments)
{
  double next_tau = 0.0;   // unset -> one step
  double taus[64];
  const double step = 0.001;
  // 10.4 ms then 9.6 ms: 10 samples, then 10 more, landing exactly on 20 ms in total
  unsigned int n1 = gridSamples(next_tau, 0.0104, step, taus, 64);
  EXPECT_EQ(n1, 10u);
  EXPECT_NEAR(taus[0], 0.001, 1e-12);
  EXPECT_NEAR(taus[9], 0.010, 1e-12);
  EXPECT_NEAR(next_tau, 0.0006, 1e-9);           // next grid point is 0.6 ms into segment 2
  unsigned int n2 = gridSamples(next_tau, 0.0096, step, taus, 64);
  EXPECT_EQ(n2, 10u);
  EXPECT_NEAR(taus[0], 0.0006, 1e-9);
  EXPECT_NEAR(taus[9], 0.0096, 1e-9);
}

TEST(GridSamples, LateLoopYieldsMoreSamplesAndStallIsCapped)
{
  double next_tau = 0.001, taus[200];
  EXPECT_EQ(gridSamples(next_tau, 0.015, 0.001, taus, 200), 15u);
  next_tau = 0.001;
  EXPECT_EQ(gridSamples(next_tau, 0.500, 0.001, taus, 80), 80u);   // capped
  next_tau = 0.001;
  EXPECT_EQ(gridSamples(next_tau, 0.0004, 0.001, taus, 80), 0u);   // shorter than a cycle
  EXPECT_NEAR(next_tau, 0.0006, 1e-9);
}

TEST(GridRegulator, SignClampAndRateLimit)
{
  GridRegulator r;
  EXPECT_DOUBLE_EQ(r.update(20u, 20u), 1.0);
  for (int i = 0; i < 1000; ++i) {r.update(20u, 1000u);}
  EXPECT_NEAR(r.scale, 1.0 + r.max_frac, 1e-12);     // deep -> larger step, clamped
  GridRegulator s;
  s.update(20u, 20u);
  const double before = s.scale;
  s.update(20u, 0u);                                   // a sudden underrun reading
  EXPECT_LE(std::abs(s.scale - before), s.max_rate + 1e-15);
  EXPECT_LT(s.scale, 1.0);
}

// Review 2026-10-13: regulating from each integer depth sample dithered the step
// and showed up as velocity kinks (0.19 rad/s^2 ramp read 0.75). Constant velocity
// through a jittery loop and an integer-depth engine model: the per-cycle
// acceleration the engine sees must stay within v * max_rate / cycle, and the
// queue must neither underrun nor run away.
TEST(GridRegulator, ConstantVelocityThroughJitteryLoopHasNoKinks)
{
  const double v = 2.0, cycle = 0.001;
  const unsigned int target = 20u;
  std::mt19937 rng(11u);
  std::normal_distribution<double> jitter(0.0, 0.001);   // sigma 1 ms on a 10 ms loop
  GridRegulator reg;
  double next_tau = 0.0, traj_t = 0.0, taus[200];
  std::deque<double> queue(target, 0.0);                  // primed holds at p = 0
  std::vector<double> played;
  double engine_debt = 0.0;
  for (int k = 0; k < 3000; ++k) {
    const double h = std::clamp(0.010 + jitter(rng), 0.004, 0.030);
    // engine plays one sample per cycle during the loop period
    engine_debt += h;
    while (engine_debt >= cycle && !queue.empty()) {
      played.push_back(queue.front());
      queue.pop_front();
      engine_debt -= cycle;
    }
    ASSERT_FALSE(queue.empty()) << "underrun at push " << k;
    const double step = cycle * reg.update(target, static_cast<unsigned int>(queue.size()));
    const unsigned int n = gridSamples(next_tau, h, step, taus, 200);
    for (unsigned int i = 0; i < n; ++i) {
      queue.push_back(v * (traj_t + taus[i]));             // constant-velocity trajectory
    }
    traj_t += h;
  }
  EXPECT_LT(queue.size(), static_cast<size_t>(target + 40));
  const double bound = v * reg.max_rate / cycle + 1e-6;
  for (size_t i = target + 2; i < played.size(); ++i) {
    const double acc = (played[i] - 2.0 * played[i - 1] + played[i - 2]) / (cycle * cycle);
    ASSERT_LE(std::abs(acc), bound) << "at cycle " << i;
  }
}

// End to end: a constant-acceleration trajectory sampled by a jittery 100 Hz loop
// (5..15 ms periods) and replayed on the 1 ms grid has the same per-cycle
// acceleration everywhere -- no staircase, no bending from the jitter.
TEST(GridSamples, JitteryLoopReplaysConstantAccelerationExactly)
{
  const double a = 6.0, step = 0.001;
  std::mt19937 rng(5u);
  std::uniform_real_distribution<double> period(0.005, 0.015);
  double t = 0.0, p0 = 0.0, v0 = 0.0, next_tau = 0.0;
  double taus[100];
  std::vector<double> samples{0.0};
  for (int k = 0; k < 300; ++k) {
    const double h = period(rng);
    const double t1 = t + h;
    const double p1 = 0.5 * a * t1 * t1, v1 = a * t1;
    const unsigned int n = gridSamples(next_tau, h, step, taus, 100);
    for (unsigned int i = 0; i < n; ++i) {
      samples.push_back(hermite(p0, v0, p1, v1, h, taus[i] / h));
    }
    t = t1; p0 = p1; v0 = v1;
  }
  ASSERT_GT(samples.size(), 1500u);
  for (size_t i = 2; i < samples.size(); ++i) {
    const double acc = (samples[i] - 2.0 * samples[i - 1] + samples[i - 2]) / (step * step);
    ASSERT_NEAR(acc, a, 1e-3) << "at sample " << i;
  }
}
