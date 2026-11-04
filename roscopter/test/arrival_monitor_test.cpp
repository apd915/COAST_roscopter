// Scenarios the waypoint-arrival backstop must and must not fire on.
//
// This is the component most likely to misfire in the field and the hardest to exercise in flight:
// firing when it should not skips a waypoint mid-approach, and failing to fire hangs the mission
// until the battery ends it. Both are cheap to cover on the bench, so they are covered here.

#include <gtest/gtest.h>

#include <cmath>

#include "navigation/arrival_monitor.hpp"

using roscopter::ArrivalMonitor;

namespace
{

constexpr double kDispatchDt = 0.02;   // 50 Hz, matching MinimumSnapManager::dispatch_loop
constexpr double kHoverJitter = 0.02;  // residual speed of a vehicle holding position

enum class Outcome { Fallback, NominalArrival, NeverFired };

struct Result
{
  Outcome outcome = Outcome::NeverFired;
  double t = 0.0;      ///< seconds from the start of the run
  double dist = 0.0;   ///< distance when the run ended
};

/**
 * Fly a vehicle in from @p d0 at @p closing_speed until it reaches @p hold_dist, then hold there.
 * hold_dist == 0 means it flies all the way to the waypoint.
 */
Result run(ArrivalMonitor mon, double d0, double closing_speed, double hold_dist,
           double duration, bool active = true, double t0 = 0.0)
{
  mon.reset(t0);
  double dist = d0;
  for (double t = t0; t < t0 + duration; t += kDispatchDt) {
    double speed = closing_speed;
    if (dist <= hold_dist) {
      speed = kHoverJitter;
    } else {
      dist = std::max(hold_dist, dist - closing_speed * kDispatchDt);
    }

    if (dist < mon.radius) { return {Outcome::NominalArrival, t - t0, dist}; }
    if (mon.update(dist, speed, active, t)) { return {Outcome::Fallback, t - t0, dist}; }
  }
  return {Outcome::NeverFired, duration, dist};
}

}  // namespace

// An approach that actually reaches the waypoint must be accepted on the nominal radius, with the
// backstop never involved.
TEST(ArrivalMonitor, NormalApproachArrivesNominally)
{
  const Result r = run(ArrivalMonitor{}, 8.0, 1.5, 0.0, 30.0);
  EXPECT_EQ(r.outcome, Outcome::NominalArrival);
}

// The case this exists for: stopped short of the radius, inside the ceiling, going nowhere.
TEST(ArrivalMonitor, StallInsideCeilingFires)
{
  ArrivalMonitor mon;
  const Result r = run(mon, 8.0, 1.5, 1.4, 60.0);
  ASSERT_EQ(r.outcome, Outcome::Fallback);
  EXPECT_GT(r.dist, mon.radius) << "must not pre-empt the nominal radius test";
  EXPECT_LT(r.dist, mon.fallback_radius);
}

// Just inside the ceiling still counts.
TEST(ArrivalMonitor, StallJustInsideCeilingFires)
{
  const Result r = run(ArrivalMonitor{}, 8.0, 1.5, 2.4, 60.0);
  EXPECT_EQ(r.outcome, Outcome::Fallback);
}

// Beyond the ceiling nothing is ever skipped -- that bound is the whole safety argument for
// declaring arrival without having arrived.
TEST(ArrivalMonitor, StallBeyondCeilingNeverFires)
{
  const Result r = run(ArrivalMonitor{}, 8.0, 1.5, 3.5, 120.0);
  EXPECT_EQ(r.outcome, Outcome::NeverFired);
}

// Regression: a vehicle creeping in below stall_speed IS making progress and must be left alone.
// Re-anchoring best_dist to the current distance every cycle (a running minimum) defeats the
// epsilon test and skips the waypoint mid-approach -- measured at 2.00 m, 4.0 s into a closing
// approach, which on hardware is an ordinary cautious final approach.
TEST(ArrivalMonitor, SlowApproachIsNotAStall)
{
  ArrivalMonitor mon;
  ASSERT_LT(0.10, mon.stall_speed) << "scenario is only meaningful below the stall speed";
  const Result r = run(mon, 2.4, 0.10, 0.0, 120.0);
  EXPECT_EQ(r.outcome, Outcome::NominalArrival);
}

// A safety pilot on the sticks must neither accumulate stall time nor bank it for later.
TEST(ArrivalMonitor, RcOverrideNeverFires)
{
  const Result r = run(ArrivalMonitor{}, 8.0, 1.5, 1.4, 120.0, /*active=*/false);
  EXPECT_EQ(r.outcome, Outcome::NeverFired);
}

// Handing back control after a long RC excursion must restart the window, not fire immediately.
TEST(ArrivalMonitor, StallTimeIsNotBankedDuringRcOverride)
{
  ArrivalMonitor mon;
  mon.reset(0.0);
  for (double t = 0.0; t < 60.0; t += kDispatchDt) {
    ASSERT_FALSE(mon.update(1.4, 0.0, /*active=*/false, t));
  }
  EXPECT_FALSE(mon.update(1.4, 0.0, true, 60.0)) << "fired the instant offboard came back";
}

// Hardware runs on wall-clock epoch time, not a sim clock starting at zero.
TEST(ArrivalMonitor, LargeAbsoluteClockBaseBehavesIdentically)
{
  const Result sim = run(ArrivalMonitor{}, 8.0, 1.5, 1.4, 60.0, true, 0.0);
  const Result hw = run(ArrivalMonitor{}, 8.0, 1.5, 1.4, 60.0, true, 1.7e9);
  ASSERT_EQ(sim.outcome, Outcome::Fallback);
  ASSERT_EQ(hw.outcome, Outcome::Fallback);
  EXPECT_NEAR(sim.t, hw.t, 2 * kDispatchDt);
}

// A system clock stepping backwards (GPS time lock, NTP correction) must restart the window rather
// than produce a nonsensical elapsed time.
TEST(ArrivalMonitor, BackwardsClockJumpIsAbsorbed)
{
  ArrivalMonitor mon;
  mon.reset(100.0);
  for (double t = 100.0; t < 103.0; t += kDispatchDt) {
    ASSERT_FALSE(mon.update(1.4, 0.0, true, t)) << "fired before stall_time elapsed";
  }
  EXPECT_FALSE(mon.update(1.4, 0.0, true, 50.0));
}

// A forwards jump larger than the guard window must also restart rather than fire on the step.
TEST(ArrivalMonitor, LargeForwardClockJumpIsAbsorbed)
{
  ArrivalMonitor mon;
  mon.reset(100.0);
  ASSERT_FALSE(mon.update(1.4, 0.0, true, 100.0));
  EXPECT_FALSE(mon.update(1.4, 0.0, true, 100.0 + 10.0 * mon.stall_time + 1.0));
}

// Setting the ceiling at or below the nominal radius must disable the backstop outright -- the
// documented escape hatch for a rig where skipping a waypoint is never acceptable.
TEST(ArrivalMonitor, DisabledByCeilingAtOrBelowRadius)
{
  ArrivalMonitor mon;
  mon.fallback_radius = mon.radius;
  ASSERT_TRUE(mon.disabled());
  mon.reset(0.0);
  for (double t = 0.0; t < 120.0; t += kDispatchDt) {
    ASSERT_FALSE(mon.update(1.4, 0.0, true, t));
  }
}

// Each waypoint gets its own closest approach; carrying the previous one's over would leave the
// detector already armed the moment the next waypoint activates.
TEST(ArrivalMonitor, ResetRearmsForTheNextWaypoint)
{
  ArrivalMonitor mon;
  mon.reset(0.0);
  double t = 0.0;
  for (; t < 10.0; t += kDispatchDt) {
    if (mon.update(1.4, 0.0, true, t)) { break; }
  }
  ASSERT_LT(t, 10.0) << "precondition: the first waypoint stalls";

  mon.reset(t);
  EXPECT_FALSE(mon.update(1.4, 0.0, true, t)) << "still armed from the previous waypoint";
  EXPECT_FALSE(mon.update(1.4, 0.0, true, t + mon.stall_time - kDispatchDt));
}

// Backing away and returning must re-earn the timer rather than inherit the old stall window.
TEST(ArrivalMonitor, RetreatAndReapproachRearms)
{
  ArrivalMonitor mon;
  mon.reset(0.0);
  double t = 0.0;
  for (; t < 6.0; t += kDispatchDt) { mon.update(2.0, 0.0, true, t); }

  // Move meaningfully closer: that is progress, so the window restarts from here.
  ASSERT_FALSE(mon.update(1.5, 0.5, true, t));
  EXPECT_FALSE(mon.update(1.5, 0.0, true, t + mon.stall_time - kDispatchDt));
  EXPECT_TRUE(mon.update(1.5, 0.0, true, t + mon.stall_time));
}
