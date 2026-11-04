#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace roscopter
{

/**
 * Liveness backstop for waypoint arrival.
 *
 * Answers one question -- "the vehicle is close to the waypoint, it has stopped, and it is no
 * longer getting closer; should the mission keep waiting?" -- and is deliberately agnostic about
 * WHY, because the plausible causes are not distinguishable from inside the planner node and all
 * of them want the same answer.
 *
 * Note what does NOT cause a stall, contrary to the obvious guess: a biased position estimate.
 * Arrival compares the estimate against the waypoint's world coordinate, the controller drives the
 * estimate to the commanded position, and the planner's goal IS that world coordinate, so on a
 * successful plan the estimate converges and the nominal radius test fires normally. Estimator
 * bias changes where the vehicle TRULY ends up, which is a safety-margin question, not a liveness
 * one. What can actually stall an approach is a front end that cannot route to the goal (a real
 * obstacle, or one phantom-displaced because it was mapped at the estimate's earlier bias while
 * the vehicle now flies at a later one) or, on hardware, wind holding the vehicle off the point
 * with the controller saturated.
 *
 * Deliberately NOT gated on the planner's reached_goal verdict. Requiring the planner to report
 * the goal reachable would disarm the backstop in precisely the case it exists for: a search that
 * cannot reach the goal is the most likely way to hover indefinitely. `fallback_radius` is the
 * safety mechanism instead -- inside it the vehicle is close enough that giving up on the point is
 * low-risk, and the next waypoint is planned with full obstacle avoidance rather than flying
 * through anything. Beyond it nothing is ever skipped.
 *
 * Lives in its own header, free of ROS types, so the scenarios that must and must not fire are
 * covered by test/arrival_monitor_test.cpp against this exact code rather than a hand-copied
 * replica. This is the component most likely to misfire in the field and the one hardest to
 * exercise in flight, so it is worth being able to test on the bench.
 */
struct ArrivalMonitor
{
  /// Nominal arrival radius (m). Handled by the caller; kept here so `disabled()` can compare.
  double radius = 1.0;
  /// Ceiling (m) on declaring arrival once progress has stalled. <= radius disables the backstop.
  double fallback_radius = 2.5;
  /// Speed (m/s) below which the vehicle counts as no longer making way.
  double stall_speed = 0.35;
  /// How long (s) progress must fail to improve before arrival is declared anyway.
  double stall_time = 4.0;
  /// Improvement in closest approach (m) that counts as progress and rearms the timer.
  double progress_epsilon = 0.15;

  /// Last CONFIRMED progress mark. NOT a per-cycle running minimum -- see update().
  double best_dist = std::numeric_limits<double>::infinity();
  /// Clock reading at the last progress toward the active waypoint.
  double progress_stamp = 0.0;

  bool disabled() const { return fallback_radius <= radius; }

  /// Rearm for a newly activated waypoint. A fresh goal has its own closest approach; carrying the
  /// previous waypoint's over would leave the detector already armed on arrival.
  void reset(double now)
  {
    best_dist = std::numeric_limits<double>::infinity();
    progress_stamp = now;
  }

  /**
   * Advance one dispatch cycle. Returns true when arrival should be declared despite being outside
   * the nominal radius.
   *
   * @param dist    distance from the estimated position to the waypoint (m)
   * @param speed   vehicle speed magnitude (m/s). Estimated from GNSS velocity and the IMU rather
   *                than GNSS position, so it does not share the position channel's slow bias.
   * @param active  false while a safety pilot has the sticks. The timer neither runs nor survives
   *                then -- otherwise a long RC excursion would bank stall time and the backstop
   *                would fire the instant offboard control came back.
   * @param now     monotonic-ish clock reading (s); the node passes ROS time so it follows sim time.
   */
  bool update(double dist, double speed, bool active, double now)
  {
    if (disabled()) { return false; }

    // Not under autopilot control, or out of range: hold the timer at the current instant rather
    // than letting it accumulate.
    if (!active || dist >= fallback_radius) {
      progress_stamp = now;
      return false;
    }

    // best_dist only moves when the vehicle beats it by a full epsilon, and that is the whole
    // point: updating it to std::min(best_dist, dist) every call re-anchors the reference to
    // wherever the vehicle currently is, so the next comparison is always against ~dist and no
    // approach can ever clear the epsilon again. A vehicle creeping in at 0.10 m/s then reads as
    // stalled and has its waypoint skipped mid-approach (measured: fired at 2.00 m, 4.0 s into a
    // closing approach). A slow, deliberate final approach is normal on hardware. Covered by
    // ArrivalMonitor.SlowApproachIsNotAStall -- do not "simplify" this into a running minimum.
    if (dist < best_dist - progress_epsilon) {
      best_dist = dist;
      progress_stamp = now;
      return false;
    }

    if (speed >= stall_speed) {
      progress_stamp = now;
      return false;
    }

    // Clock discontinuity guard: a sim reset, or a hardware system clock stepping when GPS time
    // locks in, can move `now` backwards or far forwards. Either makes the elapsed figure
    // meaningless -- restart the window instead of acting on it.
    const double stalled_for = now - progress_stamp;
    if (stalled_for < 0.0 || stalled_for > std::max(30.0, 10.0 * stall_time)) {
      progress_stamp = now;
      return false;
    }

    return stalled_for >= stall_time;
  }
};

}  // namespace roscopter
