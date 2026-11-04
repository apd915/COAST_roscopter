// RouteGate -- declines a replan that would send the vehicle somewhere meaningfully different
// NOW, while accepting the ones that merely refine the current route.
//
// The edge case that decides whether this works at all is the LEGITIMATE TURN. A flip-flop and a
// real 90-degree corner both look like "the route changed direction near the vehicle"; the
// difference is that in a real turn BOTH plans agree to turn, while in a flip-flop they disagree
// with each other. Getting that wrong wedges the vehicle at every corner, which is worse than the
// problem being solved.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "navigation/route_gate.hpp"

namespace roscopter
{
namespace
{
using V = Eigen::Vector3d;
constexpr double kZ = -4.0;
/// Goal far along +x at cruise altitude, as in the benchmark mission.
const V kGoal(105.0, 0.0, kZ);

RouteGate::Config cfg(double deg = 35.0, int bound = 10)
{
  RouteGate::Config c;
  c.reject_angle_deg = deg;
  c.lookahead_m = 3.0;
  c.max_consecutive_rejects = bound;
  return c;
}

std::vector<V> straight(double len) { return {V(0, 0, kZ), V(len, 0, kZ)}; }

/// Goes straight for `at`, then turns by `deg` in the xy-plane.
std::vector<V> turn_at(double at, double deg, double after = 10.0)
{
  const double r = deg * M_PI / 180.0;
  const V corner(at, 0, kZ);
  return {V(0, 0, kZ), corner, corner + V(std::cos(r), std::sin(r), 0) * after};
}
}  // namespace

// The feature must ship inert. reject_angle_deg = 0 is the default, and the A arm has to be
// bit-identical to having no gate compiled in.
TEST(RouteGate, ZeroAngleIsACompleteNoOp)
{
  RouteGate g(cfg(0.0));
  const auto d = g.evaluate(straight(20), turn_at(1.0, 170.0), V(0, 0, kZ), kGoal, false);
  EXPECT_TRUE(d.accept);
  EXPECT_EQ(g.evaluated(), 0u) << "a disabled gate still counted an evaluation";
}

TEST(RouteGate, StraightContinuationIsAccepted)
{
  RouteGate g(cfg());
  const auto d = g.evaluate(straight(20), straight(20), V(2, 0, kZ), kGoal, false);
  EXPECT_TRUE(d.accept);
  EXPECT_NEAR(d.angle_deg, 0.0, 1e-6);
}

// THE CASE THIS EXISTS FOR: two plans that disagree with each other about which way to go.
TEST(RouteGate, FlipFlopIsRejected)
{
  RouteGate g(cfg());
  const auto left = turn_at(1.0, +70.0);
  const auto right = turn_at(1.0, -70.0);
  const auto d = g.evaluate(left, right, V(0, 0, kZ), kGoal, false);
  EXPECT_FALSE(d.accept) << "measured only " << d.angle_deg << " deg between opposite branches";
  EXPECT_GT(d.angle_deg, 35.0);
}

// THE CASE MOST LIKELY TO BE WRONG. A real corner is not a flip-flop: both plans turn the same
// way, so the heading difference BETWEEN them stays near zero even though each one turns hard.
TEST(RouteGate, LegitimateNinetyDegreeTurnIsAccepted)
{
  RouteGate g(cfg());
  const auto current = turn_at(2.0, 90.0);
  const auto candidate = turn_at(2.0, 90.0);   // same corner, replanned
  const auto d = g.evaluate(current, candidate, V(1.0, 0, kZ), kGoal, false);
  EXPECT_TRUE(d.accept)
      << "a 90 deg corner both plans agree on was rejected at " << d.angle_deg
      << " deg -- the gate is comparing each route against straight-ahead rather than against "
         "the other route, and will wedge the vehicle at every corner";
  EXPECT_LT(d.angle_deg, 5.0);
}

// Corridor consumption: the vehicle advances past a turn waypoint, so the fresh plan starts
// AFTER the corner while the current route still contains it. Comparing corridor 0 axes would
// read a large jump; arc-length alignment must not.
TEST(RouteGate, AdvancingPastACornerIsNotADirectionChange)
{
  RouteGate g(cfg());
  const auto current = turn_at(2.0, 90.0);
  // Fresh plan from 3 m along -- already round the corner, heading +y.
  const V veh(2.0, 1.0, kZ);
  const std::vector<V> candidate{veh, veh + V(0, 9, 0)};
  const auto d = g.evaluate(current, candidate, veh, kGoal, false);
  EXPECT_TRUE(d.accept)
      << "advancing past a corner read as a " << d.angle_deg
      << " deg change; the vehicle projection onto the current route is not aligning the windows";
}

// A route that differs only far ahead is harmless -- the vehicle replans long before reaching it.
TEST(RouteGate, FarFieldDivergenceIsAccepted)
{
  RouteGate g(cfg());
  const auto current = straight(30);
  const auto candidate = turn_at(12.0, 80.0);   // well beyond the 3 m window
  const auto d = g.evaluate(current, candidate, V(0, 0, kZ), kGoal, false);
  EXPECT_TRUE(d.accept) << "far-field change rejected at " << d.angle_deg << " deg";
}

// Safety override: if the current trajectory is in collision, the replan avoiding the obstacle
// is exactly the one a directional filter would otherwise throw away.
TEST(RouteGate, ForcedAcceptBeatsDivergence)
{
  RouteGate g(cfg());
  const auto d = g.evaluate(turn_at(1.0, +70.0), turn_at(1.0, -70.0), V(0, 0, kZ), kGoal, /*force=*/true);
  EXPECT_TRUE(d.accept);
  EXPECT_EQ(g.forced(), 1u);
  EXPECT_EQ(g.rejected(), 0u);
}

// Without a bound, sustained rejection is just plan-level commitment again -- and brings back
// the stutter this design exists to avoid.
TEST(RouteGate, RejectionIsBounded)
{
  RouteGate g(cfg(35.0, /*bound=*/3));
  const auto left = turn_at(1.0, +70.0);
  const auto right = turn_at(1.0, -70.0);
  int accepts = 0;
  for (int i = 0; i < 8; ++i) {
    if (g.evaluate(left, right, V(0, 0, kZ), kGoal, false).accept) ++accepts;
  }
  EXPECT_GT(accepts, 0) << "the gate rejected every candidate; the bound never fired";
  EXPECT_LE(g.longest_reject_streak(), 3u);
  EXPECT_GE(g.hit_bound(), 1u);
}

// With no current route there is nothing to compare against, and the first plan after takeoff
// must not be declined.
TEST(RouteGate, NoCurrentRouteAccepts)
{
  RouteGate g(cfg());
  const auto d = g.evaluate({}, straight(20), V(0, 0, kZ), kGoal, false);
  EXPECT_TRUE(d.accept);
  EXPECT_EQ(g.rejected(), 0u);
}

// Angles are recorded for ACCEPTED candidates too. If the gate never fires, the distribution is
// what says the threshold is wrong -- without it a silent no-op is indistinguishable from a
// well-behaved planner.
TEST(RouteGate, AnglesAreRecordedEvenWhenAccepted)
{
  RouteGate g(cfg(80.0));   // deliberately too loose to ever reject here
  g.evaluate(turn_at(1.0, +40.0), turn_at(1.0, -40.0), V(0, 0, kZ), kGoal, false);
  g.evaluate(straight(20), straight(20), V(0, 0, kZ), kGoal, false);
  EXPECT_EQ(g.rejected(), 0u);
  EXPECT_GT(g.angle_percentile(0.9), 10.0)
      << "no angle distribution recorded, so a mis-set threshold would look like a quiet planner";
}

// Too little route left to judge must accept rather than reject by default -- declining on
// missing information would stall the vehicle at the end of every plan.
TEST(RouteGate, TooShortToCompareAccepts)
{
  RouteGate g(cfg());
  const auto d = g.evaluate(straight(20), {V(0, 0, kZ), V(0.2, 0, kZ)}, V(0, 0, kZ), kGoal, false);
  EXPECT_TRUE(d.accept);
}

// The realistic shape of a replan: the candidate BEGINS at the vehicle rather than sharing the
// current route's origin. Both framings must give the same verdict on the same corner.
TEST(RouteGate, CandidateStartingAtTheVehicleAgreesWithOneSharingTheOrigin)
{
  RouteGate g(cfg());
  const auto current = turn_at(2.0, 90.0);
  const V veh(1.0, 0, kZ);
  const std::vector<V> candidate{veh, V(2, 0, kZ), V(2, 10, kZ)};   // same corner, from the vehicle
  const auto d = g.evaluate(current, candidate, veh, kGoal, false);
  EXPECT_TRUE(d.accept) << "rejected at " << d.angle_deg << " deg";
  EXPECT_LT(d.angle_deg, 5.0);
}

// The window is sampled ACROSS its whole span, not just at its far end. A divergence that
// begins inside the window but has been rejoined by the lookahead distance -- a short detour
// around a trunk, which is exactly what a junction flip-flop looks like at 8 m/s -- would be
// invisible to a single-point comparison and the gate would silently pass every one of them.
TEST(RouteGate, DivergenceInsideTheWindowIsCaughtNotJustAtItsEnd)
{
  RouteGate g(cfg());
  const auto current = straight(20);
  // Peels off, then comes back so that at the 3 m lookahead point the headings match again.
  // The detour must be fully REJOINED before the lookahead point, or a single-point
  // comparison would still land mid-detour and the test would not discriminate.
  const std::vector<V> candidate{
    V(0, 0, kZ), V(0.5, 0.8, kZ), V(1.5, 0.0, kZ), V(20.0, 0.0, kZ)};
  const auto d = g.evaluate(current, candidate, V(0, 0, kZ), kGoal, false);
  EXPECT_FALSE(d.accept)
      << "a detour that rejoins before the lookahead point measured only " << d.angle_deg
      << " deg -- the comparison is sampling one point instead of the whole window";
}

// THE SELF-DEFENDING BAD ROUTE. previous_route_ only updates on an accepted plan, so one bad
// plan getting through makes the gate defend it: every correction afterwards diverges from it
// and is declined for up to max_consecutive_rejects cycles. Observed in flight as the vehicle
// departing diagonally UPWARD and holding it while the planner commanded level flight.
//
// A candidate better aimed at the goal must always be accepted, however far it diverges.
TEST(RouteGate, CorrectionAwayFromABadRouteIsNeverDeclined)
{
  RouteGate g(cfg());
  // Currently climbing steeply; the goal is level and far along +x.
  const std::vector<V> climbing{V(0, 0, kZ), V(3, 0, kZ - 3.0), V(6, 0, kZ - 6.0)};
  const std::vector<V> level{V(0, 0, kZ), V(20, 0, kZ)};
  const auto d = g.evaluate(climbing, level, V(0, 0, kZ), kGoal, false);
  EXPECT_TRUE(d.accept)
      << "the level correction was declined at " << d.angle_deg
      << " deg -- the gate is defending a route that points away from the goal, which is how a "
         "single bad plan turns into a sustained climb";
  EXPECT_EQ(g.better_aimed(), 1u);
  EXPECT_EQ(g.rejected(), 0u);
}

// And the override must NOT dissolve flip-flop suppression: two branches of a genuine junction
// are comparably goal-ward, so neither is meaningfully better and the tie is still declined.
TEST(RouteGate, JunctionTieIsStillDeclined)
{
  RouteGate g(cfg());
  const auto left = turn_at(1.0, +70.0);
  const auto right = turn_at(1.0, -70.0);
  const auto d = g.evaluate(left, right, V(0, 0, kZ), kGoal, false);
  EXPECT_FALSE(d.accept)
      << "a symmetric junction tie was accepted as 'better aimed' (" << d.reason
      << ") -- the alignment margin is too loose and the gate no longer suppresses flip-flops";
}

// A candidate aimed WORSE than the current route must still be declined when it diverges.
TEST(RouteGate, WorseAimedCandidateIsStillDeclined)
{
  RouteGate g(cfg());
  const std::vector<V> level{V(0, 0, kZ), V(20, 0, kZ)};
  const std::vector<V> backwards{V(0, 0, kZ), V(-10, 0, kZ)};
  const auto d = g.evaluate(level, backwards, V(0, 0, kZ), kGoal, false);
  EXPECT_FALSE(d.accept) << "a route reversing away from the goal was accepted";
}

// NOTE: a test asserting the bound WITHHOLDS release for candidates aimed away from the goal
// was written here and removed with the behaviour it covered. Withholding recreated plan-level
// commitment -- rejection streaks reached 45, T_trav 146 -> 312 s, L_path 170 -> 403 m. The
// bound releases unconditionally; see the note at bound_reached in route_gate.hpp.

// And it must still release for a COMPARABLE candidate, or it is not a bound at all and
// sustained rejection becomes the plan-level commitment it exists to prevent.
TEST(RouteGate, BoundStillReleasesAComparablyAimedCandidate)
{
  RouteGate g(cfg(35.0, /*bound=*/3));
  const auto left = turn_at(1.0, +70.0);
  const auto right = turn_at(1.0, -70.0);   // symmetric: neither is better aimed
  int accepts = 0;
  for (int i = 0; i < 8; ++i) {
    if (g.evaluate(left, right, V(0, 0, kZ), kGoal, false).accept) ++accepts;
  }
  EXPECT_GT(accepts, 0);
  EXPECT_GE(g.hit_bound(), 1u);
}

}  // namespace roscopter
