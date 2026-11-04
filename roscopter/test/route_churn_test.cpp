// RouteChurn -- measures whether consecutive plans disagree about where to go NEXT.
//
// This exists to settle a question we have guessed wrong on twice. Two route-commitment
// mechanisms were built on the assumption that the planner flip-flops in the near field, and both
// measured WORSE (the anti-stutter gate cost +31% travel time; hysteresis collapsed v_avg to
// 1.02 m/s). The measurement has to be trustworthy or the third attempt is another guess.
//
// The property that matters: a route that agrees near the vehicle and differs only far ahead must
// NOT be reported as near-field churn -- the vehicle replans long before reaching that far
// segment, so committing would change nothing. Getting that backwards would send us building the
// wrong fix.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <vector>

#include "navigation/route_churn.hpp"

namespace roscopter
{
namespace
{
using V = Eigen::Vector3d;
std::vector<V> straight(double len, double y = 0.0)
{
  return {V(0, y, -4), V(len / 2, y, -4), V(len, y, -4)};
}
}  // namespace

// Identical routes must read as unchanged, not as a tiny divergence at distance 0.
TEST(RouteChurn, IdenticalRoutesAreUnchanged)
{
  RouteChurn c(1.0);
  c.add(straight(20.0));
  c.add(straight(20.0));
  EXPECT_EQ(c.compared(), 1u);
  EXPECT_EQ(c.unchanged(), 1u);
  EXPECT_DOUBLE_EQ(c.unchanged_pct(), 100.0);
  EXPECT_EQ(c.near_field(), 0u);
}

// THE DISTINCTION THE DIAGNOSTIC EXISTS FOR, part 1: a route that peels off immediately is
// near-field churn -- the vehicle is being sent somewhere different RIGHT NOW.
TEST(RouteChurn, DivergingImmediatelyCountsAsNearField)
{
  RouteChurn c(1.0);
  c.add(straight(20.0, 0.0));
  c.add({V(0, 0, -4), V(2, 5, -4), V(20, 5, -4)});   // turns hard at once
  ASSERT_EQ(c.compared(), 1u);
  EXPECT_EQ(c.near_field(), 1u);
  EXPECT_LT(c.percentile(0.5), c.near_field_threshold());
}

// Part 2, and the one that would mislead us if wrong: routes that agree near the vehicle and
// differ only far ahead must NOT count as near-field. The vehicle never flies that far before
// replanning, so commitment would buy nothing.
TEST(RouteChurn, DivergingFarAheadIsNotNearField)
{
  RouteChurn c(1.0);
  c.add({V(0, 0, -4), V(10, 0, -4), V(30, 0, -4)});
  c.add({V(0, 0, -4), V(10, 0, -4), V(30, 12, -4)});   // identical for the first 10 m
  ASSERT_EQ(c.compared(), 1u);
  EXPECT_EQ(c.near_field(), 0u)
      << "a route that only differs far ahead was reported as near-field churn -- this would "
         "send us building route commitment for a problem commitment cannot fix";
  EXPECT_GT(c.percentile(0.5), c.near_field_threshold());
}

// Divergence is reported as arc length ALONG the new route, so it is comparable between plans
// with different corridor counts and different stitch points.
TEST(RouteChurn, DivergenceIsReportedAsArcLength)
{
  RouteChurn c(1.0);
  c.add({V(0, 0, -4), V(20, 0, -4)});
  // Departs the old line after about 8 m.
  c.add({V(0, 0, -4), V(8, 0, -4), V(12, 6, -4)});
  ASSERT_EQ(c.compared(), 1u);
  const double d = c.percentile(0.5);
  EXPECT_GT(d, 7.0);
  EXPECT_LT(d, 13.0) << "divergence at " << d << " m does not match the ~8 m the routes share";
}

// Distance is to the polyline, not to its vertices. Comparing vertex-to-vertex would report a
// sparsely-sampled route as diverged purely for being sparse, inflating near-field churn.
TEST(RouteChurn, MeasuresToTheSegmentNotTheVertices)
{
  RouteChurn c(1.0);
  c.add({V(0, 0, -4), V(40, 0, -4)});                       // two vertices, 40 m apart
  c.add({V(0, 0, -4), V(10, 0, -4), V(20, 0, -4), V(40, 0, -4)});  // same line, more vertices
  EXPECT_EQ(c.unchanged(), 1u)
      << "re-sampling the same straight line was reported as a route change";
}

// Only counted while the autopilot has control, so the first plan has no predecessor and must
// not be compared against nothing.
TEST(RouteChurn, FirstPlanHasNoPredecessor)
{
  RouteChurn c(1.0);
  c.add(straight(20.0));
  EXPECT_EQ(c.compared(), 0u);
  EXPECT_DOUBLE_EQ(c.unchanged_pct(), 0.0);
  EXPECT_DOUBLE_EQ(c.near_field_pct(), 0.0);
}

// Degenerate plans (a single waypoint, or none) must be skipped rather than counted as agreement.
TEST(RouteChurn, DegenerateRoutesAreSkipped)
{
  RouteChurn c(1.0);
  c.add(straight(20.0));
  c.add({V(1, 1, -4)});          // single waypoint
  c.add({});                     // empty
  EXPECT_EQ(c.compared(), 0u);
}

// A lateral shift smaller than the tolerance is tracking noise in the stitch point, not a route
// change; counting it would report ~100% churn on a perfectly stable route.
TEST(RouteChurn, SubToleranceWobbleIsNotAChange)
{
  RouteChurn c(1.0);
  c.add(straight(20.0, 0.0));
  c.add(straight(20.0, 0.4));   // 0.4 m offset, under the 1.0 m tolerance
  EXPECT_EQ(c.unchanged(), 1u);
}

// EPISODES. The metric that actually predicts travel-time damage is not the rate of near-field
// changes but whether they CLUSTER: a wedge where the planner flips between two routes for
// seconds on end while the vehicle crawls. Scattered changes at the same rate are harmless.
// Reading the rate instead of the episodes is a mistake that was made once already, and it
// pointed at the wrong fix.
TEST(RouteChurn, ScatteredChurnIsNotAnEpisode)
{
  RouteChurn c(1.0);
  const auto base = straight(30.0, 0.0);
  const auto off  = std::vector<V>{V(0, 0, -4), V(2, 6, -4), V(30, 6, -4)};
  // Isolated changes separated by agreement. NOTE: alternating base/off every cycle would NOT
  // be scattered -- both base->off and off->base diverge, so it is a continuous streak and is
  // exactly the wedge this metric is built to catch. Hold each route for several plans instead,
  // so a change costs at most two consecutive near-field divergences.
  for (int block = 0; block < 5; ++block) {
    for (int i = 0; i < 4; ++i) c.add(base);
    c.add(off);
    for (int i = 0; i < 4; ++i) c.add(off);
  }
  c.finalize();
  EXPECT_GT(c.near_field(), 5u) << "scenario must actually produce near-field changes";
  EXPECT_EQ(c.episodes(), 0u)
      << "scattered alternation was counted as a sustained episode; the metric would blame "
         "junction indecision for churn that costs nothing";
}

TEST(RouteChurn, SustainedFlipFloppingIsAnEpisode)
{
  RouteChurn c(1.0);
  // A wedge: every consecutive plan disagrees near the vehicle.
  for (int i = 0; i < 12; ++i) {
    c.add({V(0, 0, -4), V(2, i % 2 ? 6.0 : -6.0, -4), V(30, i % 2 ? 6.0 : -6.0, -4)});
  }
  c.finalize();
  EXPECT_GE(c.episodes(), 1u);
  EXPECT_GE(c.longest_streak(), RouteChurn::kEpisodeMin);
  EXPECT_GT(c.episode_pct(), 50.0)
      << "a run that flip-flopped on every plan reported " << c.episode_pct()
      << "% of replans inside an episode";
}

// An episode still open when the run ends must still be counted -- otherwise a vehicle that
// wedges and never recovers reports zero episodes, which is the worst possible failure of this
// metric.
TEST(RouteChurn, EpisodeOpenAtEndOfRunIsCounted)
{
  RouteChurn c(1.0);
  for (int i = 0; i < 8; ++i) {
    c.add({V(0, 0, -4), V(2, i % 2 ? 6.0 : -6.0, -4), V(30, i % 2 ? 6.0 : -6.0, -4)});
  }
  EXPECT_EQ(c.episodes(), 0u) << "episode should still be open before finalize()";
  c.finalize();
  EXPECT_EQ(c.episodes(), 1u) << "a run that wedged and never recovered reported no episode";
}

}  // namespace roscopter
