// CollisionMonitor -- the run's pass/fail verdict.
//
// This exists because the failure it guards against is SILENT. The benchmark obstacles are not
// solid: the simulator reports contact but never stops the vehicle, so a run that flew through
// three trunks still completes the course and prints a full, healthy metrics table -- with the
// best T_trav and L_path of the set, because cutting through an obstacle is a shortcut. Three
// consecutive runs did exactly that and were nearly used as evidence that a planner change had
// improved travel time.
//
// So the two properties that matter most here are counted in events rather than samples (a long
// graze is one collision, not hundreds), and NEVER-MEASURED must not read as CLEAN.

#include <gtest/gtest.h>

#include "navigation/collision_monitor.hpp"

namespace roscopter
{
namespace
{

/// Wall clock at 50 Hz, matching the rate the detector publishes at.
rclcpp::Time t(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

}  // namespace

// THE POINT OF THE WHOLE CLASS. A monitor that never heard from the detector must report
// "unmeasured", not "clean" -- otherwise a misconfigured run is indistinguishable from a good
// one, which is the exact condition that wasted three runs.
TEST(CollisionMonitor, NeverMeasuredIsNotTheSameAsClean)
{
  CollisionMonitor m;
  EXPECT_FALSE(m.topic_seen);
  EXPECT_FALSE(m.clean()) << "a monitor that saw no data claims the run was clean";

  m.update(false, t(0.0), true);
  EXPECT_TRUE(m.topic_seen);
  EXPECT_TRUE(m.clean()) << "one clean sample should establish a measured, clean run";
}

// The topic carries a LEVEL, not an event. Counting samples would turn a one-second graze at
// 50 Hz into fifty collisions and make the number useless for R_succ.
TEST(CollisionMonitor, SustainedContactIsOneEvent)
{
  CollisionMonitor m;
  for (int i = 0; i < 50; ++i) m.update(true, t(i * 0.02), true);
  EXPECT_EQ(m.events, 1) << "sustained contact counted per sample instead of per rising edge";
}

TEST(CollisionMonitor, SeparateContactsCountSeparately)
{
  CollisionMonitor m;
  m.update(true,  t(0.00), true);
  m.update(true,  t(0.02), true);
  m.update(false, t(0.04), true);   // clear
  m.update(false, t(0.06), true);
  m.update(true,  t(0.08), true);   // second strike
  EXPECT_EQ(m.events, 2);
  EXPECT_FALSE(m.clean());
}

// Time in contact must accumulate only while actually in contact, and must not bill the gaps.
TEST(CollisionMonitor, ContactTimeIntegratesOnlyWhileTouching)
{
  CollisionMonitor m;
  m.update(true,  t(0.00), true);
  m.update(true,  t(0.02), true);   // +0.02 in contact
  m.update(true,  t(0.04), true);   // +0.02
  m.update(false, t(0.06), true);   // +0.02, then clear
  m.update(false, t(1.00), true);   // long gap, NOT in contact -- must not count
  EXPECT_NEAR(m.contact_seconds, 0.06, 1e-9);
}

// Metrics elsewhere only accumulate while the autopilot has control. Contact must follow the
// same rule or a collision during a manual recovery would be blamed on the planner.
TEST(CollisionMonitor, ContactOutsideAutopilotControlIsNotCounted)
{
  CollisionMonitor m;
  m.update(true, t(0.00), /*counting=*/false);
  m.update(true, t(0.02), /*counting=*/false);
  EXPECT_EQ(m.events, 0);
  EXPECT_EQ(m.contact_seconds, 0.0);
  EXPECT_TRUE(m.topic_seen) << "the topic was still observed even if not counted";
}

// A collision already in progress when control is handed over is the SAME event, not a new one,
// and resuming must not bill the pause as contact time.
TEST(CollisionMonitor, ResumingControlDoesNotDoubleCountOrBillThePause)
{
  CollisionMonitor m;
  m.update(true, t(0.00), false);   // already touching before handover
  m.update(true, t(5.00), true);    // control acquired, still touching
  m.update(true, t(5.02), true);
  EXPECT_EQ(m.events, 0) << "an ongoing contact was re-counted as a new collision on resume";
  EXPECT_NEAR(m.contact_seconds, 0.02, 1e-9)
      << "the 5 s pause was billed as time in contact";
}

// A clock jump must not silently inflate contact time -- the guard exists because the pause
// above would otherwise add five seconds.
TEST(CollisionMonitor, ImplausibleTimeStepIsIgnored)
{
  CollisionMonitor m;
  m.update(true, t(0.00), true);
  m.update(true, t(9.00), true);    // 9 s step, beyond the guard
  EXPECT_EQ(m.contact_seconds, 0.0);
}

}  // namespace roscopter
