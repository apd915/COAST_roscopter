#ifndef ROSCOPTER_NAVIGATION_COLLISION_MONITOR_HPP
#define ROSCOPTER_NAVIGATION_COLLISION_MONITOR_HPP

#include <rclcpp/time.hpp>

namespace roscopter
{

/**
 * Contact with the world, from the simulator's own detector on mock_env/collision.
 *
 * WHY THIS IS IN THE PLANNER'S SUMMARY. The benchmark obstacles are NOT solid: the sim
 * publishes contact but never stops the vehicle, so a run that flies straight through a
 * trunk still reports "reached the goal" and produces a full, healthy-looking metrics
 * table. Three consecutive runs did exactly that before this existed, and nothing in the
 * output distinguished them from clean ones -- T_trav and L_path were the BEST of any run,
 * because cutting through obstacles is a shortcut. Any comparison drawn from runs that are
 * silently taking shortcuts is worthless, so success has to be reported next to the numbers
 * it qualifies, not in a separate terminal.
 *
 * The topic carries a LEVEL (true while in contact), so events are rising edges -- counting
 * samples would report one long graze as hundreds of collisions.
 *
 * "Never saw the topic" and "saw the topic, no contact" are reported DIFFERENTLY and that
 * distinction is the whole point: an unmeasured run must never read as a clean one.
 */
struct CollisionMonitor {
    bool topic_seen = false;      ///< false => not measured, which is NOT the same as clean
    bool in_contact = false;
    int events = 0;               ///< rising edges
    double contact_seconds = 0.0;
    bool have_last = false;
    rclcpp::Time last_stamp;

    /// @param colliding level from the detector
    /// @param now       wall clock, for integrating time in contact
    /// @param counting  whether the autopilot had control; matches every other metric
    void update(bool colliding, const rclcpp::Time& now, bool counting) {
        topic_seen = true;
        if (!counting) { have_last = false; in_contact = colliding; return; }
        if (have_last && in_contact) {
            const double dt = (now - last_stamp).seconds();
            // Guard against clock jumps and the gap across a metrics pause.
            if (dt > 0.0 && dt < 1.0) contact_seconds += dt;
        }
        last_stamp = now;
        have_last = true;
        if (colliding && !in_contact) ++events;
        in_contact = colliding;
    }

    bool clean() const { return topic_seen && events == 0; }
};

}  // namespace roscopter

#endif  // ROSCOPTER_NAVIGATION_COLLISION_MONITOR_HPP
