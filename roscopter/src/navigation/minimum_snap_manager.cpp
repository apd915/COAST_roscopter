#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <roscopter_msgs/msg/state.hpp>
#include <roscopter_msgs/msg/waypoint.hpp>
#include <roscopter_msgs/msg/trajectory_command.hpp>
#include <roscopter_msgs/msg/computation_times.hpp>
#include <rosflight_msgs/msg/status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>

#include "navigation/arrival_monitor.hpp"
#include "navigation/collision_monitor.hpp"
#include "navigation/route_churn.hpp"
#include "navigation/route_gate.hpp"

#include "trajectory_planner.hpp"
#include "voxel_grid.hpp"
#include "trajectory_metrics.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// =====================================
// B-Spline Evaluation Helpers
// =====================================
Eigen::Vector3d evaluate_bspline(double t, const Eigen::VectorXd& control_points, const Eigen::VectorXd& knots, int degree) {
    if (control_points.size() == 0 || knots.size() == 0) return Eigen::Vector3d::Zero();

    if (control_points.size() / 3 <= degree || knots.size() <= degree + 1) {
        RCLCPP_ERROR(rclcpp::get_logger("bspline"),
            "evaluate_bspline: degenerate spline (control_points=%ld, knots=%ld, degree=%d); returning zero",
            control_points.size() / 3, knots.size(), degree);
        return Eigen::Vector3d::Zero();
    }

    // Clamp t to valid range
    double t_min = knots(degree);
    double t_max = knots(knots.size() - degree - 1);
    t = std::max(t_min, std::min(t, t_max));

    // Find knot span
    int span = degree;
    if (t >= t_max) {
        span = knots.size() - degree - 2;
    } else {
        while (span < knots.size() - degree - 1 && t >= knots(span + 1)) {
            span++;
        }
    }
    
    // De Boor's algorithm
    std::vector<Eigen::Vector3d> d(degree + 1);
    for (int j = 0; j <= degree; ++j) {
        int idx = (span - degree + j) * 3;
        d[j] = Eigen::Vector3d(control_points(idx), control_points(idx+1), control_points(idx+2));
    }
    
    for (int r = 1; r <= degree; ++r) {
        for (int j = degree; j >= r; --j) {
            double denom = knots(span + j + 1 - r) - knots(span - degree + j);
            double alpha = (denom > 1e-9) ? (t - knots(span - degree + j)) / denom : 0.0;
            d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
        }
    }
    return d[degree];
}

Eigen::VectorXd get_derivative_cps(const Eigen::VectorXd& cps, const Eigen::VectorXd& knots, int degree) {
    if (cps.size() < 6) return Eigen::VectorXd();
    int n = cps.size() / 3;
    Eigen::VectorXd deriv_cps((n - 1) * 3);
    for (int i = 0; i < n - 1; ++i) {
        double denom = knots(i + degree + 1) - knots(i + 1);
        double factor = (denom > 1e-9) ? (degree / denom) : 0.0;
        deriv_cps(i*3) = factor * (cps((i+1)*3) - cps(i*3));
        deriv_cps(i*3+1) = factor * (cps((i+1)*3+1) - cps(i*3+1));
        deriv_cps(i*3+2) = factor * (cps((i+1)*3+2) - cps(i*3+2));
    }
    return deriv_cps;
}

Eigen::VectorXd get_derivative_knots(const Eigen::VectorXd& knots) {
    if (knots.size() <= 2) return Eigen::VectorXd();
    Eigen::VectorXd d_knots(knots.size() - 2);
    for (int i = 0; i < d_knots.size(); ++i) {
        d_knots(i) = knots(i + 1);
    }
    return d_knots;
}

class MinimumSnapManager : public rclcpp::Node {
public:
    /// Print whatever the run gathered, however it ended. See print_planning_time_table.
    ~MinimumSnapManager() override {
        if (!summary_printed_) {
            RCLCPP_WARN(this->get_logger(),
                        "Shutting down before the mission finished -- printing the summary for "
                        "what was flown. Metrics are for a PARTIAL run; score it accordingly.");
            print_planning_time_table();
        }
    }

    MinimumSnapManager() : Node("minimum_snap_manager") {
        // 1. Declare Parameters
        this->declare_parameter("dynamics_file", "/home/apd915/rosflight_ws/src/rosflight_ros_pkgs/rosflight_sim/params/multirotor_dynamics.yaml");
        this->declare_parameter("sensor_config_file", "/home/apd915/rosflight_ws/src/rosflight_ros_pkgs/rosflight_sim/simulators/standalone_sim/params/standalone_sim_params.yaml");
        this->declare_parameter("v_max", 3.0);
        this->declare_parameter("a_max", 2.0);
        // Minimum ground distance the truncated corridor chain must cover. max_sfc_count counts
        // CORRIDORS, not distance, which is an absorbing failure when segments are short -- see
        // FrontEndConfig::min_chain_length. 0 disables.
        // Minimum ground the corridor chain must cover, and the hard ceiling on corridor count
        // while chasing it. BOTH failure modes are real and they pull opposite ways:
        //
        //   too small -> the chain can collapse to a couple of metres, every exclusive
        //     control-point pool zeroes out, the chain ends at the vehicle and the plan commands
        //     a hover forever (hard s1: T_trav 1662 s at 0.11 m/s).
        //   too large -> the chain grows to the ceiling chasing distance through short segments.
        //     At 9 corridors that is 8 junctions x degree 4 = 32 bridge points, i.e. ~32 s of
        //     flight regardless of ground covered, plus a 3-5x more expensive A*
        //     (hard s0: T_trav 150-158 s, L_path 241-273 m, front-end 46-65 ms, and the
        //     directional gate saturating with p99 divergence at 180 deg).
        //
        // 8.0 m with a 3x ceiling produced the second. 4.0 m with a ceiling of 5 keeps the
        // escape from the first while capping bridge overhead at 16 points.
        // A/B switch for the A* geofence containing the vehicle -- see
        // FrontEndConfig::geofence_contains_vehicle. true fixes the above-ceiling lock-up but
        // also lets A* cruise above the canopy; false is the pre-fix behaviour exactly.
        this->declare_parameter("geofence_contains_vehicle", true);
        this->declare_parameter("min_chain_length", 4.0);
        this->declare_parameter("max_sfc_count_hard", 5);
        // Escape hatch for plans that go nowhere; see the escape in replan_loop. Set false to
        // isolate it -- when it fires it disables the planning horizon for that cycle, which is
        // a large behavioural change, and the geofence pad then falls back to 60 m so a goal
        // further than that is OUTSIDE the search bounds and A* returns a PARTIAL path whose
        // endpoint can be almost anywhere on the search frontier.
        this->declare_parameter("degenerate_escape", true);
        // Seconds a mission may sit stalled beyond the arrival fallback ceiling before the run
        // is declared failed. 0 disables (wait forever, the old behaviour).
        //
        // The arrival backstop only skips a waypoint within fallback_radius (2.5 m), which is
        // right -- skipping from 33 m away would silently teleport the mission past terrain it
        // never flew. But that leaves NOTHING to end a run that gives up at distance: the
        // planner reports the goal unreachable, the vehicle hovers, and the mission waits
        // indefinitely. Measured on hard s1 at v_max 8, stopped 33.3 m out and holding.
        //
        // For a 90-run benchmark an indefinite hang is far more expensive than a recorded
        // failure, and a stall IS a failure -- it is exactly what R_succ is supposed to count.
        // This ends the leg so the run terminates, prints its summary, and can be scored.
        this->declare_parameter("stall_timeout", 45.0);
        // Whether declaring a stall ENDS the run. Detection (stall_timeout) is separate so
        // the snapshot is still written when the run is allowed to continue.
        this->declare_parameter("stall_ends_run", false);
        // Where to dump the planner's full state when a stall fires. Empty disables.
        this->declare_parameter("stall_snapshot_path", std::string("/tmp/stall_snapshot.txt"));
        this->declare_parameter("max_sfc_count", -1);  // -1 = no cap; let the smoother decide how many corridors the path needs
        this->declare_parameter("planning_horizon", 10.0);
        // SFC geometry. These were previously stuck at the planner library's internal
        // defaults with no way to tune them from ROS. Corridor lateral width is capped
        // at (sfc_width - 2 * drone_radius), so raising sfc_width is what lets corridors
        // expand into open space.
        this->declare_parameter("sfc_width", 5.0);
        this->declare_parameter("sfc_height", 5.0);
        this->declare_parameter("sfc_start_ext", 5.0);
        this->declare_parameter("sfc_end_ext", 5.0);
        // Maximum age of a committed trajectory before we replan against the latest map,
        // even when that trajectory is still collision-free. Trades a little re-plan
        // "stutter" for acting on newly revealed free space. 0 disables the freshness
        // check entirely (fly each plan until it drains or becomes unsafe).
        this->declare_parameter("max_plan_age", 0.5);

        // ROUTE COMMITMENT. Off by default, so the A arm is exactly the historical behaviour.
        //
        // When on, a plan is held until something INVALIDATES it -- the buffered trajectory
        // collides against the updated map, the plan ages out, or the goal moves -- instead of
        // being discarded every cycle merely because a newer one could be computed. That is the
        // mechanism aimed at junction indecision: at v_max 8 the vehicle averaged 1.61 m/s
        // (20% of the limit) over 955 replans on a path 1.68x the direct distance, and the
        // planner changing its mind is the suspected cause. Measure it with the ROUTE CHURN
        // block, not by eye.
        //
        // TWO EARLIER COMMITMENT MECHANISMS MEASURED WORSE. The anti-stutter gate cost +31%
        // travel time and path hysteresis collapsed v_avg to 1.02 m/s. Both failed for the same
        // reason: committing means actually FLYING a trajectory that the control-point
        // allocation had made far longer than the kinematics require, which rapid replanning
        // was accidentally hiding by only ever executing each plan's fast opening. That
        // over-allocation is smaller at max_sfc_count 3 (~4x rather than ~9x) but is NOT gone,
        // so treat a travel-time regression here as the expected failure and check the churn
        // numbers before concluding anything.
        this->declare_parameter("commit_route", false);
        // How long a committed plan may be held before it is replanned regardless. This is the
        // bound on flying open-loop: the collision check only asks "is my plan still safe?",
        // never "has a better route appeared?". Separate from max_plan_age so the uncommitted
        // arm keeps its 0.5 s exactly.
        this->declare_parameter("commit_max_age", 2.0);

        // DIRECTIONAL GATE on candidate plans. 0 disables it, so the default build behaves
        // exactly as if it were not compiled in.
        //
        // Unlike commit_route, this freezes NOTHING: the planner runs in full every cycle and
        // every accepted plan is fresh, so the vehicle keeps flying trajectory openings rather
        // than tails, and a held route cannot go stale against a drifting state estimate. The
        // only change is that a candidate whose near-field heading disagrees with the route
        // being flown is declined. See RouteGate for why the two commitment designs that froze
        // geometry both lost.
        this->declare_parameter("route_reject_angle_deg", 0.0);
        // How much better aimed at the goal a candidate must be to BYPASS the gate, as a
        // difference of direction cosines. The override stops one bad plan self-defending; at a
        // junction with two comparably goal-ward branches it is instead what lets the planner
        // flip-flop, because the vehicle shifts each cycle and that flips the scoring.
        //
        // Across ten medium seeds the override was used 0-43 times on PASSING runs and 213
        // times on the one that wedged (s4: 642 m flown on a 105 m route, 6 collisions, p90
        // route divergence 150.95 deg). Correlational, n=5 -- the offline harness cannot
        // reproduce wedging (it replans 5 m apart, flight replans ~0.5 m apart), so a flat
        // sweep there proved nothing. Left at the historical 0.10; test it on s4, which wedges
        // reproducibly.
        this->declare_parameter("route_alignment_margin", 0.10);
        // How far ahead the two routes are compared. Matches the churn metric's near-field
        // threshold; beyond it a difference is harmless because the vehicle replans past it.
        this->declare_parameter("route_reject_lookahead_m", 3.0);
        // Force-accept after this many consecutive rejections -- without it, sustained
        // rejection IS plan-level commitment and brings back the stutter.
        this->declare_parameter("route_reject_max_consecutive", 10);
        // Extra cost A* pays per step through never-observed space, as a multiplier on the
        // step cost. Higher = hugs measured-free space harder but may fail to find a route;
        // negative = hard-block unknown (the old behaviour, which could not plan past a
        // corner because everything beyond it is unobserved).
        this->declare_parameter("unknown_traversal_cost", 2.0);
        // Whether unmeasured space walls corridors off. OFF by default: it is inconsistent with
        // A* (which is free to route through unknown space for unknown_traversal_cost) and it
        // collapses the corridor around any waypoint the LiDAR has not swept yet to zero width.
        // Measured over 204 goals, turning it ON costs: failures 8 -> 15, mean narrowest corridor
        // half-width 0.476 -> 0.122 m, mean duration 5.54 -> 6.07 s, mean plan time 1.70 -> 3.51 ms.
        // Safety against unknown space comes from unknown_traversal_cost, the 10 Hz collision
        // re-check of buffered setpoints, and the braking-bounded terminal velocity instead.
        this->declare_parameter("frontier_blocks_corridors", false);

        // OCCUPANCY DEPTH BEHIND A RETURN, in metres. 0 = the old one-cell-per-beam behaviour.
        //
        // A beam marks only the cell it ends in, so the far side of every obstacle is sensor
        // shadow: never observed, hence UNKNOWN, hence NOT blocked -- A* crosses it for
        // unknown_traversal_cost, and the corridor builder (which shrinks only against KNOWN
        // obstacles) then wraps a box around a real trunk. Measured on the forest_hard_s2
        // bottleneck: 22 of 55 cells inside a trunk were unblocked, inflation covering only the
        // near 2.4 m of a 3.6 m trunk.
        //
        // NOT fixable by raising unknown_traversal_cost. A* is weighted, and the guidance
        // invariant is 1 + unknown_traversal_cost + heat_weight < astar_heuristic_weight; at
        // heat_weight 0.6 and astar_heuristic_weight 3.0 the ceiling is 1.4, well below the
        // ~2.2 this geometry would need. Past the ceiling A* stops preferring the goal and
        // floods. The map has to know the tree is solid.
        //
        // 1.0 m suits the 1.0-1.5 m benchmark trunks; inflation carries the remainder. The fill
        // stops at any cell already measured FREE, so a swept gap behind a thin obstacle stays
        // open.
        this->declare_parameter("occupancy_thickness", 0.0);
        // Slack on the initial flight-time budget. The ideal straight-line time is
        // systematically a little short once the trajectory also has to curve and stop, so
        // the first solve fails and the retry loop lengthens it -- and those added control
        // points ARE flight time, permanently. Budgeting slightly long up front means fewer
        // retries, faster planning, and (counter-intuitively) often a faster trajectory.
        this->declare_parameter("time_budget_factor", 1.20);
        // Wall-clock ceiling on the OSQP retry cycle for one replan, summed over attempts.
        // A doomed solve spends a full max_iter before OSQP can certify infeasibility, so an
        // unlucky replan could run 4 attempts and ~270 ms against a 100 ms period. Missing a
        // replan is cheap (the committed trajectory keeps flying); overrunning starves the
        // 50 Hz dispatch loop. 0 disables.
        this->declare_parameter("max_solve_time_ms", 60.0);
        // Goals closer than this are interpolated directly rather than planned. Must stay ABOVE
        // the 1.0 m arrival radius in dispatch_loop, otherwise a band of goals exists that is
        // neither "arrived" nor short enough to bypass, and the planner is handed a sub-metre
        // problem it cannot solve -- see the short-move block in replan_loop.
        this->declare_parameter("min_planning_distance", 1.5);
        // A move counts as "mostly vertical" -- and is interpolated rather than planned -- when
        // horizontal_dist < max(0.5, ratio * vertical_dist). Corridor planning degenerates as the
        // corridor axis approaches world up, and a climb has no lateral avoidance content, so
        // handing such moves to the QP produces boundary-block infeasibility. 0 disables.
        this->declare_parameter("vertical_bypass_ratio", 0.35);
        // Worst constraint violation (m, m/s, m/s^2) at which a NON-CONVERGED OSQP iterate is
        // still flown. OSQP needs primal AND dual residuals inside tolerance, so it can stop at
        // max_iter holding a point whose constraints are satisfied to ~1e-4 purely because
        // optimality has not settled. Discarding that costs a whole replan and can leave the
        // vehicle hovering at a waypoint. Never applied to a primal-infeasible verdict.
        this->declare_parameter("qp_accept_violation", 1e-2);
        // A* search budget. Node count is the primary, DETERMINISTIC limit; the wall-clock value
        // is only a backstop for the case where per-node cost has grown with map density (it was
        // hardcoded at 100 ms -- the whole replan period -- so it could never protect the loop).
        // Measured over 204 goals, 3000 / 6000 / 12000 nodes are indistinguishable in both
        // solved and reached_goal, while an exhausted search cost ~95 ms in flight.
        this->declare_parameter("astar_max_nodes", 3000);
        this->declare_parameter("astar_max_time_ms", 25.0);
        // Clearance demanded BEYOND the vehicle hull, in metres. Obstacles are inflated by
        // (drone_physical_radius + this), so the corridor -- and therefore the reference
        // trajectory -- is held this far off the vehicle's own surface.
        //
        // It was previously 0 (the grid was constructed with drone_physical_radius alone). That
        // makes the guarantee exactly "a PERFECTLY tracked trajectory grazes the obstacle", which
        // leaves nothing for tracking error, and the vehicle clips obstacles it planned around.
        // Measured horizontal peak error was 0.539 m north / 0.606 m east against a 0.490 m hull,
        // i.e. up to ~0.6 m of penetration on a trajectory the planner had certified clear.
        //
        // SANDO budgets the same thing: inflation_hgp 0.45 m against a drone_bbox half-extent of
        // 0.1 m, so ~0.35 m of clearance beyond the hull, plus a soft "heat" halo (rmax 1.0 m)
        // that biases the search away from obstacles rather than merely permitting grazing ones.
        // They widen both on hardware.
        //
        // This is not free: it narrows every corridor by the same amount, so dense environments
        // get harder to solve. sfc_width must stay above 2*(radius + margin) for a corridor to
        // exist at all. Tune it against the forest benchmarks rather than in the abstract.
        this->declare_parameter("obstacle_margin", 0.30);
        // Voxel edge length, m. Was hardcoded at 0.5; exposed because it trades against sensor
        // sampling in a way that is not obvious and is worth being able to sweep.
        //
        // Finer is NOT automatically better here. Free space is only marked where a beam passes
        // through, so the map's usable resolution is bounded by BEAM SPACING, not voxel size:
        // 3 deg rings are 0.79 m apart at 15 m, so at 0.5 m a beam gap spans 1.6 voxels and at
        // 0.3 m it spans 2.6. Shrinking the voxel below the beam spacing manufactures UNKNOWN
        // cells between beams -- which A* then pays unknown_traversal_cost to cross -- and it
        // shrinks how far the node budget reaches, since the explored volume grows as (r/res)^3.
        // Raise the LiDAR sampling rate before, or with, lowering this.
        // (SANDO runs 0.3 m, but against a far denser scan and a 0.1 m vehicle.)
        this->declare_parameter("voxel_resolution", 0.5);
        // Radius, m, of the accumulated map kept around the vehicle. Cells outside are pruned.
        //
        // This is the dominant cost in the whole node, and it was never exposed. The per-scan
        // rebuild of the derived structures stamps a 5x5x5 inflation box per occupied voxel and
        // then builds a KD-tree over the result, so its cost scales with the OCCUPIED COUNT --
        // i.e. with window area, not with the scan. Measured in a forest at the 25 m default:
        // 90 ms/scan early in the traverse rising to 117 ms as the map filled, against a 100 ms
        // period, on the same single thread as the 50 Hz dispatch loop. That is the whole
        // "starts fast, stalls halfway" symptom: nothing local goes wrong, the map simply grows.
        //
        // Nothing beyond planning_horizon is used for planning, so a window much larger than it
        // buys nothing and costs quadratically.
        this->declare_parameter("map_window_radius", 25.0);
        // Height AGL, m, above which a short/vertical move is COMMITTED and flown open-loop.
        // Below it the move is re-planned from the vehicle's actual position every cycle, so a
        // thrust-limited vehicle still breaking ground contact cannot be left behind by its own
        // setpoint. See the commit site in replan_loop.
        this->declare_parameter("takeoff_agl", 1.0);
        // Altitude tolerance, m, that the FIRST waypoint must additionally satisfy before the
        // mission advances. Without it a 1.0 m arrival sphere around a waypoint 3 m overhead is
        // satisfied at 2 m altitude, and the vehicle starts the horizontal traverse mid-climb.
        this->declare_parameter("takeoff_altitude_tol", 0.35);
        // Vertical speed (m/s) below which the initial climb counts as SETTLED.
        //
        // takeoff_altitude_tol alone tests POSITION, and a vehicle climbing through the target
        // at takeoff_speed satisfies it for one instant -- the gate opens mid-climb, the mission
        // pulls the next waypoint, and momentum carries the vehicle on up while the trajectory
        // turns horizontal underneath it. That is the diagonal "up and ahead" departure seen at
        // the start of every run, and it is why vertical peak_true sits at 4.2-5.3 m above a
        // 3.0 m cruise in EVERY flight measured today.
        //
        // It matters beyond looking wrong: the overshoot is not in any corridor or trajectory,
        // so the follower begins each run having to close an error the planner never commanded,
        // and the tracking numbers for the whole flight inherit it.
        // Speed (m/s) below which the vehicle counts as SETTLED at the first waypoint. 0 disables.
        //
        // The traverse must not begin until the vehicle is actually AT waypoint altitude and
        // essentially stationary. Two reasons, and the second is the one that bites:
        //
        //   Planning from an altitude the mission never asked for means planning from a place
        //   the LiDAR has a poor picture of -- above the canopy the map is mostly unknown, so
        //   the route is chosen on the least information available anywhere in the flight.
        //
        //   And once above max_altitude, A* has an obstacle-free corridor straight to the goal
        //   (trunks are 6 m, max_altitude is 6 m), so it prefers to STAY up there. Measured:
        //   from a 7.5 m start the whole chain sits at 6.25-7.50 m; from 8.5 m it is a single
        //   corridor above the entire forest. The waypoint is supposed to dictate where
        //   planning starts, not the overshoot.
        //
        // Checks FULL speed, not just vertical: a vehicle still carrying forward velocity when
        // the traverse begins starts its first plan from a state the plan did not assume.
        this->declare_parameter("takeoff_settle_speed", 0.30);
        // Seconds to wait for that settle before proceeding anyway. Without a bound, a vehicle
        // that never settles -- oscillating, or held off altitude by wind -- hangs the mission:
        // the arrival backstop only fires within fallback_radius, which an overshoot exceeds.
        this->declare_parameter("takeoff_settle_timeout", 8.0);
        // Climb rate, m/s, used to size the FIRST move only. v_max is a cruise limit and makes
        // for a violent lift-off when applied to it -- see generate_direct_trajectory.
        this->declare_parameter("takeoff_speed", 1.0);
        // --- Proximity ("heat") field -------------------------------------------------------
        // Graded clearance cost around obstacles: 1 at the inflated boundary, decaying to 0 over
        // heat_radius beyond it. The inflated set alone is a BINARY test, so A* has no reason to
        // prefer the middle of a gap over grazing its edge -- paths hug inflated boundaries and
        // wedge where a slightly wider route existed. heat_radius <= 0 disables it entirely.
        this->declare_parameter("heat_radius", 1.0);
        // Weight on heat in A*'s step cost. Bounded by the invariant below.
        this->declare_parameter("heat_weight", 0.6);
        // Weight on the A* heuristic. This is the CEILING for the summed step-cost multipliers:
        //     1 + unknown_traversal_cost + heat_weight  <  astar_heuristic_weight
        // A* prefers a step toward the goal only while that holds. It was hardcoded at 2.0, which
        // left no room for a clearance term at all -- the reason the earlier binary toll broke
        // the search. Raised to 3.0 to make room; the front end warns if the invariant breaks.
        this->declare_parameter("astar_heuristic_weight", 3.0);
        // Penalty per step for traversing an inflated cell instead of blocking it. 0 = hard
        // block (the old behaviour). Applied only on a SECOND A* pass, when the hard search
        // cannot reach the goal: a soft search cannot fail, so it recovers the route shape, and
        // truncating it at the first blocked cell gives a reachable sub-goal far better than the
        // dead-end stub a failed hard search returns. The corridor builder still only ever sees
        // all-free cells.
        this->declare_parameter("astar_soft_obstacle_cost", 5.0);
        // Smoothed path segments shorter than this many VOXELS are merged away where
        // line-of-sight permits. 0 disables.
        //
        // The smoother re-anchors on the cell before each line-of-sight failure, so at a tight
        // corner it emits consecutive cells -- one-voxel segments. Those make degenerate corridor
        // spines: diagonal (SAT corner term 0.354 m rather than 0.25 m) and still carrying the
        // full sfc_end_ext, so the box is ~8.6x the length of its own segment and is crushed by
        // its neighbours' obstacles. Measured: a lateral bound of exactly 0, a primal-infeasible
        // QP that time-stretching cannot fix, and a permanent stall in open air.
        this->declare_parameter("astar_min_segment_voxels", 2.0);

        // DEGENERATE-CORRIDOR REPAIR, in metres of lateral half-width. 0 disables.
        //
        // A corridor whose narrowest lateral face falls below this has the interior waypoint
        // that produced it dropped, merging its segment into a neighbour, and the chain is
        // rebuilt. Kept only if that reduces the number of degenerate corridors.
        //
        // A collapsed corridor is NOT unsafe -- the box is already inset by hull +
        // obstacle_margin, so a trajectory on its wall still keeps the CENTRE OF MASS clear,
        // and the trajectory tracks the centre of mass. It is INFEASIBLE: every control point
        // must lie inside the corridor, so a collapsed axis forbids curvature on that axis,
        // and the QP comes back primal-infeasible on the corridor block.
        //
        // Measured over 25 paired Monte-Carlo forests at 20% density, v_max 5:
        //     0.00  68% goal-reach, 32% stall     0.15  92% /  8%   <-- default
        //     0.05  76% / 24%                     0.30  84% / 16%   (over-merges)
        // p90 plan time FELL 31.7 -> 26.1 ms, because a failed QP burns the full max_iter.
        this->declare_parameter("sfc_min_usable_width", 0.15);
        // Altitude CEILING for A*, metres above ground. <= 0 disables (map_bounds geofence).
        //
        // min_altitude has always had a floor; there was no ceiling, so the only upward bound was
        // map_bounds.z/2 = 7.5 m -- a map-sizing number acting as a flight limit by accident. In
        // the forest that left a 0.81 m window above the 6.69 m inflated trunk tops, and A* used
        // it: climb 4.14 -> 7.24 m, cross above the canopy, dive back at 77 degrees. Legal, so no
        // failure was reported, but the resulting corridor chain turns too sharply for
        // consecutive boxes to share volume and the QP is infeasible on the corridor block.
        //
        // SANDO carries this as z_max (6.0 m, its trunk height) on both the search and the
        // corridor decomposition.
        this->declare_parameter("max_altitude", -1.0);
        // Altitude FLOOR for A*, metres above ground. <0 = auto (drone radius + 0.1).
        // See where min_altitude is consumed for why ~2.0 is worth trying.
        this->declare_parameter("min_altitude", -1.0);
        // --- Path hysteresis -----------------------------------------------------------------
        // Discount on the A* step cost for cells on the PREVIOUS committed route. 0 disables.
        //
        // Replanning at 10 Hz re-decides the route from scratch every cycle. Where two or three
        // branches cost within a few percent of each other, the argmin flips on tiny changes in
        // vehicle position and the vehicle dithers between them instead of committing. A discount
        // makes continuity cheap without making it mandatory: an alternative must be more than
        // this fraction cheaper to win.
        this->declare_parameter("path_hysteresis_bonus", 0.20);
        this->declare_parameter("path_hysteresis_radius_cells", 1);
        // --- Waypoint arrival ---
        //
        // Nominal arrival radius, tested against the ESTIMATED position. Note that estimator bias
        // does NOT by itself keep this from firing, even though it looks like it should: the
        // controller drives the estimate to the commanded position and the planner's goal is the
        // waypoint's world coordinate, so on a successful plan the estimate converges to the
        // waypoint and the test passes. Bias changes where the vehicle TRULY ends up, which is a
        // safety-margin question (see the corridor argument in CLAUDE.md), not a liveness one.
        this->declare_parameter("arrival_radius", 1.0);
        // A mission can still hover indefinitely a metre or two short of a waypoint -- observed
        // repeatedly at the 3rd waypoint in sim. The mechanisms that actually do this are a front
        // end that cannot route to the goal (a real obstacle, or one phantom-displaced because it
        // was mapped at the estimate's earlier bias and the vehicle now flies at a later one) and,
        // on hardware, wind holding the vehicle off the point with the controller saturated.
        //
        // arrival_fallback_radius is the ceiling on declaring arrival anyway once progress has
        // stalled. It is the safety mechanism for that fallback: inside it, giving up on the point
        // is low-risk and the next waypoint is planned with full obstacle avoidance; beyond it
        // nothing is ever skipped. Set at or below arrival_radius to disable the fallback entirely.
        this->declare_parameter("arrival_fallback_radius", 2.5);
        // Speed (m/s) below which the vehicle counts as no longer making way toward the waypoint.
        this->declare_parameter("arrival_stall_speed", 0.35);
        // How long (s) closest-approach must fail to improve, with the vehicle stopped and the
        // planner reporting it planned all the way to the goal, before arrival is declared anyway.
        // Must comfortably exceed a replan period (100 ms) so ordinary replanning pauses never
        // trip it.
        this->declare_parameter("arrival_stall_time", 4.0);
        // Improvement in closest approach (m) that counts as progress and rearms the stall timer.
        // Above the position noise floor, so estimator jitter alone cannot hold the timer off.
        this->declare_parameter("arrival_progress_epsilon", 0.15);
        // Default to hardcoded Livox specs if parsing fails, but these will be overwritten by YAML
        this->declare_parameter("sensor_range", 40.0);
        
        // 2. Parse YAML for drone radius
        double drone_radius = 0.5;
        std::string dynamics_file = this->get_parameter("dynamics_file").as_string();
        try {
            YAML::Node config = YAML::LoadFile(dynamics_file);
            auto multirotor = config["/multirotor_forces_and_moments"]["ros__parameters"];
            if (multirotor && multirotor["rotor_positions"]) {
                auto positions = multirotor["rotor_positions"].as<std::vector<double>>();
                if (positions.size() % 3 != 0) {
                    RCLCPP_ERROR(this->get_logger(),
                        "rotor_positions has %zu entries (not a multiple of 3); skipping drone_radius parsing, using default %f m",
                        positions.size(), drone_radius);
                } else {
                    double max_r_sq = 0;
                    for (size_t i = 0; i < positions.size(); i += 3) {
                        double r_sq = positions[i]*positions[i] + positions[i+1]*positions[i+1];
                        if (r_sq > max_r_sq) max_r_sq = r_sq;
                    }
                    double d_prop = multirotor["D_prop"].as<double>();
                    drone_radius = std::sqrt(max_r_sq) + (d_prop / 2.0);
                    RCLCPP_INFO(this->get_logger(), "Parsed drone_radius: %f m", drone_radius);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse dynamics file: %s", e.what());
        }

        // 3. Parse YAML for sensor specs
        double fov_min = -7.0;
        double fov_max = 52.0;
        double sensor_range = this->get_parameter("sensor_range").as_double();
        
        std::string sensor_config_file = this->get_parameter("sensor_config_file").as_string();
        try {
            YAML::Node s_config = YAML::LoadFile(sensor_config_file);
            auto mock_env = s_config["/mock_environment"]["ros__parameters"];
            if (mock_env) {
                if (mock_env["lidar_v_fov_min_deg"]) fov_min = mock_env["lidar_v_fov_min_deg"].as<double>();
                if (mock_env["lidar_v_fov_max_deg"]) fov_max = mock_env["lidar_v_fov_max_deg"].as<double>();
                if (mock_env["lidar_max_range"]) sensor_range = mock_env["lidar_max_range"].as<double>();
                RCLCPP_INFO(this->get_logger(), "Parsed LiDAR specs - range: %f m, FoV min: %f deg, FoV max: %f deg", 
                            sensor_range, fov_min, fov_max);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse sensor config file: %s", e.what());
        }

        // 4. Initialize Planner Config
        fe_config_.drone_physical_radius = drone_radius;
        fe_config_.voxel_resolution = this->get_parameter("voxel_resolution").as_double();
        // In minimum snap, map_bounds needs to be large enough for the simulation
        fe_config_.map_bounds = Eigen::Vector3d(100.0, 100.0, 15.0);
        fe_config_.sensor_range = sensor_range;
        fe_config_.fov_vertical_min = fov_min;
        fe_config_.fov_vertical_max = fov_max;
        // ALTITUDE FLOOR FOR A*. <0 keeps the historical auto value (drone radius + 0.1 =
        // 0.59 m), which is a CLEARANCE limit masquerading as a flight limit.
        //
        // The mission cruises at 3.0 m, so the auto value leaves a 2.4 m band the search may
        // sag into, and nothing in the QP penalises altitude: the cost is snap, and the
        // corridor's z half-extents run to ~2 m. Each replan then starts from wherever the
        // last one sagged to, so the descent ratchets. Observed in flight as the vehicle
        // traversing near the floor, and captured in a stall snapshot at 0.82 m altitude --
        // 0.23 m above this floor, with the goal 39.7 m away.
        //
        // It is not a density cheat the way overflight would be (trunks span ground to 6 m, so
        // the horizontal clutter at 0.8 m is identical to 3 m), but it is unrealistic, it
        // leaves the search almost no vertical room below the vehicle, and it is where the
        // "stalls with open space all around" reports come from.
        //
        // Setting this to ~2.0 bounds the band to 2.0-6.0 m around a 3.0 m cruise. Left on
        // auto by default so the benchmark configuration is not changed silently.
        const double min_alt_param = this->get_parameter("min_altitude").as_double();
        fe_config_.min_altitude =
            (min_alt_param >= 0.0) ? min_alt_param : (drone_radius + 0.1);
        fe_config_.max_sfc_count = this->get_parameter("max_sfc_count").as_int();
        fe_config_.min_chain_length = this->get_parameter("min_chain_length").as_double();
        fe_config_.geofence_contains_vehicle =
            this->get_parameter("geofence_contains_vehicle").as_bool();
        fe_config_.max_sfc_count_hard =
            static_cast<int>(this->get_parameter("max_sfc_count_hard").as_int());

        {
            roscopter::RouteGate::Config gc;
            gc.reject_angle_deg = this->get_parameter("route_reject_angle_deg").as_double();
            gc.alignment_margin = this->get_parameter("route_alignment_margin").as_double();
            gc.lookahead_m = this->get_parameter("route_reject_lookahead_m").as_double();
            gc.max_consecutive_rejects =
                static_cast<int>(this->get_parameter("route_reject_max_consecutive").as_int());
            route_gate_.set_config(gc);
        }
        fe_config_.planning_horizon = this->get_parameter("planning_horizon").as_double();
        fe_config_.sfc_width = this->get_parameter("sfc_width").as_double();
        fe_config_.sfc_height = this->get_parameter("sfc_height").as_double();
        fe_config_.sfc_start_ext = this->get_parameter("sfc_start_ext").as_double();
        fe_config_.sfc_end_ext = this->get_parameter("sfc_end_ext").as_double();
        fe_config_.v_max = this->get_parameter("v_max").as_double();
        fe_config_.a_max = this->get_parameter("a_max").as_double();
        fe_config_.unknown_traversal_cost =
            this->get_parameter("unknown_traversal_cost").as_double();
        fe_config_.max_solve_time_ms =
            this->get_parameter("max_solve_time_ms").as_double();
        min_planning_distance_ = this->get_parameter("min_planning_distance").as_double();
        vertical_bypass_ratio_ = this->get_parameter("vertical_bypass_ratio").as_double();
        fe_config_.qp_accept_violation =
            this->get_parameter("qp_accept_violation").as_double();
        fe_config_.heat_radius = this->get_parameter("heat_radius").as_double();
        fe_config_.heat_weight = this->get_parameter("heat_weight").as_double();
        fe_config_.astar_heuristic_weight =
            this->get_parameter("astar_heuristic_weight").as_double();
        fe_config_.astar_soft_obstacle_cost =
            this->get_parameter("astar_soft_obstacle_cost").as_double();
        fe_config_.astar_min_segment_voxels =
            this->get_parameter("astar_min_segment_voxels").as_double();
        fe_config_.sfc_min_usable_width =
            this->get_parameter("sfc_min_usable_width").as_double();
        fe_config_.max_altitude = this->get_parameter("max_altitude").as_double();
        fe_config_.path_hysteresis_bonus =
            this->get_parameter("path_hysteresis_bonus").as_double();
        fe_config_.path_hysteresis_radius_cells =
            static_cast<int>(this->get_parameter("path_hysteresis_radius_cells").as_int());
        if (fe_config_.max_altitude > 0.0) {
            RCLCPP_INFO(this->get_logger(),
                        "Altitude band: %.2f m floor, %.2f m ceiling. Overflight of taller "
                        "obstacles is structurally prevented.",
                        fe_config_.min_altitude, fe_config_.max_altitude);
            if (fe_config_.max_altitude <= fe_config_.min_altitude) {
                RCLCPP_WARN(this->get_logger(),
                            "max_altitude (%.2f) <= min_altitude (%.2f): A* has no traversable "
                            "altitude band and every search will fail.",
                            fe_config_.max_altitude, fe_config_.min_altitude);
            }
        }
        fe_config_.astar_max_nodes = this->get_parameter("astar_max_nodes").as_int();
        fe_config_.astar_max_time_ms = this->get_parameter("astar_max_time_ms").as_double();
        fe_config_.time_budget_factor =
            this->get_parameter("time_budget_factor").as_double();
        // Stamp the kinodynamic limits into every quality accumulator so the L-inf violation
        // counters have something to measure against. Must happen before any setpoint is
        // dispatched; a zero limit disables counting rather than reporting 100% violation.
        {
            const double v_lim = this->get_parameter("v_max").as_double();
            const double a_lim = this->get_parameter("a_max").as_double();
            for (FlownQuality* q : {&leg_quality_, &leg_planned_quality_,
                                    &planned_quality_, &direct_quality_}) {
                q->set_limits(v_lim, a_lim);
            }
        }

        obstacle_margin_ = this->get_parameter("obstacle_margin").as_double();
        takeoff_agl_ = this->get_parameter("takeoff_agl").as_double();
        takeoff_altitude_tol_ = this->get_parameter("takeoff_altitude_tol").as_double();
        takeoff_settle_speed_ = this->get_parameter("takeoff_settle_speed").as_double();
        takeoff_settle_timeout_ = this->get_parameter("takeoff_settle_timeout").as_double();
        takeoff_speed_ = this->get_parameter("takeoff_speed").as_double();
        const double required_width = 2.0 * (drone_radius + obstacle_margin_);
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle inflation %.3f m = hull %.3f + margin %.3f. A gap must exceed "
                    "%.2f m to be flyable.",
                    drone_radius + obstacle_margin_, drone_radius, obstacle_margin_,
                    required_width);
        if (fe_config_.sfc_width <= required_width) {
            RCLCPP_WARN(this->get_logger(),
                        "sfc_width (%.2f m) <= 2*(radius+margin) (%.2f m): corridors cannot open "
                        "to full width and may collapse. Raise sfc_width or lower obstacle_margin.",
                        fe_config_.sfc_width, required_width);
        }
        arrival_radius_ = this->get_parameter("arrival_radius").as_double();
        arrival_.radius = arrival_radius_;
        arrival_.fallback_radius =
            this->get_parameter("arrival_fallback_radius").as_double();
        arrival_.stall_speed = this->get_parameter("arrival_stall_speed").as_double();
        arrival_.stall_time = this->get_parameter("arrival_stall_time").as_double();
        arrival_.progress_epsilon =
            this->get_parameter("arrival_progress_epsilon").as_double();
        if (!arrival_.disabled()) {
            RCLCPP_INFO(this->get_logger(),
                        "Arrival: %.2f m nominal, stalled-progress fallback out to %.2f m "
                        "(speed < %.2f m/s, no %.2f m gain for %.1f s).",
                        arrival_radius_, arrival_.fallback_radius, arrival_.stall_speed,
                        arrival_.progress_epsilon, arrival_.stall_time);
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Arrival: %.2f m nominal, stalled-progress fallback DISABLED.",
                        arrival_radius_);
        }
        // These two are coupled: goals inside min_planning_distance are interpolated rather than
        // planned, so if the arrival radius grows past it a band of goals opens that is neither
        // "arrived" nor short enough to bypass, and the full A*/SFC/QP pipeline is handed a
        // sub-metre problem it comes back primal-infeasible on. Retuning arrival_radius in the
        // field is exactly how that band gets reopened, so say so at startup rather than leaving
        // it to be rediscovered as a hover.
        if (arrival_radius_ >= min_planning_distance_) {
            RCLCPP_WARN(this->get_logger(),
                        "arrival_radius (%.2f m) >= min_planning_distance (%.2f m). Goals between "
                        "them are neither arrived nor interpolated, and the planner will be given "
                        "sub-metre problems it cannot solve. Raise min_planning_distance above the "
                        "arrival radius.",
                        arrival_radius_, min_planning_distance_);
        }
        RCLCPP_INFO(this->get_logger(), "Sensor range: %.1f m, Min altitude: %.2f m, Max SFCs: %d",
                    fe_config_.sensor_range, fe_config_.min_altitude, fe_config_.max_sfc_count); 

        // Inflate by hull radius PLUS the tracking margin -- see the obstacle_margin parameter.
        // drone_physical_radius alone certifies only that a perfectly tracked trajectory grazes
        // the obstacle, which is not a usable guarantee for a vehicle with real tracking error.
        shared_grid_ = std::make_shared<trajectory_planner::mapping::SparseVoxelGrid>(
            fe_config_.voxel_resolution,
            fe_config_.drone_physical_radius + obstacle_margin_);
        shared_grid_->set_accumulate(true, this->get_parameter("map_window_radius").as_double());
        shared_grid_->set_frontier_blocks_corridors(
            this->get_parameter("frontier_blocks_corridors").as_bool());
        shared_grid_->set_occupancy_thickness(
            this->get_parameter("occupancy_thickness").as_double());
        
        planner_ = std::make_unique<trajectory_planner::TrajectoryPlanner>(
            fe_config_, shared_grid_, 
            this->get_parameter("v_max").as_double(), 
            this->get_parameter("a_max").as_double()
        );

        // 4. Subscriptions
        rclcpp::QoS qos_transient_local(10);
        qos_transient_local.transient_local();

        state_sub_ = this->create_subscription<roscopter_msgs::msg::State>(
            "estimated_state", 10, std::bind(&MinimumSnapManager::state_cb, this, _1));
        
        wp_sub_ = this->create_subscription<roscopter_msgs::msg::Waypoint>(
            "waypoints", qos_transient_local, std::bind(&MinimumSnapManager::waypoint_cb, this, _1));
        
        // World-NED hit points. The default is the mock's truth-pose cloud; "lidar/points_ned" is
        // lidar_preprocessor's output, placed with the ESTIMATED pose as on the vehicle.
        this->declare_parameter("lidar_topic", "/mock_env/lidar");
        const std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, 10, std::bind(&MinimumSnapManager::lidar_cb, this, _1));
        RCLCPP_INFO(this->get_logger(), "LiDAR input: %s", lidar_topic.c_str());

        // 5. Publishers
        traj_cmd_pub_ = this->create_publisher<roscopter_msgs::msg::TrajectoryCommand>("trajectory_command", 10);
        sfc_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("sfc_markers", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory_path", 10);
        comp_times_pub_ = this->create_publisher<roscopter_msgs::msg::ComputationTimes>(
            "computation_times", 10);

        // 6. Control and Replan Loops
        // Flight-controller status. Metrics are only accumulated while the autopilot actually
        // has control -- see metrics_active().
        status_sub_ = this->create_subscription<rosflight_msgs::msg::Status>(
            "status", 1, [this](const rosflight_msgs::msg::Status::SharedPtr msg) {
                have_status_ = true;
                const bool in_control =
                    msg->armed && msg->offboard &&
                    msg->rc_override == rosflight_msgs::msg::Status::OVERRIDE_NO_OVERRIDE;
                if (in_control != autopilot_in_control_) {
                    RCLCPP_INFO(this->get_logger(), "Autopilot control %s -- metrics %s.",
                                in_control ? "ACQUIRED" : "released",
                                in_control ? "running" : "paused");
                }
                autopilot_in_control_ = in_control;
            });

        // Contact with the world. Subscribed here rather than read from a separate terminal so
        // that a run's success verdict is printed next to the numbers it qualifies -- see
        // CollisionMonitor for why that matters with non-solid obstacles.
        collision_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "mock_env/collision", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                const bool was_clean = (collisions_.events == 0);
                collisions_.update(msg->data, this->now(), metrics_active());
                if (was_clean && collisions_.events > 0) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "COLLISION during a measured leg -- this run is a FAILURE for "
                                 "benchmarking purposes. Obstacles are not solid, so the flight "
                                 "will continue and the metrics will look healthy.");
                }
            });

        dispatch_timer_ = this->create_wall_timer(20ms, std::bind(&MinimumSnapManager::dispatch_loop, this)); // 50 Hz
        replan_timer_ = this->create_wall_timer(100ms, std::bind(&MinimumSnapManager::replan_loop, this)); // 10 Hz
        
        has_state_ = false;
        initial_pos_set_ = false;
        active_waypoint_ = false;
        grid_initialized_ = false;
        mission_started_ = false;
        first_waypoint_done_ = false;
    }

private:
    void state_cb(const roscopter_msgs::msg::State::SharedPtr msg) {
        state_ = *msg;
        has_state_ = true;
        if (!initial_pos_set_) {
            init_x_ = state_.p_n;
            init_y_ = state_.p_e;
            init_z_ = state_.p_d;
            initial_pos_set_ = true;
        }
    }

    void lidar_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // The beam origin is needed to know which space was *measured empty*, not just
        // where the returns landed. The LiDAR is body-mounted and the sim casts from the
        // vehicle position, so the current state estimate is the origin. Without a state
        // we cannot say where the beams came from, so there is nothing safe to record.
        if (!has_state_) return;

        const Eigen::Vector3d sensor_origin(state_.p_n, state_.p_e, state_.p_d);

        std::vector<Eigen::Vector3d> hits;
        hits.reserve(msg->width * msg->height);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            hits.emplace_back(*iter_x, *iter_y, *iter_z);
        }

        // Builds the three-state map: FREE along every beam, OCCUPIED at the returns,
        // UNKNOWN everywhere else.
        //
        // Timed, because this callback shares the single-threaded executor with the 50 Hz
        // dispatch timer and is the only other thing on it doing per-beam work. The dispatch
        // rate has sat near 12 Hz while the planner itself finishes in 8 ms, so roughly
        // two-thirds of the thread is unaccounted for -- and three successive estimates of
        // where it goes (raycast length, inflation stamping, KD-tree rebuild) have each failed
        // to move the number. Measure it instead of modelling it.
        const auto t0 = std::chrono::steady_clock::now();
        shared_grid_->update_from_scan(sensor_origin, hits);
        const double ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
                .count();

        scan_ms_sum_ += ms;
        scan_ms_max_ = std::max(scan_ms_max_, ms);
        scan_points_sum_ += hits.size();
        if (++scan_n_ >= 50) {
            const double mean = scan_ms_sum_ / scan_n_;
            RCLCPP_INFO(this->get_logger(),
                        "Map update: mean %.2f ms, max %.2f ms over %zu scans "
                        "(%.0f pts/scan). At 10 Hz that is %.0f%% of one thread; the 50 Hz "
                        "dispatch loop shares it.",
                        mean, scan_ms_max_, scan_n_,
                        static_cast<double>(scan_points_sum_) / scan_n_, mean * 10.0 / 10.0);
            scan_ms_sum_ = 0.0;
            scan_ms_max_ = 0.0;
            scan_points_sum_ = 0;
            scan_n_ = 0;
        }
        grid_initialized_ = true;
    }

    void waypoint_cb(const roscopter_msgs::msg::Waypoint::SharedPtr msg) {
        if (msg->clear_wp_list) {
            wp_queue_.clear();
            setpoint_deque_.clear(); setpoint_from_planner_.clear();
            splice_countdown_ = 0;
            splice_armed_ = true;
            active_waypoint_ = false;
            return;
        }
        wp_queue_.push_back(*msg);
        mission_started_ = true;
        RCLCPP_INFO(this->get_logger(), "Waypoint queued. Queue size: %zu", wp_queue_.size());
    }

    // Direct (path_manager-style) quintic straight-line navigation, with no SFC/
    // obstacle avoidance. Used only for the very first commanded waypoint, since
    // that leg typically starts on/near the ground where SFC + min-snap planning
    // is not well-posed. See path_manager.cpp's quintic_interpolation for the
    // reference implementation this is ported from (Beard & McLain Ch. 14, Algorithm 15).
    /**
     * Quintic from the CURRENT state (position, velocity, acceleration) to rest at end_pos.
     *
     * This used to hardcode a rest-to-rest profile -- sigma(tau) = 6t^5 - 15t^4 + 10t^3, whose
     * first and second derivatives are both zero at tau = 0. That is correct only if the vehicle
     * is already stationary. Whenever the bypass took over mid-flight (closing on a waypoint at
     * ~1.5 m/s, which is the common case) the very first setpoint commanded ZERO velocity, so
     * the vehicle was told to stop dead in one 20 ms tick. That is the aggressive jerk seen on
     * waypoint arrival, and it shows up in the metrics as DIR jerk_rms of 6-12 against the
     * planner's 1.10. It is also a real hazard: the lurch moves the vehicle somewhere the
     * planner did not expect, so the next plan starts from a stale assumption.
     *
     * Carrying (v0, a0) through as boundary conditions costs nothing and removes the
     * discontinuity -- the handover from spline to quintic becomes C2, like every other splice.
     */
    void generate_direct_trajectory(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos,
                                     double start_psi, double end_psi, int stitch_index,
                                     const Eigen::Vector3d& start_vel = Eigen::Vector3d::Zero(),
                                     const Eigen::Vector3d& start_acc = Eigen::Vector3d::Zero()) {
        auto sigma = [](double tau) { return 6*std::pow(tau,5) - 15*std::pow(tau,4) + 10*std::pow(tau,3); };
        auto sigma_p = [](double tau) { return 30*std::pow(tau,4) - 60*std::pow(tau,3) + 30*std::pow(tau,2); };
        auto sigma_pp = [](double tau) { return 120*std::pow(tau,3) - 180*std::pow(tau,2) + 60*tau; };

        double norm = (end_pos - start_pos).norm();
        // The initial climb gets its own, gentler speed.
        //
        // v_max/a_max are sized for the cruise -- 5 m/s and 20 m/s^2 for the forest benchmark --
        // and applying them to a lift-off produces a genuinely violent takeoff. The 2.0 s
        // duration floor below is what actually binds for a 3 m climb, and a quintic covering
        // 3 m in 2 s peaks at 2.81 m/s and 4.33 m/s^2. Nothing about leaving the ground needs
        // that; it just makes the vehicle harder to track through the one phase where it starts
        // from rest against an unmodelled ground contact.
        //
        // At takeoff_speed 1.0 m/s the same climb takes 5.6 s and peaks at 0.55 m/s^2.
        const bool is_takeoff = !first_waypoint_done_;
        double max_vel = std::max(this->get_parameter("v_max").as_double(), 0.1);
        double max_accel = std::max(this->get_parameter("a_max").as_double(), 0.1);
        if (is_takeoff) {
            max_vel = std::max(takeoff_speed_, 0.1);
            // Hold the acceleration limit proportionate, or t_acc_limited stays tiny and the
            // velocity term alone decides the duration -- which works, but leaves the profile's
            // peak acceleration set by a limit that has nothing to do with taking off.
            max_accel = std::min(max_accel, 4.0 * max_vel);
        }

        double t_vel_limited = sigma_p(0.5) * norm / max_vel;
        double t_acc_limited = std::sqrt(sigma_pp((3.0 - std::sqrt(3.0)) / 6.0) * norm / max_accel);
        double duration = std::max({t_vel_limited, t_acc_limited, 2.0});
        // Also allow enough time to bleed off the speed we are carrying in, otherwise the
        // profile has to decelerate harder than a_max to make the deadline.
        const double v0n = start_vel.norm();
        if (v0n > 1e-3) duration = std::max(duration, 1.5 * v0n / max_accel);

        // Clear old buffer past stitch point
        if (setpoint_deque_.size() > (size_t)stitch_index) {
            setpoint_deque_.erase(setpoint_deque_.begin() + stitch_index + 1, setpoint_deque_.end());
            setpoint_from_planner_.erase(setpoint_from_planner_.begin() + std::min<size_t>(stitch_index + 1, setpoint_from_planner_.size()), setpoint_from_planner_.end());
        }
        splice_countdown_ = setpoint_deque_.size();
        splice_armed_ = true;

        double dpsi = end_psi - start_psi;
        int num_points = std::ceil(duration / 0.02);
        for (int i = 1; i <= num_points; ++i) {
            double tau = std::min((double)i / num_points, 1.0);
            double s = sigma(tau);
            double s_p = sigma_p(tau);
            double s_pp = sigma_pp(tau);

            roscopter_msgs::msg::TrajectoryCommand cmd;
            const double t = tau * duration;
            for (int k = 0; k < 3; ++k) {
                const double dp = end_pos[k] - start_pos[k];
                if (std::abs(start_vel[k]) < 1e-6 && std::abs(start_acc[k]) < 1e-6) {
                    // Starting from rest on this axis: the classic profile is exact.
                    cmd.position[k] = start_pos[k] + s * dp;
                    cmd.velocity[k] = s_p * dp / duration;
                    cmd.acceleration[k] = s_pp * dp / (duration * duration);
                } else {
                    // General quintic p(0)=p0, p'(0)=v0, p''(0)=a0 -> p(T)=p1, p'(T)=0, p''(T)=0.
                    const double T = duration, v0 = start_vel[k], a0 = start_acc[k];
                    const double c3 = (20.0 * dp - 12.0 * v0 * T - 3.0 * a0 * T * T) / (2.0 * T * T * T);
                    const double c4 = (-30.0 * dp + 16.0 * v0 * T + 3.0 * a0 * T * T) / (2.0 * T * T * T * T);
                    const double c5 = (12.0 * dp - 6.0 * v0 * T - a0 * T * T) / (2.0 * T * T * T * T * T);
                    cmd.position[k] = start_pos[k] + v0 * t + 0.5 * a0 * t * t
                                      + c3 * t * t * t + c4 * t * t * t * t + c5 * t * t * t * t * t;
                    cmd.velocity[k] = v0 + a0 * t + 3.0 * c3 * t * t + 4.0 * c4 * t * t * t
                                      + 5.0 * c5 * t * t * t * t;
                    cmd.acceleration[k] = a0 + 6.0 * c3 * t + 12.0 * c4 * t * t + 20.0 * c5 * t * t * t;
                }
            }
            cmd.psi = start_psi + s * dpsi;
            cmd.psi_dot = s_p * dpsi / duration;
            cmd.psi_dot_dot = s_pp * dpsi / (duration * duration);
            setpoint_deque_.push_back(cmd);
            setpoint_from_planner_.push_back(0);
        }
    }

    // Mirrors SANDO's dynus_interfaces/ComputationTimes so the two planners can be
    // benchmarked stage-by-stage. Note comparable_total_ms deliberately excludes nothing
    // on our side but corresponds to only PART of their total_replanning_ms -- they also
    // time yaw sequencing/fitting and safety checking, which this planner does not do.
    void publish_computation_times(const trajectory_planner::PlanningResult& result) {
        roscopter_msgs::msg::ComputationTimes msg;
        msg.header.stamp = this->now();
        msg.result = result.success;
        msg.reached_goal = result.reached_goal;

        msg.total_replanning_ms = result.total_time_ms;
        msg.front_end_ms = result.sfc_time_ms;
        msg.matrix_build_ms = result.matrix_time_ms;
        msg.back_end_ms = result.opt_time_ms;
        msg.overhead_ms = result.overhead_ms;
        msg.comparable_total_ms =
            result.sfc_time_ms + result.matrix_time_ms + result.opt_time_ms;

        // Running distribution of total planning time. Every replan counts, including
        // failures -- those are the expensive ones, so excluding them would flatter the tail.
        if (metrics_active()) plan_time_stats_.add(result.total_time_ms);
        if (metrics_active()) { opt_time_stats_.add(result.opt_time_ms);
                                opt_time_all_.push_back(result.opt_time_ms); }
        // Keep the full stage breakdown of the leg's WORST cycle. The mean says the planner is
        // fine; the max is what starves the 50 Hz dispatch loop, and knowing whether it was the
        // front-end or a pile of solver retries is the difference between tuning
        // astar_max_nodes and tuning time_budget_factor.
        if (metrics_active() && result.total_time_ms > worst_.total_ms) {
            worst_ = {result.total_time_ms, result.sfc_time_ms, result.matrix_time_ms,
                      result.opt_time_ms, result.overhead_ms, result.solve_attempts,
                      static_cast<int>(result.corridors.size()), result.total_control_points};
        }
        if (metrics_active() && !result.corridors.empty()) {
            plan_cps_.push_back(static_cast<double>(result.total_control_points));
            plan_sfc_.push_back(static_cast<double>(result.corridors.size()));
            double arc = 0.0;
            for (const auto& c : result.corridors)
                arc += (c.secondaryPosition - c.primaryPosition).norm();
            const double dur = std::max(1.0, static_cast<double>(result.total_control_points)
                                             - static_cast<double>(fe_config_.degree));
            plan_dur_speed_.push_back(arc / dur);
        }
        msg.replan_count  = static_cast<uint32_t>(plan_time_stats_.count());
        msg.leg           = static_cast<uint16_t>(leg_stats_.size() + 1);
        msg.min_total_ms  = plan_time_stats_.min();
        msg.max_total_ms  = plan_time_stats_.max();
        msg.mean_total_ms = plan_time_stats_.mean();

        msg.solve_attempts = static_cast<uint16_t>(result.solve_attempts);
        msg.num_corridors = static_cast<uint16_t>(result.corridors.size());
        msg.num_control_points = static_cast<uint16_t>(result.total_control_points);

        comp_times_pub_->publish(msg);
    }

    /** A single DELETEALL marker; prepend to every MarkerArray publish to clear the last set. */
    static visualization_msgs::msg::Marker make_delete_all_marker() {
        visualization_msgs::msg::Marker m;
        m.ns = "sfc";
        m.action = visualization_msgs::msg::Marker::DELETEALL;
        return m;
    }

    /** Clears the corridor and trajectory overlays. Used when a replan produces nothing. */
    void clear_visualization() {
        visualization_msgs::msg::MarkerArray clear_markers;
        clear_markers.markers.push_back(make_delete_all_marker());
        sfc_pub_->publish(clear_markers);

        nav_msgs::msg::Path empty_path;
        empty_path.header.stamp = this->now();
        empty_path.header.frame_id = "NED";
        path_pub_->publish(empty_path);
    }

    /**
     * Append a constant-deceleration ramp so the committed buffer always ends at REST.
     *
     * The planner may finish a trajectory still moving when it stopped at a horizon rather
     * than the goal (see the terminal-velocity reasoning in trajectory_planner.cpp). That is
     * only safe because a full stop is guaranteed to fit inside measured-free space -- and a
     * guarantee nobody can execute is not a guarantee. Without this tail, a replan that
     * fails to arrive leaves dispatch_loop draining a buffer that ends at speed and then
     * falling back to holding the last position, which is an instantaneous velocity step:
     * the vehicle overshoots into exactly the unobserved space the bound was protecting.
     *
     * In normal operation this tail is never flown -- the next replan replaces it 100 ms
     * later. It exists for the cycle where that does not happen.
     */
    void append_braking_tail() {
        if (setpoint_deque_.empty()) return;

        Eigen::Vector3d v(setpoint_deque_.back().velocity[0],
                          setpoint_deque_.back().velocity[1],
                          setpoint_deque_.back().velocity[2]);
        const double v0 = v.norm();
        if (v0 < 0.05) return;   // already at rest

        const double a_max = std::max(this->get_parameter("a_max").as_double(), 0.1);
        const Eigen::Vector3d dir = v / v0;
        Eigen::Vector3d p(setpoint_deque_.back().position[0],
                          setpoint_deque_.back().position[1],
                          setpoint_deque_.back().position[2]);
        const double psi = setpoint_deque_.back().psi;

        // Constant deceleration: stops in v0/a_max seconds over v0^2/(2*a_max) metres, which
        // is the same quantity the planner bounded the terminal velocity against.
        const int steps = static_cast<int>(std::ceil((v0 / a_max) / 0.02));
        for (int i = 1; i <= steps; ++i) {
            const double t = i * 0.02;
            const double s = std::max(0.0, v0 - a_max * t);          // speed after braking
            const double d = v0 * t - 0.5 * a_max * t * t;           // distance covered

            roscopter_msgs::msg::TrajectoryCommand cmd;
            const Eigen::Vector3d pos = p + dir * std::min(d, (v0 * v0) / (2.0 * a_max));
            cmd.position[0] = pos.x(); cmd.position[1] = pos.y(); cmd.position[2] = pos.z();
            const Eigen::Vector3d vel = dir * s;
            cmd.velocity[0] = vel.x(); cmd.velocity[1] = vel.y(); cmd.velocity[2] = vel.z();
            const Eigen::Vector3d acc = (s > 0.0) ? Eigen::Vector3d(-dir * a_max)
                                                  : Eigen::Vector3d::Zero();
            cmd.acceleration[0] = acc.x(); cmd.acceleration[1] = acc.y(); cmd.acceleration[2] = acc.z();
            cmd.psi = psi; cmd.psi_dot = 0; cmd.psi_dot_dot = 0;
            setpoint_deque_.push_back(cmd);
            setpoint_from_planner_.push_back(0);
        }
    }


    /**
     * Quality of the trajectory the vehicle was actually COMMANDED to fly.
     *
     * Accumulated from setpoints as dispatch_loop pops them, not from PlanningResult. That
     * distinction matters: the planner re-solves at 10 Hz but only ~1 s of each solution is ever
     * flown before the next replan replaces it, so summing per-plan metrics would count the same
     * stretch of path ten times over and report a mission "energy" an order of magnitude too
     * high. What is dispatched is flown exactly once, so these totals are additive.
     *
     * Definitions match trajectory_metrics.hpp so a leg here is directly comparable with a
     * single-shot evaluate() on one trajectory -- integrals are trapezoidal over uniform time,
     * at the 50 Hz dispatch period.
     */
    struct FlownQuality {
        double duration = 0.0;          ///< s, dispatch periods actually issued
        double path_length = 0.0;       ///< m, summed over commanded positions
        double max_speed = 0.0;         ///< m/s
        double max_accel = 0.0;         ///< m/s^2
        double control_effort = 0.0;    ///< integral ||a||^2 dt
        double thrust_integral = 0.0;   ///< integral ||a - g|| dt, the energy proxy
        double jerk_sq_sum = 0.0;       ///< running sum for jerk_rms
        std::size_t jerk_n = 0;
        /// Integral ||j|| dt -- SANDO's S_jerk. Reported ALONGSIDE jerk_rms rather than replacing
        /// it: the L1 integral is what their Table IV/VI quotes, but it grows with trajectory
        /// duration, so it cannot be compared between runs of different length the way an RMS can.
        double jerk_l1 = 0.0;

        // --- Per-axis (L-infinity) kinodynamic accounting -------------------------------------
        // The MINVO constraints bound each AXIS at v_max/a_max, not the norm, so a per-axis
        // measurement is the one that matches what the QP actually enforces -- and it is also
        // what SANDO reports (they enforce hard L-inf constraints and relaxed the methods they
        // compare against to L-inf "for fair comparison"). Reporting a norm peak against a
        // per-axis limit is what made a_max read 2.30 against a stated 2.0 and look like a breach.
        std::size_t n_samples = 0;
        std::size_t n_vel_violation = 0;
        std::size_t n_accel_violation = 0;
        double max_axis_speed = 0.0;    ///< max over axes and samples of |v_i|
        double max_axis_accel = 0.0;    ///< max over axes and samples of |a_i|
        /// Limits the violation counters are measured against. Deliberately survive reset() --
        /// the accumulators are cleared per leg, but the limits are a property of the mission.
        double v_limit = 0.0;
        double a_limit = 0.0;

        bool has_prev = false;
        Eigen::Vector3d prev_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d prev_acc = Eigen::Vector3d::Zero();

        double jerk_rms() const { return jerk_n ? std::sqrt(jerk_sq_sum / jerk_n) : 0.0; }
        double mean_speed() const { return duration > 0.0 ? path_length / duration : 0.0; }
        double vel_violation_pct() const {
            return n_samples ? 100.0 * static_cast<double>(n_vel_violation) / n_samples : 0.0;
        }
        double accel_violation_pct() const {
            return n_samples ? 100.0 * static_cast<double>(n_accel_violation) / n_samples : 0.0;
        }

        void add(const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a,
                 double dt, bool spliced) {
            static const Eigen::Vector3d kGravityNed(0.0, 0.0, 9.81);
            // Distance may only be accumulated between CONSECUTIVE samples. `spliced` marks a
            // break in the sample stream -- a replan join, or (for the per-generator buckets) a
            // switch to the other generator and back. Gating on has_prev alone charged this
            // bucket for everything the vehicle flew while the OTHER generator was driving:
            // the DIR row reported 101.10 m against its own v_max of 0.26 m/s over 30.42 s, and
            // PLN + DIR summed to 212.75 m on a 114.93 m mission.
            if (has_prev && !spliced) path_length += (p - prev_pos).norm();
            duration += dt;
            max_speed = std::max(max_speed, v.norm());
            max_accel = std::max(max_accel, a.norm());
            control_effort  += dt * a.squaredNorm();
            thrust_integral += dt * (a - kGravityNed).norm();

            // Per-axis peaks and violation counts. Counted over EVERY dispatched sample, not
            // only the ones that survive the jerk splice filter, so the denominator is the whole
            // commanded trajectory -- that is the "percentage of trajectory points that violate"
            // definition the comparison uses.
            ++n_samples;
            const double v_inf = v.cwiseAbs().maxCoeff();
            const double a_inf = a.cwiseAbs().maxCoeff();
            max_axis_speed = std::max(max_axis_speed, v_inf);
            max_axis_accel = std::max(max_axis_accel, a_inf);
            if (v_limit > 0.0 && v_inf > v_limit) ++n_vel_violation;
            if (a_limit > 0.0 && a_inf > a_limit) ++n_accel_violation;
            // Jerk by finite difference of commanded acceleration. Samples straddling a replan
            // splice are DISCARDED: consecutive plans are only C2-continuous at the stitch point
            // to within solver tolerance, and differencing across that step produces a spike
            // worth tens of m/s^3 that would swamp the statistic. Skipping one sample per replan
            // costs ~2% of them.
            if (has_prev && !spliced) {
                const Eigen::Vector3d j = (a - prev_acc) / dt;
                jerk_sq_sum += j.squaredNorm();
                jerk_l1 += dt * j.norm();
                ++jerk_n;
            }
            prev_pos = p;
            prev_acc = a;
            has_prev = true;
        }

        void merge(const FlownQuality& o) {
            duration += o.duration;
            path_length += o.path_length;
            max_speed = std::max(max_speed, o.max_speed);
            max_accel = std::max(max_accel, o.max_accel);
            control_effort += o.control_effort;
            thrust_integral += o.thrust_integral;
            jerk_sq_sum += o.jerk_sq_sum;
            jerk_n += o.jerk_n;
            jerk_l1 += o.jerk_l1;
            n_samples += o.n_samples;
            n_vel_violation += o.n_vel_violation;
            n_accel_violation += o.n_accel_violation;
            max_axis_speed = std::max(max_axis_speed, o.max_axis_speed);
            max_axis_accel = std::max(max_axis_accel, o.max_axis_accel);
        }

        /// Clears the accumulators but KEEPS the limits -- they describe the mission, not the leg,
        /// and wiping them would silently zero every violation count after the first leg.
        void reset() {
            const double v = v_limit, a = a_limit;
            *this = FlownQuality{};
            v_limit = v;
            a_limit = a;
        }
        void set_limits(double v, double a) { v_limit = v; a_limit = a; }
    };

    using CollisionMonitor = roscopter::CollisionMonitor;
    using RouteChurn = roscopter::RouteChurn;

    struct WorstCycle {
        double total_ms = 0.0, sfc_ms = 0.0, matrix_ms = 0.0, opt_ms = 0.0, overhead_ms = 0.0;
        int attempts = 0, corridors = 0, control_points = 0;
    };

    struct LegTiming {
        int leg;
        std::size_t replans;
        double min_ms, mean_ms, max_ms;
        WorstCycle worst;
        FlownQuality quality;         ///< every setpoint, whatever produced it
        FlownQuality planned;         ///< spline samples only -- comparable across legs
        bool fallback_arrival;        ///< ended on stalled progress, not inside arrival_radius
        double arrival_dist;          ///< estimated distance to the waypoint when it was accepted
        bool arrival_reached_goal;    ///< planner's last verdict; splits planner- from tracking-side
        /// WALL-CLOCK seconds for this leg, start of the leg to arrival.
        ///
        /// NOT the same as quality.duration, and the difference is the whole point. duration is
        /// TRAJECTORY time: it accumulates kDispatchDt per setpoint consumed from the buffer. A
        /// setpoint that never existed is never counted, so whenever planning fails and the
        /// buffer drains, the vehicle hovers on the fallback path -- which does not go through
        /// record_dispatched -- and that time VANISHES from the metric.
        ///
        /// Measured on hard s1 at v5: 8163 replans at 10 Hz is ~816 s of wall clock, and the
        /// run reported T_trav 57.98 s and a PASS. It stalled for minutes and scored BETTER
        /// than a clean 68.70 s run. Averaging that with clean runs inverts the result.
        double wall_s = 0.0;
        /// wall_s minus the trajectory time actually commanded: seconds the vehicle spent with
        /// nothing to fly. This is the stall, stated directly.
        double uncommanded_s = 0.0;
    };

    /**
     * Planning-time breakdown, printed when the last queued waypoint is reached.
     *
     * Per-leg rows plus a mission total. The MAX column is the one that matters: a 10 Hz
     * loop is broken by its worst cycle, not its average, and a single 120 ms replan starves
     * the 50 Hz dispatch loop no matter how good the mean looks.
     */
    /**
     * Dump everything the planner had when it stalled, so the failure can be replayed offline.
     *
     * WHY THE MAP IS THE POINT. This stall only happens on a PARTIAL map -- at the reproducible
     * stall position the vehicle sits outside the trunk field where the LiDAR has swept almost
     * nothing, so A* is routing largely through UNKNOWN space. Seven separate hypotheses about
     * it were each tested against an offline harness built from GROUND-TRUTH obstacles, and
     * every one died on that gap: a fully-known map is not the map the planner had. Guessing
     * from summary statistics cost a sim run per attempt and produced no answer.
     *
     * Only the RAW sets are written. The inflated set, continuous bounds, KD-tree and heat field
     * are pure functions of them, so the replay rebuilds an identical map from this alone.
     */
    void write_stall_snapshot(double dist_to_goal) {
        std::string path = this->get_parameter("stall_snapshot_path").as_string();
        if (path.empty() || !shared_grid_) return;

        // One file PER EPISODE. The run no longer ends at the first stall (stall_ends_run), so
        // a fixed path would let a later stall overwrite the one actually worth looking at.
        // First episode keeps the plain name so existing tooling and docs still apply.
        if (stall_episode_ > 0) {
            const std::size_t dot = path.find_last_of('.');
            const std::string stem = (dot == std::string::npos) ? path : path.substr(0, dot);
            const std::string ext = (dot == std::string::npos) ? "" : path.substr(dot);
            path = stem + "_" + std::to_string(stall_episode_) + ext;
        }
        ++stall_episode_;

        std::ofstream out(path);
        if (!out) {
            RCLCPP_ERROR(this->get_logger(), "Could not write stall snapshot to %s", path.c_str());
            return;
        }
        out << std::setprecision(10);
        out << "# stall snapshot -- replay with tools/replay_stall.cpp\n";
        out << "resolution " << shared_grid_->get_resolution() << "\n";
        out << "inflation " << shared_grid_->get_inflation_radius() << "\n";
        out << "v_max " << this->get_parameter("v_max").as_double() << "\n";
        out << "a_max " << this->get_parameter("a_max").as_double() << "\n";
        out << "planning_horizon " << fe_config_.planning_horizon << "\n";
        out << "max_sfc_count " << fe_config_.max_sfc_count << "\n";
        out << "min_chain_length " << fe_config_.min_chain_length << "\n";
        out << "max_sfc_count_hard " << fe_config_.max_sfc_count_hard << "\n";
        out << "unknown_traversal_cost " << fe_config_.unknown_traversal_cost << "\n";
        out << "astar_max_nodes " << fe_config_.astar_max_nodes << "\n";
        out << "astar_heuristic_weight " << fe_config_.astar_heuristic_weight << "\n";
        out << "min_altitude " << fe_config_.min_altitude << "\n";
        out << "max_altitude " << fe_config_.max_altitude << "\n";
        out << "drone_radius " << fe_config_.drone_physical_radius << "\n";
        out << "geofence_contains_vehicle "
            << (fe_config_.geofence_contains_vehicle ? 1 : 0) << "\n";
        out << "dist_to_goal " << dist_to_goal << "\n";

        // The state the next plan would be built from. The anchor replan_loop actually uses is
        // the last dispatched COMMAND, not the vehicle pose -- they differ, and which one is
        // wrong is exactly what needs settling.
        out << "vehicle " << state_.p_n << " " << state_.p_e << " " << state_.p_d << "\n";
        out << "goal " << current_wp_.w[0] << " " << current_wp_.w[1] << " "
            << current_wp_.w[2] << "\n";
        if (have_last_dispatched_) {
            out << "last_cmd_pos " << last_dispatched_.position[0] << " "
                << last_dispatched_.position[1] << " " << last_dispatched_.position[2] << "\n";
            out << "last_cmd_vel " << last_dispatched_.velocity[0] << " "
                << last_dispatched_.velocity[1] << " " << last_dispatched_.velocity[2] << "\n";
        }
        out << "buffer " << setpoint_deque_.size() << "\n";

        const auto& occ = shared_grid_->get_occupied_raw();
        const auto& fre = shared_grid_->get_free();
        out << "occupied " << occ.size() << "\n";
        for (const auto& c : occ) out << c.x() << " " << c.y() << " " << c.z() << "\n";
        out << "free " << fre.size() << "\n";
        for (const auto& c : fre) out << c.x() << " " << c.y() << " " << c.z() << "\n";
        out.close();

        RCLCPP_ERROR(this->get_logger(),
                     "Stall snapshot written to %s (%zu occupied, %zu free cells). Replay it "
                     "offline: this map is the variable every previous hypothesis was missing.",
                     path.c_str(), occ.size(), fre.size());
    }

    /**
     * The run summary. Printed on mission completion, on a run-ending stall, and -- since a
     * stall no longer ends the run by default -- from the destructor when the process is
     * stopped by hand.
     *
     * THAT LAST PATH MATTERS. With stall_ends_run false a wedged vehicle hovers until someone
     * interrupts it, and the only two callers before were "mission complete" and
     * "mission_stalled_". Neither fires on Ctrl-C, so interrupting a hung run threw away every
     * metric it had gathered -- the worst possible outcome for a benchmark, because the runs
     * you most want to characterise are exactly the ones you have to stop.
     */
    void print_planning_time_table() {
        if (leg_stats_.empty()) return;
        if (summary_printed_) return;   // mission end followed by shutdown must not print twice
        summary_printed_ = true;

        std::size_t total_replans = 0;
        double t_min = 1e300, t_max = 0.0, weighted_sum = 0.0;
        for (const auto& l : leg_stats_) {
            total_replans += l.replans;
            t_min = std::min(t_min, l.min_ms);
            t_max = std::max(t_max, l.max_ms);
            weighted_sum += l.mean_ms * static_cast<double>(l.replans);
        }
        const double t_mean = total_replans ? weighted_sum / total_replans : 0.0;

        std::ostringstream os;
        os << "\n"
           << "==========================================================\n"
           << "            PLANNING TIME  (per waypoint leg)\n"
           << "==========================================================\n"
           << "  leg   replans     min[ms]    mean[ms]     max[ms]\n"
           << "  ----------------------------------------------------\n";
        os << std::fixed << std::setprecision(2);
        for (const auto& l : leg_stats_) {
            os << "  " << std::setw(3) << l.leg
               << std::setw(10) << l.replans
               << std::setw(12) << l.min_ms
               << std::setw(12) << l.mean_ms
               << std::setw(12) << l.max_ms << "\n";
        }
        os << "  ----------------------------------------------------\n"
           << "  ALL " << std::setw(10) << total_replans
           << std::setw(12) << t_min
           << std::setw(12) << t_mean
           << std::setw(12) << t_max << "\n"
           << "==========================================================\n"
           << "  budget 100 ms @ 10 Hz -- watch the max, not the mean.\n";
            if (!opt_time_all_.empty()) {
                std::vector<double> v = opt_time_all_;
                std::sort(v.begin(), v.end());
                auto q = [&v](double f) { return v[std::min(v.size() - 1,
                                     static_cast<std::size_t>(f * v.size()))]; };
                os << "\n  OSQP STAGE ALONE over " << v.size() << " solves"
                   << "   mean " << opt_time_stats_.mean()
                   << "   med " << q(0.50) << "   p90 " << q(0.90)
                   << "   p99 " << q(0.99) << "   max " << opt_time_stats_.max() << " ms\n"
                   << "    This is T_opt -- the QP alone. The table above is END-TO-END replan\n"
                   << "    time (front end + matrix build + QP + overhead). Quote both.\n";
            }

        // --- Where the worst cycle of each leg actually went ---
        os << "\n"
           << "  WORST CYCLE PER LEG (stage breakdown of the max above)\n"
           << "  leg    total    front-end   matrix     osqp   overhead  tries  sfc   cps\n"
           << "  ---------------------------------------------------------------------------\n";
        for (const auto& l : leg_stats_) {
            os << "  " << std::setw(3) << l.leg
               << std::setw(9) << l.worst.total_ms
               << std::setw(12) << l.worst.sfc_ms
               << std::setw(9) << l.worst.matrix_ms
               << std::setw(9) << l.worst.opt_ms
               << std::setw(11) << l.worst.overhead_ms
               << std::setw(7) << l.worst.attempts
               << std::setw(5) << l.worst.corridors
               << std::setw(6) << l.worst.control_points << "\n";
        }
        os << "  ---------------------------------------------------------------------------\n"
           << "  tries > 1 means the QP was re-solved with more time (see time_budget_factor);\n"
           << "  a front-end-dominated max points at astar_max_nodes / planning_horizon instead.\n";

        // --- How each leg ended ---
        //
        // Reported because a fallback arrival is NOT evidence the vehicle reached the waypoint --
        // it is the mission declining to hang. Any waypoint-accuracy claim has to exclude these
        // legs or say explicitly that they were accepted on a timeout.
        std::size_t fallback_legs = 0;
        for (const auto& l : leg_stats_) fallback_legs += l.fallback_arrival ? 1 : 0;
        if (fallback_legs > 0) {
            os << "\n"
               << "  ARRIVAL MODE\n"
               << "  leg    dist[m]   accepted by\n"
               << "  ---------------------------------------------------------------\n";
            for (const auto& l : leg_stats_) {
                os << "  " << std::setw(3) << l.leg
                   << std::setw(11) << l.arrival_dist << "   ";
                if (!l.fallback_arrival) {
                    os << "arrival radius\n";
                } else {
                    os << "STALLED PROGRESS -- "
                       << (l.arrival_reached_goal ? "tracking-side (goal was reachable)"
                                                  : "planner-side (goal was UNREACHABLE)")
                       << "\n";
                }
            }
            os << "  ---------------------------------------------------------------\n"
               << "  " << fallback_legs << " of " << leg_stats_.size()
               << " leg(s) ended outside the " << arrival_radius_ << " m arrival radius.\n"
               << "  planner-side  -> the front end could not route to the goal. Check for a real\n"
               << "                   obstacle, or one displaced by estimator drift between the\n"
               << "                   scan that mapped it and the traversal.\n"
               << "  tracking-side -> it planned there and the vehicle did not close the gap.\n"
               << "                   Check tracking_error and controller saturation.\n";
        }

        // --- Trajectory quality, measured over the setpoints actually dispatched ---
        const double v_lim = this->get_parameter("v_max").as_double();
        const double a_lim = this->get_parameter("a_max").as_double();
        FlownQuality all;
        for (const auto& l : leg_stats_) all.merge(l.quality);

        os << "\n"
           << "=======================================================================================\n"
           << "            TRAJECTORY QUALITY  (as commanded, per waypoint leg)\n"
           << "=======================================================================================\n"
           << "  leg     dur[s]   dist[m]  v_avg  v_max  a_max   jerk_rms   effort   thrust*s\n"
           << "  -------------------------------------------------------------------------------------\n";
        auto row = [&](const std::string& label, const FlownQuality& q) {
            os << "  " << std::setw(3) << label
               << std::setw(11) << q.duration
               << std::setw(10) << q.path_length
               << std::setw(7) << q.mean_speed()
               << std::setw(7) << q.max_speed
               << std::setw(7) << q.max_accel
               << std::setw(11) << q.jerk_rms()
               << std::setw(9) << q.control_effort
               << std::setw(11) << q.thrust_integral << "\n";
        };
        for (const auto& l : leg_stats_) row(std::to_string(l.leg), l.quality);
        os << "  -------------------------------------------------------------------------------------\n";
        row("ALL", all);

        // Same legs, spline samples only. Leg 1 always looks worst in the rows above because it
        // carries the liftoff, whose quintic/vertical generators command stepwise acceleration;
        // these rows are what the PLANNER did on each leg, and are comparable leg to leg.
        os << "\n"
           << "  PLANNER ONLY, PER LEG (liftoff and other direct segments excluded)\n"
           << "  -------------------------------------------------------------------------------------\n";
        for (const auto& l : leg_stats_) row(std::to_string(l.leg), l.planned);

        // Split by generator. Leg 1 is dominated by takeoff, and the non-spline generators
        // command piecewise-constant (often zero) acceleration -- so every switch into or out of
        // one is a step in a, and a 50 Hz difference turns that step into a jerk spike ~10x the
        // spline's own. Scoring the planner against a number that includes them understates it
        // badly. PLANNED is the row to quote for the planner's trajectory quality.
        os << "\n"
           << "  BY GENERATOR (same samples, attributed to what produced them)\n"
           << "  -------------------------------------------------------------------------------------\n";
        row("PLN", planned_quality_);
        row("DIR", direct_quality_);

        // --- Benchmark row, in the form the SANDO comparison expects --------------------------
        //
        // Constraint satisfaction is reported PER AXIS because that is what the QP enforces (the
        // MINVO box bounds each axis, not the norm) and what the comparison uses -- SANDO applies
        // hard L-inf constraints and relaxed the methods it benchmarks against to L-inf as well.
        // Quoting a norm peak against a per-axis limit invents violations that do not exist.
        //
        // S_jerk is the L1 integral, matching their definition. It scales with trajectory
        // duration, so it is only comparable between runs over the SAME route.
        os << "\n"
           << "  BENCHMARK METRICS (per-axis / L-inf, as SANDO Table VI reports them)\n"
           << "  -------------------------------------------------------------------------------------\n"
           << "  set     T_trav[s]  L_path[m]   S_jerk    v_inf   a_inf   rho_vel[%]  rho_acc[%]\n";
        auto bench = [&](const std::string& label, const FlownQuality& q) {
            os << "  " << std::setw(3) << label
               << std::setw(11) << q.duration
               << std::setw(11) << q.path_length
               << std::setw(10) << q.jerk_l1
               << std::setw(9) << q.max_axis_speed
               << std::setw(8) << q.max_axis_accel
               << std::setw(12) << q.vel_violation_pct()
               << std::setw(12) << q.accel_violation_pct() << "\n";
        };
        bench("PLN", planned_quality_);
        bench("ALL", all);

        // --- WALL CLOCK ---------------------------------------------------------------------
        //
        // T_trav above is TRAJECTORY time: kDispatchDt per setpoint consumed from the buffer.
        // When planning fails the buffer drains, the vehicle hovers on a fallback that never
        // reaches record_dispatched, and that time is simply absent from T_trav.
        //
        // The consequence is not a rounding error. On hard s1 at v_max 5 a run that stalled for
        // minutes -- 8163 replans at 10 Hz, ~816 s of wall clock -- reported T_trav 57.98 s and
        // PASS, scoring BETTER than a clean 68.70 s traverse. Averaged into a seed sweep, runs
        // like that do not merely add noise; they pull the mean the wrong way.
        {
            // Compare LIKE SCOPES. wall_s is summed over RECORDED legs only, while `all` is
            // mission-wide and includes the takeoff, which is not recorded as a leg. Comparing
            // them made T_wall come out BELOW commanded time -- impossible for an elapsed clock
            // -- and would have under-reported stall time by the takeoff duration on any run
            // that did stall. Use each leg's own commanded duration instead.
            double wall = 0.0, unc = 0.0, leg_cmd = 0.0;
            for (const auto& l : leg_stats_) {
                wall += l.wall_s;
                unc += l.uncommanded_s;
                leg_cmd += l.quality.duration;
            }
            if (!plan_cps_.empty()) {
                auto med = [](std::vector<double> v) {
                    std::sort(v.begin(), v.end());
                    return v[v.size() / 2];
                };
                const double one_frac =
                    static_cast<double>(std::count(plan_sfc_.begin(), plan_sfc_.end(), 1.0))
                    / static_cast<double>(plan_sfc_.size());
                os << "  -------------------------------------------------------------------------------------\n"
                   << "  PLAN SHAPE over " << plan_cps_.size() << " replans"
                   << "   median control points " << med(plan_cps_)
                   << "   median corridors " << med(plan_sfc_) << "\n"
                   << "    single-corridor plans " << (100.0 * one_frac) << "%"
                   << "   median plan-mean speed " << med(plan_dur_speed_) << " m/s\n"
                   << "    Unit knots: duration[s] = control_points - degree, so control-point COUNT\n"
                   << "    IS flight time, and each corridor junction forces a bridge pool of degree\n"
                   << "    points. Compare against the offline harness on the SAME forest files:\n"
                   << "    median 9-10 points, ~50% single-corridor, plan-mean speed 2.6 m/s. A higher\n"
                   << "    count here means the live map is fragmenting corridors that the idealised\n"
                   << "    offline scan does not -- which is a MAP problem, not a QP problem.\n";
            }
            if (wall > 0.0) {
                os << "  -------------------------------------------------------------------------------------\n"
                   << "  WALL CLOCK  T_wall " << wall << " s"
                   << "   commanded " << leg_cmd << " s"
                   << "   UNCOMMANDED " << unc << " s ("
                   << (wall > 0.0 ? 100.0 * unc / wall : 0.0) << "%)\n";
                if (unc > 0.05 * wall) {
                    os << "    *** QUOTE T_wall, NOT T_trav, FOR THIS RUN. " << unc << " s of it had no\n"
                       << "    trajectory to fly -- planning failed and the buffer was empty -- and none\n"
                       << "    of that appears in T_trav. The run stalled; T_trav describes only the\n"
                       << "    portion that was actually flown.\n";
                }
            }
        }

        // --- RUN VERDICT --------------------------------------------------------------------
        //
        // Printed INSIDE the benchmark block, immediately under the numbers it qualifies,
        // because every column above is meaningless for a run that cut through an obstacle --
        // and it will not look meaningless. Obstacles are non-solid, so a run that flew through
        // three trunks still reads as a completed traverse with the BEST T_trav and L_path of
        // the set. Do not lift rows out of this table without reading this line.
        os << "  -------------------------------------------------------------------------------------\n";
        if (!collisions_.topic_seen) {
            os << "  RUN VERDICT: *** NOT MEASURED *** -- nothing received on 'mock_env/collision'.\n"
               << "    This is NOT a clean run; it is an unchecked one. Obstacles are non-solid, so\n"
               << "    a run that flew through them is indistinguishable from one that did not.\n"
               << "    Check collision_check_enabled in the mock_environment params and that the\n"
               << "    node is up, then re-run. Do not enter these numbers in a results table.\n";
        } else if (collisions_.events == 0) {
            os << "  RUN VERDICT: PASS -- no contact while the autopilot had control.\n";
        } else {
            os << "  RUN VERDICT: *** FAIL *** -- " << collisions_.events
               << " collision event(s), " << collisions_.contact_seconds
               << " s in contact.\n"
               << "    Counts as a failed run for R_succ. WHICH metrics are also invalid depends\n"
               << "    on the cause, and the two cases differ:\n"
               << "      PLANNER  -- the corridor itself enclosed an obstacle. Flying through one\n"
               << "                  is a shortcut, so T_trav and L_path come out BETTER than a\n"
               << "                  clean run and must not be averaged with them.\n"
               << "      TRACKING -- the corridor was valid and the vehicle drifted out of it.\n"
               << "                  The commanded trajectory is still a legitimate sample; only\n"
               << "                  the success verdict is affected.\n"
               << "    Distinguish them with tracking_diagnostic.py: if horizontal cross-track is\n"
               << "    at or above the corridor clearance (hull + obstacle_margin + half a voxel,\n"
               << "    ~0.94 m at the current settings) the cause is tracking, not the planner.\n"
               << "    mock_environment's own log has the position and penetration depth of each.\n";
        }
        // --- ROUTE CHURN ---------------------------------------------------------------------
        // Does the planner keep changing its mind about where to go NEXT, or only about the far
        // field it never reaches? Decides whether route commitment is worth building -- two
        // commitment mechanisms have already been built and reverted on guesses.
        // --- DIRECTIONAL GATE ----------------------------------------------------------------
        // Report the ANGLE DISTRIBUTION, not just the rejection count. If the gate never fires,
        // the distribution is what says the threshold is wrong -- otherwise a mis-set threshold
        // is indistinguishable from a well-behaved planner, which is how two earlier diagnostics
        // in this file managed to measure nothing.
        os << "  -------------------------------------------------------------------------------------\n"
           << "  DIRECTIONAL GATE (declines a replan that turns away from the route being flown)\n";
        if (degenerate_plans_ > 0) {
            os << "    *** " << degenerate_plans_ << " plan(s) went NOWHERE with the goal still far"
               << " (" << degenerate_escapes_ << " escaped by lifting the horizon).\n"
               << "    A plan that does not move is an absorbing state -- the vehicle stays put,\n"
               << "    so the next cycle rebuilds it. Any count here means the run was at risk of\n"
               << "    stalling; a large one means it did.\n";
        }
        if (route_gate_.config().reject_angle_deg <= 0.0) {
            os << "    disabled (route_reject_angle_deg = 0)\n";
        } else {
            os << "    threshold " << route_gate_.config().reject_angle_deg << " deg over "
               << route_gate_.config().lookahead_m << " m lookahead | "
               << route_gate_.evaluated() << " judged, " << route_gate_.rejected()
               << " declined (" << route_gate_.reject_pct() << "%)\n"
               << "    forced accepts " << route_gate_.forced()
               << " (collision / goal moved / no current route), hit the "
               << route_gate_.config().max_consecutive_rejects
               << "-reject bound " << route_gate_.hit_bound() << " times (held "
               << route_gate_.bound_held() << "x for candidates aimed AWAY from the goal), "
               << "longest streak " << route_gate_.longest_reject_streak() << "\n"
               << "    accepted as better aimed at the goal: " << route_gate_.better_aimed()
               << " (these are corrections the gate must never decline -- without them one bad\n"
               << "    plan is self-defending and the vehicle holds it for a full second)\n";
            const double p50 = route_gate_.angle_percentile(0.50);
            if (p50 < 0.0) {
                os << "    no angles measured -- every cycle was a forced accept\n";
            } else {
                os << "    measured divergence, ALL candidates: p50 " << p50
                   << "  p90 " << route_gate_.angle_percentile(0.90)
                   << "  p99 " << route_gate_.angle_percentile(0.99) << " deg\n"
                   << "    if p90 sits well under the threshold the gate is inert -- lower it;\n"
                   << "    if the reject %% is high or streaks reach the bound, it is too tight.\n";
            }
        }

        os << "  -------------------------------------------------------------------------------------\n"
           << "  ROUTE CHURN (consecutive plans, arc length at which the route changes)\n"
           << "    route commitment: "
           << (this->get_parameter("commit_route").as_bool()
                 ? "ON  (plan held until collision / age / goal change)"
                 : "OFF (replans every cycle -- baseline)")
           << ", commit_max_age "
           << this->get_parameter("commit_max_age").as_double() << " s\n";
        if (route_churn_.compared() == 0) {
            os << "    no comparable replan pairs -- too few successful plans to measure.\n";
        } else {
            os << "    " << route_churn_.compared() << " replan pairs | route unchanged "
               << route_churn_.unchanged_pct() << "% | changed within "
               << route_churn_.near_field_threshold() << " m of the vehicle "
               << route_churn_.near_field_pct() << "%\n";
            route_churn_.finalize();
            os << "    sustained episodes (>=" << roscopter::RouteChurn::kEpisodeMin
               << " consecutive near-field changes, i.e. >=0.5 s of flip-flopping): "
               << route_churn_.episodes() << ", longest " << route_churn_.longest_streak()
               << " replans, " << route_churn_.episode_pct() << "% of all replans spent in one\n";
            const double p50 = route_churn_.percentile(0.50);
            if (p50 < 0.0) {
                os << "    the route never changed.\n";
            } else {
                os << "    divergence distance: p10 " << route_churn_.percentile(0.10)
                   << "  med " << p50 << "  p90 " << route_churn_.percentile(0.90) << " m\n";
            }
            os << "    READ THE EPISODES, NOT THE RATE. Churn spread thinly across a run costs\n"
               << "    little; a WEDGE -- the planner flipping between two routes for seconds\n"
               << "    while the vehicle crawls -- dominates T_trav even if it happens twice.\n"
               << "    A rate averages the two together. On hard s0 at v_max 8, 17.9% near-field\n"
               << "    churn with a 1.19 m median divergence gave 405 s / 407 m, against 74 s /\n"
               << "    164 m at 4.6% and 5.50 m. Long episodes + short divergence = junction\n"
               << "    indecision, and route commitment is the fix. Many short scattered changes\n"
               << "    far ahead are harmless -- the vehicle replans past them anyway.\n";
        }
        os << "  -------------------------------------------------------------------------------------\n"
           << "  limits: |v_i| <= " << v_lim << " m/s, |a_i| <= " << a_lim << " m/s^2 (per axis)\n"
           << "  S_jerk   = integral ||j|| dt          -- L1 jerk integral; grows with duration, so\n"
           << "             compare only across runs of the SAME route\n"
           << "  v_inf/a_inf = peak PER-AXIS magnitude -- the quantity the limits actually bound\n"
           << "  rho_vel/rho_acc = share of commanded setpoints exceeding the per-axis limit.\n"
           << "  Quote the PLN row: the planner is what is being benchmarked, and DIR's takeoff\n"
           << "  and braking segments are not min-snap trajectories.\n";

        os << "=======================================================================================\n"
           << "  effort   = integral ||a||^2 dt        -- control effort, penalises sustained aggression\n"
           << "  thrust*s = integral ||a - g|| dt      -- specific-impulse energy proxy; x mass for N*s\n"
           << "  jerk_rms = sqrt(mean ||da/dt||^2)     -- smoothness; samples at replan splices dropped\n"
           << "  v_max/a_max are peak VECTOR NORMS. The MINVO constraints are a PER-AXIS box, so\n"
           << "  the bound they guarantee is norm <= limit*sqrt(3): " << std::setprecision(2)
           << (v_lim * 1.7320508) << " m/s and " << (a_lim * 1.7320508) << " m/s^2 here.\n"
           << "  Comparing the norm directly against v_max / a_max will look like a violation\n"
           << "  when it is not.\n" << std::setprecision(2)
           << "  Totals are over COMMANDED setpoints (each flown once), not summed per-plan.\n"
           << (have_status_
                 ? "  Measured only while the autopilot had control (armed, offboard, no RC override).\n"
                 : "  WARNING: no rosflight Status seen, so time on the ground before handover IS\n"
                   "  included -- leg 1 will read long and slow. Check the 'status' topic.\n")
           << "  PLN = optimised B-spline (the planner). DIR = takeoff, vertical bypass,\n"
           << "  short-move quintic and braking tail -- none of which are min-snap, and whose\n"
           << "  acceleration steps dominate jerk. Quote PLN for the planner; PLN+DIR is the\n"
           << "  whole flight.\n";

        RCLCPP_INFO(this->get_logger(), "%s", os.str().c_str());

        leg_stats_.clear();
        leg_quality_.reset();
        planned_quality_.reset();
        direct_quality_.reset();
        last_source_ = -1;
    }

    void replan_loop() {
        // A mission that gave up beyond the arrival fallback ceiling ends here rather than
        // hovering indefinitely -- see stall_timeout. Ending it makes the run print its summary
        // so it can be scored as the failure it is.
        if (mission_stalled_) {
            if (active_waypoint_ || !wp_queue_.empty() || !setpoint_deque_.empty()) {
                active_waypoint_ = false;
                wp_queue_.clear();
                setpoint_deque_.clear();
                setpoint_from_planner_.clear();
                print_planning_time_table();
            }
            return;
        }
        if (!has_state_ || !grid_initialized_) return;

        // --- A COMMITTED SHORT MOVE MUST BE ALLOWED TO FINISH ---
        //
        // generate_direct_trajectory lays down a REST-TO-REST quintic: sigma'(0) = sigma''(0) = 0,
        // and duration has a 2 s floor. Re-planning it on the normal cadence truncated the buffer
        // a couple of hundred ms in and laid down a FRESH rest-to-rest profile from that point --
        // so the vehicle only ever executed the opening sliver of each one, where sigma(0.1) is
        // 0.0086, i.e. under 1% of the remaining distance, and then started from rest again.
        // Every restart also slammed commanded velocity back to zero, which is a step in v and
        // hence a jerk spike.
        //
        // Measured before this guard: 57.12 s of commanded flight covering 0.56 m (v_avg
        // 0.01 m/s) with a DIR jerk_rms of 12.14, and the vehicle visibly refusing to close the
        // last metre to a waypoint. A sub-1.5 m hop is one committed manoeuvre lasting ~2 s, not
        // something to re-decide ten times a second -- but it is still re-validated against the
        // newest map each cycle, and abandoned the moment that check fails.
        if (direct_move_active_) {
            if (!setpoint_deque_.empty()) {
                bool still_clear = true;
                if (shared_grid_) {
                    for (size_t i = 0; i < setpoint_deque_.size(); i += 10) {
                        const Eigen::Vector3d pt(setpoint_deque_[i].position[0],
                                                 setpoint_deque_[i].position[1],
                                                 setpoint_deque_[i].position[2]);
                        if (shared_grid_->is_occupied_radius(pt, kClearanceQueryRadius)) {
                            still_clear = false;
                            break;
                        }
                    }
                }
                if (still_clear) return;
                RCLCPP_WARN(this->get_logger(),
                            "Committed short move became blocked; abandoning it and re-planning.");
            }
            direct_move_active_ = false;
        }

        // If we don't have an active waypoint but we have a queue, pull the next one
        // Honour the waypoint's own hold time before pulling the next one.
        //
        // hold_seconds is parsed into the Waypoint message and was then IGNORED here, so the
        // mission advanced the instant the arrival test passed. On the first waypoint that runs
        // the climb transient straight into the 105 m traverse: the vehicle is still settling
        // vertically while the commanded path turns horizontal, which shows up as a large
        // vertical error that is 99% CONTROL error and an along-track lag over a metre.
        //
        // Holding lets the vehicle settle on the point before the next leg is planned from it,
        // which also means the next leg starts from a state the planner actually believes -- the
        // stitch is taken from a stationary buffer rather than mid-transient.
        if (!active_waypoint_ && !wp_queue_.empty() && this->now().seconds() >= hold_until_) {
            current_wp_ = wp_queue_.front();
            wp_queue_.erase(wp_queue_.begin());
            direct_move_active_ = false;
            active_waypoint_ = true;
            // A fresh goal has its own closest approach; carrying the previous waypoint's over
            // would leave the stall detector already armed on arrival.
            reset_arrival_tracking();
            // The stored route led to the PREVIOUS waypoint; biasing toward it now would pull
            // the search backwards.
            previous_route_.clear();
            // last_reached_goal_ still describes the PREVIOUS goal until the first replan against
            // this one lands. Clearing it keeps the fallback from firing on a stale true.
            last_reached_goal_ = false;
            // Clear buffer to force immediate replan from current state
            setpoint_deque_.clear(); setpoint_from_planner_.clear();
            splice_countdown_ = 0;
            splice_armed_ = true;
        }

        if (!active_waypoint_) return;

        // We only skip replanning if the buffer has enough, AND the plan is still fresh, AND it
        // does not collide with newly discovered obstacles.
        //
        // This used to additionally require last_reached_goal_, which made the whole gate DEAD
        // CODE on any long leg. reached_goal is computed as
        //     astar_path.back() == goal_idx && !horizon_was_clamped
        // and the horizon is clamped for the entire traverse of a 105 m leg with a 15 m horizon.
        // So the condition was false start to finish, the planner re-decided the route from
        // scratch at the full 10 Hz, and the anti-stutter behaviour this block exists to provide
        // never once applied.
        //
        // That is what let the vehicle dither at junctions: with two branches within a few
        // percent of each other, the argmin flips on tiny changes in position, and nothing held
        // the previous decision. Biasing the SEARCH toward the old route was tried instead
        // (path_hysteresis_bonus) and measured worse on the same map -- 0.20 turned cheap
        // high-frequency flipping into an expensive commit-travel-reverse oval (+21 m, +40% time)
        // and 0.30 collapsed average speed to 1.02 m/s against a 5 m/s limit. Holding a plan that
        // is still VALID is the right mechanism; making reversal expensive is not.
        //
        // A horizon-truncated plan is the normal case here, not a reason to replan immediately.
        // What actually matters is what the remaining three terms already test: enough runway
        // buffered, recent enough to reflect the map, and still collision-free. max_plan_age
        // (0.5 s) bounds how long any plan can be held, so this lowers the effective replan rate
        // during healthy flight rather than suppressing it.
        //
        // The plan-age term matters as much as the collision term. Without it, a long
        // successful trajectory satisfies the other conditions for its entire duration, so
        // the vehicle flies many seconds open-loop and NEW information is never acted on --
        // the collision check only asks "is my existing plan still safe?", never "has a
        // better route become visible?". That is especially wasteful here because the map
        // is memoryless: rounding an obstacle reveals the space behind it for exactly as
        // long as it stays in the sensor FOV, and a stale plan throws that away.
        double plan_age = this->now().seconds() - last_plan_time_;
        double max_plan_age = this->get_parameter("max_plan_age").as_double();

        bool plan_is_fresh = (max_plan_age <= 0.0) || (plan_age < max_plan_age);

        // Two gates, selected by the commit_route parameter, so the arms differ in exactly one
        // thing and nothing else in this function changes.
        //
        //   OFF (historical): requires last_reached_goal_. With max_sfc_count 3 the plan is
        //     almost always horizon-truncated, so that term is usually false and the gate
        //     effectively never fires -- the planner replans every cycle. That is the baseline
        //     whose junction dithering we are trying to remove.
        //
        //   ON (commitment): drops last_reached_goal_ -- a truncated plan is the normal case
        //     here, not a reason to throw the route away -- and holds the plan until something
        //     actually INVALIDATES it: the buffered trajectory collides against the updated map,
        //     the plan exceeds commit_max_age, or the goal moves. Dropping that term is
        //     precisely what was tried before and cost +31% travel time; the difference now is
        //     that max_sfc_count 3 cut the control-point over-allocation from ~9x to ~4x, so
        //     flying a held plan is far less expensive than it was when that was measured.
        const bool commit_route = this->get_parameter("commit_route").as_bool();
        const double commit_age = this->get_parameter("commit_max_age").as_double();

        // Goal changes invalidate a committed route outright -- the plan was optimised toward a
        // waypoint that is no longer the target, and no amount of collision-checking would
        // notice that.
        const Eigen::Vector3d goal_now(current_wp_.w[0], current_wp_.w[1], current_wp_.w[2]);
        const bool goal_unchanged = (goal_now - committed_goal_).norm() < 1e-6;

        // Two gates, selected by commit_route, so the arms differ in exactly one thing.
        //
        //   OFF (baseline): requires last_reached_goal_. At max_sfc_count 3 the plan is almost
        //     always horizon-truncated, so that is usually false and the gate effectively never
        //     fires -- the planner re-searches the route every cycle.
        //
        //   ON (commitment): drops last_reached_goal_ and holds the plan until it collides
        //     against the updated map, exceeds commit_max_age, or the goal moves.
        //
        // MEASURED, hard s0 at v_max 8, three runs per arm:
        //     OFF  T_trav med 101 s [74-405]   v_avg 1.74   sustained churn episodes present
        //     ON   T_trav med 135 s [82-146]   v_avg 1.69   episodes 0 in all three runs
        // Commitment costs 34% on the median and cuts the worst case by 64% (405 -> 146 s),
        // narrowing the spread from 5.5x to 1.8x. Which is preferable depends on whether the
        // tail or the median matters more -- it is a trade, not a win.
        //
        // The median cost is understood: holding a plan means flying its decelerating TAIL,
        // where re-searching every cycle only ever executed each plan's fast opening and so hid
        // the control-point over-allocation. The obvious repair -- hold the route, refresh the
        // trajectory -- was built and measured FAR worse; see the note at the plan_mission call.
        const bool held_fresh = commit_route
            ? ((commit_age <= 0.0) || (plan_age < commit_age))
            : plan_is_fresh;
        const bool gate_open = commit_route
            ? (setpoint_deque_.size() >= 50 && held_fresh && goal_unchanged)
            : (setpoint_deque_.size() >= 50 && last_reached_goal_ && plan_is_fresh);

        if (gate_open) {
            bool trajectory_collides = false;
            // Check every 10th setpoint (0.2s) for collision.
            //
            // COVERAGE. Samples sit v*0.2 m apart. is_occupied_radius queries the INFLATED set,
            // so each sample effectively sweeps (drone_radius + obstacle_margin +
            // kClearanceQueryRadius) = 0.94 m at current settings, not the bare hull -- an
            // earlier note here said 4.9 m/s by leaving the inflation out. Coverage is
            // continuous while v < 2*0.94/0.2 = 9.40 m/s, so the benchmark sweep at v_max
            // 2 / 5 / 8 is covered, with 8 the tightest (1.60 m spacing against 1.88 m of
            // reach). Raising v_max past ~9 m/s, or cutting obstacle_margin, opens holes --
            // tighten the stride first.
            for (size_t i = 0; i < setpoint_deque_.size(); i += 10) {
                Eigen::Vector3d pt(setpoint_deque_[i].position[0], setpoint_deque_[i].position[1], setpoint_deque_[i].position[2]);
                if (shared_grid_->is_occupied_radius(pt, kClearanceQueryRadius)) {
                    trajectory_collides = true;
                    break;
                }
            }
            if (!trajectory_collides) {
                return; // Path is fresh, smooth and safe, do not stutter!
            }
        }

        Eigen::Vector3d end_pos(current_wp_.w[0], current_wp_.w[1], current_wp_.w[2]);
        Eigen::Vector3d start_pos, start_vel, start_acc;
        
        int stitch_index = 0;
        
        if (setpoint_deque_.empty()) {
            // ANCHOR ON THE LAST COMMAND, NOT ON THE VEHICLE.
            //
            // Re-anchoring at the vehicle's own position whenever the buffer runs dry makes the
            // COMMANDED trajectory discontinuous: every such replan teleports the setpoint to
            // wherever the vehicle currently is, and the follower then chases a target that
            // jumped. Measured on hard s0 at v_max 8, commanded north position went
            // 7.97 -> 4.68 -> 2.13 -> 12.24 m -- the command moving BACKWARD 5.8 m and forward
            // again on a route whose goal is 105 m straight ahead, with the vehicle faithfully
            // following it.
            //
            // It is worse than untidy during the liftoff transient, where the buffer empties
            // repeatedly while the vehicle is moving fastest. The vehicle overshoots, the next
            // plan anchors on the overshoot, the command climbs to meet it, and the whole thing
            // ratchets: commanded altitude followed the vehicle from 0.09 m up to 4.45 m while
            // the waypoint sat at 3.0 m.
            //
            // Continuing from the last dispatched setpoint keeps the command continuous by
            // construction. The vehicle's own position is used ONLY when there is no command
            // history at all -- the very first plan -- where there is nothing else to continue
            // from.
            if (have_last_dispatched_) {
                start_pos = Eigen::Vector3d(last_dispatched_.position[0],
                                            last_dispatched_.position[1],
                                            last_dispatched_.position[2]);
                start_vel = Eigen::Vector3d(last_dispatched_.velocity[0],
                                            last_dispatched_.velocity[1],
                                            last_dispatched_.velocity[2]);
                start_acc = Eigen::Vector3d(last_dispatched_.acceleration[0],
                                            last_dispatched_.acceleration[1],
                                            last_dispatched_.acceleration[2]);
            } else {
                start_pos = Eigen::Vector3d(state_.p_n, state_.p_e, state_.p_d);
                start_vel = Eigen::Vector3d::Zero();
                start_acc = Eigen::Vector3d::Zero();
            }
        } else {
            // Plan from 200ms (10 setpoints) into the future, or the end of the deque if < 10
            stitch_index = std::min((int)setpoint_deque_.size() - 1, 10);
            
            const auto& sp = setpoint_deque_[stitch_index];
            start_pos = Eigen::Vector3d(sp.position[0], sp.position[1], sp.position[2]);
            start_vel = Eigen::Vector3d(sp.velocity[0], sp.velocity[1], sp.velocity[2]);
            start_acc = Eigen::Vector3d(sp.acceleration[0], sp.acceleration[1], sp.acceleration[2]);
        }

        double dx = start_pos.x() - end_pos.x();
        double dy = start_pos.y() - end_pos.y();
        double horizontal_dist = std::sqrt(dx*dx + dy*dy);

        // The first commanded waypoint typically starts on/near the ground, where SFC +
        // min-snap planning is not well-posed (undefined/degenerate corridors at ground
        // level). Navigate directly to it using simple path_manager-style quintic
        // interpolation instead. Subsequent waypoints use the full planner below.
        if (!first_waypoint_done_) {
            generate_direct_trajectory(start_pos, end_pos, state_.psi, current_wp_.psi, stitch_index,
                                       start_vel, start_acc);
            last_reached_goal_ = true;
            return;
        }

        // --- SHORT MOVES: interpolate, do not plan ---
        //
        // There is a dead band where nothing catches a short goal. Arrival is declared at 1.0 m
        // (3D, in dispatch_loop) while the vertical bypass below needs horizontal_dist < 0.5 m.
        // A vehicle sitting, say, 0.7 m horizontally and 1.0 m below a waypoint is 1.22 m away:
        // neither arrived nor bypassed. The full A*/SFC/QP pipeline then gets handed a sub-metre
        // goal and comes back over-constrained -- observed in flight as 2 corridors, the minimum
        // 10 control points, 198 constraints on 30 variables, with OSQP's infeasibility
        // certificate implicating 18/18 boundary rows. The buffer drains, the vehicle hovers,
        // the next replan repeats it, and it only escapes when drift carries it inside the
        // arrival radius -- which took 15-30 s in practice, and once did not recover at all.
        //
        // Below min_planning_distance there is nothing for the planner to buy anyway: obstacles
        // in the map already carry the drone-radius inflation, so sampling the straight line
        // against it is the same safety test the corridor would have encoded. If that line is
        // blocked, fall through and let the planner try.
        const double direct_dist = (end_pos - start_pos).norm();

        // A move is "mostly vertical" when its horizontal component is small RELATIVE to its
        // vertical one -- not when it is small in absolute terms. The absolute thresholds left a
        // gap: a vehicle 0.7 m horizontally and 2.5 m below a waypoint is 2.6 m away, so it
        // cleared min_planning_distance, while horizontal_dist = 0.7 cleared the < 0.5 m vertical
        // bypass below. A dominantly vertical move then went to A*/SFC/QP, which came back
        // primal-infeasible with all 18 boundary rows in the certificate.
        //
        // Corridor planning is the wrong tool for that geometry regardless of distance. The SFC
        // frame degenerates as the corridor axis approaches world up -- AsymmetricBoxBuilder
        // carries an explicit guard at |ux . up| > 0.99 for exactly this -- and a climb or
        // descent has no lateral obstacle-avoidance content for corridors to express. The
        // straight-line check below is the same safety test a corridor would encode, since map
        // obstacles already carry the drone-radius inflation; if it fails we fall through and
        // let the planner try.
        const double vertical_dist = std::abs(end_pos.z() - start_pos.z());
        const bool mostly_vertical =
            horizontal_dist < std::max(0.5, vertical_bypass_ratio_ * vertical_dist);

        if (direct_dist < min_planning_distance_ || mostly_vertical) {
            bool line_clear = true;
            if (shared_grid_) {
                const int samples = std::max(2, static_cast<int>(std::ceil(direct_dist / 0.25)));
                for (int i = 0; i <= samples; ++i) {
                    const Eigen::Vector3d p =
                        start_pos + (end_pos - start_pos) * (static_cast<double>(i) / samples);
                    if (shared_grid_->is_occupied_radius(p, kClearanceQueryRadius)) {
                        line_clear = false;
                        break;
                    }
                }
            }
            if (line_clear) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Goal %.2f m away (h=%.2f v=%.2f, %s): interpolating directly instead of planning.",
                    direct_dist, horizontal_dist, vertical_dist,
                    mostly_vertical ? "mostly vertical" : "short");
                generate_direct_trajectory(start_pos, end_pos, state_.psi, current_wp_.psi,
                                           stitch_index, start_vel, start_acc);
                last_reached_goal_ = true;
                // Commit the move -- EXCEPT while still on the ground.
                //
                // Committing means the quintic plays out open-loop for its full duration (see the
                // guard at the top of replan_loop), which is right for a short hop in the air but
                // wrong for the very first climb. A vehicle on the ground is thrust-limited: it
                // cannot follow a trajectory that starts moving immediately, because it has to
                // spool up and break contact first. The command advances on schedule regardless,
                // so by the time the vehicle actually lifts it is already a second or more behind
                // its own setpoint -- and then it has the authority to catch up fast, and sails
                // past. That is the takeoff overshoot: not gains, not windup, just a commitment
                // to a plan the vehicle could not start on time.
                //
                // Below takeoff_agl the move is re-planned from the ACTUAL position every cycle,
                // so the command can never run away from the vehicle. Once airborne it commits
                // normally and short hops keep the anti-stutter behaviour they need.
                const double agl = -state_.p_d;   // NED: negative down is up
                direct_move_active_ = (agl >= takeoff_agl_);
                return;
            }
        }

        // Bypass planner for pure vertical takeoffs to prevent collision errors with the ground
        if (horizontal_dist < 0.5) {
            // Simple vertical plan
            double dz = std::abs(end_pos.z() - start_pos.z());
            double v_max = std::max(this->get_parameter("v_max").as_double(), 0.1);
            // A quintic reaches 1.875x its average speed at the midpoint, so the duration is
            // scaled by that to keep the PEAK climb rate where the old constant-rate profile
            // put it. Without the scaling, smoothing the profile would make it climb faster.
            constexpr double kQuinticPeak = 1.875;
            double duration = kQuinticPeak * (3.0 * dz) / v_max;
            if (duration < 2.0) duration = 2.0;
            
            int num_points = std::ceil(duration / 0.02);
            
            // Clear old buffer past stitch point
            if (setpoint_deque_.size() > (size_t)stitch_index) {
                setpoint_deque_.erase(setpoint_deque_.begin() + stitch_index + 1, setpoint_deque_.end());
            setpoint_from_planner_.erase(setpoint_from_planner_.begin() + std::min<size_t>(stitch_index + 1, setpoint_from_planner_.size()), setpoint_from_planner_.end());
            }

            for (int i = 1; i <= num_points; ++i) {
                double t = duration * (double)i / num_points;
                double a = t / duration;

                // QUINTIC, not linear. The old profile commanded a CONSTANT climb velocity that
                // stepped to zero at the target, with zero acceleration feedforward throughout:
                // the vehicle was never asked to slow down, so it carried its momentum straight
                // past. Measured on takeoff -- commanded 3.0 m, reached 5.31 m, and the
                // controller then spent ~4 s pulling it back with thrust at 14 N against a
                // 34.3 N hover.
                //
                // s(a) = 10a^3 - 15a^4 + 6a^5 has s(0)=0, s(1)=1 and BOTH first and second
                // derivatives zero at each end, so the climb accelerates out of rest and
                // decelerates into the target, and the follower gets real velocity and
                // acceleration feedforward instead of a step.
                const double a2 = a * a, a3 = a2 * a, a4 = a3 * a, a5 = a4 * a;
                const double s_pos = 10.0 * a3 - 15.0 * a4 + 6.0 * a5;
                const double s_vel = (30.0 * a2 - 60.0 * a3 + 30.0 * a4) / duration;
                const double s_acc = (60.0 * a - 180.0 * a2 + 120.0 * a3) / (duration * duration);
                const double dzs = end_pos.z() - start_pos.z();

                roscopter_msgs::msg::TrajectoryCommand cmd;
                cmd.position[0] = start_pos.x();
                cmd.position[1] = start_pos.y();
                cmd.position[2] = start_pos.z() + dzs * s_pos;
                cmd.velocity[0] = 0; cmd.velocity[1] = 0; cmd.velocity[2] = dzs * s_vel;
                cmd.acceleration[0] = 0; cmd.acceleration[1] = 0; cmd.acceleration[2] = dzs * s_acc;
                cmd.psi = state_.psi;
                cmd.psi_dot = 0; cmd.psi_dot_dot = 0;
                setpoint_deque_.push_back(cmd);
                setpoint_from_planner_.push_back(0);
            }
            return;
        }

        // We re-instantiate planner to ensure it grabs the freshest map KD-tree from shared_grid_
        planner_ = std::make_unique<trajectory_planner::TrajectoryPlanner>(
            fe_config_, shared_grid_, 
            this->get_parameter("v_max").as_double(), 
            this->get_parameter("a_max").as_double()
        );

        // Hand back the route committed last cycle so A* prefers to stay on it. The planner is
        // rebuilt every replan and keeps no state, so this must be re-supplied each time.
        planner_->set_previous_path(previous_route_);

        // ROUTE-COMMIT-WITH-TRAJECTORY-REFRESH WAS TRIED HERE AND REVERTED -- measured far worse.
        //
        // The idea: hold the committed CORRIDOR CHAIN so A* cannot flip between homotopy classes,
        // but re-solve the QP every cycle from the current stitch state, so the vehicle always
        // flies a trajectory's fast opening rather than its decelerating tail. Corridors behind
        // the vehicle were trimmed each cycle; the chain was voided on age, goal change, or a new
        // obstacle inside the commitment window. It was clean offline: a vehicle walked along a
        // committed 3-corridor chain, re-solved at every metre, zero solve failures, and control
        // points fell 11 -> 9 -> 8 as corridors were trimmed.
        //
        // In flight it was by far the worst of the three designs. hard s0, v_max 8:
        //     no commitment        T_trav med 101 s [74-405]    v_avg 1.74
        //     hold the whole plan  T_trav med 135 s [82-146]    v_avg 1.69
        //     hold route, refresh  T_trav med 484 s [341-627]   v_avg 0.58, and one run with
        //                          18 collision events / 58 s in contact
        //
        // Two things to know before anyone tries it again:
        //
        //   A frozen chain lives in WORLD coordinates while the state estimate drifts ~1.5 m
        //   horizontally over the hold window. The map is built in that same drifting frame, so
        //   the held corridors stop corresponding to it -- the route goes geometrically stale in
        //   a way that re-solving the QP inside it cannot repair. Re-searching every cycle keeps
        //   route and map consistent in the estimate frame even while both drift, which is
        //   probably why the uncommitted arm tolerates the drift at all.
        //
        //   The ROUTE CHURN diagnostic becomes TAUTOLOGICAL under chain reuse. Consecutive plans
        //   share a chain by construction, so "route unchanged 98%, episodes 0" measured what was
        //   forced, not what was fixed. Do not read that block as evidence for this design.
        //
        // What survives is plan-level commitment, in the gate above.

        auto result = planner_->plan_mission(start_pos, end_pos, start_vel, start_acc);

        // Publish timing for EVERY replan, including failures. Failed cycles are the
        // expensive ones (they burn every time-stretch retry), so dropping them would
        // bias the tail statistics that actually matter for a 10 Hz loop.
        publish_computation_times(result);

        if (result.success) {
            // DEGENERATE-PLAN ESCAPE.
            //
            // A plan whose trajectory goes essentially NOWHERE, while the goal is still far
            // away, is an absorbing state: the vehicle does not move, so the next cycle rebuilds
            // the same plan, forever. Measured in flight on hard s0 at v_max 8 -- T_trav 1662 s
            // at 0.11 m/s, 14502 replans, route unchanged 99.53% of the time, and the planner
            // reporting result: true at 10 Hz throughout. Nothing else catches it: the QP solves
            // happily (start and end are both inside the corridors when they coincide), and the
            // arrival backstop only fires within fallback_radius of the waypoint, which at 25 m
            // out it correctly did not.
            //
            // The cause is the corridor chain terminating at the vehicle. Rather than try to
            // divine why the search returned so little, escalate once: re-plan this cycle with
            // the horizon projection lifted, so A* searches to the real goal instead of to a
            // sub-goal that has collapsed onto the start. Bounded by astar_max_nodes /
            // astar_max_time_ms, and only on a cycle that is already known to be useless.
            auto net_travel = [](const trajectory_planner::PlanningResult& r) {
                if (r.control_points.size() < 6) return 0.0;
                const int n = static_cast<int>(r.control_points.size()) / 3;
                return (r.control_points.segment<3>(3 * (n - 1))
                        - r.control_points.segment<3>(0)).norm();
            };
            const double goal_dist = (end_pos - start_pos).norm();
            if (goal_dist > kDegenerateGoalFar && net_travel(result) < kDegenerateNetTravel) {
                ++degenerate_plans_;
                auto escaped_cfg = fe_config_;
                // Extend the horizon rather than removing it. Setting it to -1 drops the goal
                // projection AND collapses the geofence pad to its 60 m fallback, so a goal
                // beyond that is outside the search bounds: A* then returns a PARTIAL path whose
                // endpoint sits wherever the frontier happened to reach, which is not a
                // direction anyone chose.
                escaped_cfg.planning_horizon =
                    std::min(goal_dist, 3.0 * std::max(1.0, fe_config_.planning_horizon));
                trajectory_planner::TrajectoryPlanner escape(
                    escaped_cfg, shared_grid_,
                    this->get_parameter("v_max").as_double(),
                    this->get_parameter("a_max").as_double());
                auto alt = this->get_parameter("degenerate_escape").as_bool()
                    ? escape.plan_mission(start_pos, end_pos, start_vel, start_acc)
                    : result;
                if (alt.success && net_travel(alt) > net_travel(result)) {
                    ++degenerate_escapes_;
                    result = std::move(alt);
                }
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Plan went nowhere (%.2f m of travel, goal %.1f m away). Escalated past the "
                    "planning horizon; %zu such plans, %zu escapes so far.",
                    net_travel(result), goal_dist, degenerate_plans_, degenerate_escapes_);
            }

            // DIRECTIONAL GATE. Decline a candidate that would send the vehicle somewhere
            // meaningfully different right now, and keep flying what we have. Evaluated before
            // anything is applied, so a rejection leaves setpoint_deque_, previous_route_ and
            // last_plan_time_ untouched -- the current plan simply continues.
            std::vector<Eigen::Vector3d> candidate_route;
            candidate_route.reserve(result.corridors.size() + 1);
            for (const auto& c : result.corridors) candidate_route.push_back(c.primaryPosition);
            if (!result.corridors.empty()) {
                candidate_route.push_back(result.corridors.back().secondaryPosition);
            }

            // Overrides. The collision term is the important one: if the trajectory being flown
            // is now blocked, the replan that avoids the obstacle is exactly the one a
            // directional filter would otherwise throw away.
            bool force_accept = previous_route_.size() < 2 || setpoint_deque_.size() < 10;
            if (!force_accept) {
                const Eigen::Vector3d goal_now(current_wp_.w[0], current_wp_.w[1],
                                               current_wp_.w[2]);
                if ((goal_now - committed_goal_).norm() > 1e-6) force_accept = true;
            }
            if (!force_accept) {
                // BOUNDED to what the vehicle can actually fly before the gate must yield.
                //
                // Sweeping the WHOLE buffer tests setpoints several seconds ahead, in space the
                // LiDAR has not reached yet, so the far end goes "occupied" as the map fills in
                // and forces an accept for a hazard the vehicle would never have reached. That
                // fired on 37-52% of cycles across the first three gated runs, leaving the gate
                // governing only about half of them.
                //
                // The gate can hold a route for at most max_consecutive_rejects replan cycles
                // before the bound force-accepts anyway, so anything past that window is a
                // decision the NEXT replan gets to make. Add braking time (v_max / a_max) so an
                // obstacle appearing at the edge of the window is still stoppable, and floor it
                // so the window never collapses at low speed.
                const double reject_window_s =
                    this->get_parameter("route_reject_max_consecutive").as_int() / kReplanHz;
                const double brake_s = this->get_parameter("v_max").as_double()
                                     / std::max(1e-3, this->get_parameter("a_max").as_double());
                const double horizon_s = std::max(1.5, reject_window_s + brake_s);
                const std::size_t horizon = std::min(
                    setpoint_deque_.size(),
                    static_cast<std::size_t>(horizon_s / kDispatchDt));
                for (std::size_t i = 0; i < horizon; i += 10) {
                    const Eigen::Vector3d pt(setpoint_deque_[i].position[0],
                                             setpoint_deque_[i].position[1],
                                             setpoint_deque_[i].position[2]);
                    if (shared_grid_->is_occupied_radius(pt, kClearanceQueryRadius)) {
                        force_accept = true;
                        break;
                    }
                }
            }

            const auto gate = route_gate_.evaluate(previous_route_, candidate_route,
                                                   start_pos, end_pos, force_accept);
            if (!gate.accept) {
                RCLCPP_DEBUG(this->get_logger(),
                             "Declined a replan diverging %.1f deg from the route being flown.",
                             gate.angle_deg);
                return;
            }

            // Extract the trajectory and knots
            Eigen::VectorXd active_trajectory = result.control_points;
            Eigen::VectorXd active_knots = result.knots;
            int active_degree = result.degree;
            
            Eigen::VectorXd active_vel_cps = get_derivative_cps(active_trajectory, active_knots, active_degree);
            Eigen::VectorXd active_vel_knots = get_derivative_knots(active_knots);
            Eigen::VectorXd active_accel_cps = get_derivative_cps(active_vel_cps, active_vel_knots, active_degree - 1);
            Eigen::VectorXd active_accel_knots = get_derivative_knots(active_vel_knots);
            
            double t_start = active_knots(active_degree);
            double t_end = active_knots(active_knots.size() - active_degree - 1);
            
            // Generate setpoints at 50Hz (0.02s)
            int new_setpoints_needed = std::ceil((t_end - t_start) / 0.02);
            
            // Only keep up to stitch_index from the current deque
            if (setpoint_deque_.size() > (size_t)stitch_index) {
                setpoint_deque_.erase(setpoint_deque_.begin() + stitch_index + 1, setpoint_deque_.end());
            setpoint_from_planner_.erase(setpoint_from_planner_.begin() + std::min<size_t>(stitch_index + 1, setpoint_from_planner_.size()), setpoint_from_planner_.end());
            }
            
            for (int i = 1; i <= new_setpoints_needed; ++i) {
                double t = t_start + i * 0.02;
                if (t > t_end) t = t_end;
                
                Eigen::Vector3d pos = evaluate_bspline(t, active_trajectory, active_knots, active_degree);
                Eigen::Vector3d vel = evaluate_bspline(t, active_vel_cps, active_vel_knots, active_degree - 1);
                Eigen::Vector3d accel = evaluate_bspline(t, active_accel_cps, active_accel_knots, active_degree - 2);
                
                roscopter_msgs::msg::TrajectoryCommand cmd;
                cmd.position[0] = pos.x(); cmd.position[1] = pos.y(); cmd.position[2] = pos.z();
                cmd.velocity[0] = vel.x(); cmd.velocity[1] = vel.y(); cmd.velocity[2] = vel.z();
                cmd.acceleration[0] = accel.x(); cmd.acceleration[1] = accel.y(); cmd.acceleration[2] = accel.z();
                
                double speed = std::sqrt(vel.x()*vel.x() + vel.y()*vel.y());
                if (speed > 0.1) {
                    cmd.psi = std::atan2(vel.y(), vel.x());
                } else {
                    cmd.psi = state_.psi;
                }
                cmd.psi_dot = 0; cmd.psi_dot_dot = 0;
                setpoint_deque_.push_back(cmd);
                setpoint_from_planner_.push_back(1);
            }

            append_braking_tail();

            // Publish SFC visualization
            visualization_msgs::msg::MarkerArray sfc_markers;

            // Wipe the previous cycle's corridors first. Markers are keyed by id and persist
            // until something overwrites or deletes them, so a replan that produces FEWER
            // corridors than the last one leaves the surplus ids on screen indefinitely --
            // which is why stale corridors kept hanging around behind the vehicle.
            sfc_markers.markers.push_back(make_delete_all_marker());

            int id = 0;
            for (const auto& sfc : result.corridors) {
                visualization_msgs::msg::Marker marker;
                marker.header.stamp = this->now();
                marker.header.frame_id = "NED"; // RViz fixed frame
                marker.ns = "sfc";
                marker.id = id++;
                marker.type = visualization_msgs::msg::Marker::LINE_LIST;
                marker.action = visualization_msgs::msg::Marker::ADD;
                // Backstop: if the node dies or a publish is missed, these expire on their
                // own rather than freezing on screen. Comfortably longer than the 100 ms
                // replan period so they never flicker during normal operation.
                marker.lifetime = rclcpp::Duration::from_seconds(0.5);
                // 0.15 m, up from 0.05. For a LINE_LIST only scale.x is read, and it is a
                // WORLD-SPACE width -- at 0.05 m the corridor edges vanish once the whole 105 m
                // course is in frame, which is the view a paper figure uses.
                marker.scale.x = 0.15; // line width
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 0.5;

                auto vertices = sfc.getAllVertices_3D();
                static const int edges[12][2] = {
                    {0,1}, {0,3}, {0,4}, {1,2}, {1,5}, {2,3}, {2,6}, {3,7}, {4,5}, {4,7}, {5,6}, {6,7}
                };
                for (int e = 0; e < 12; ++e) {
                    geometry_msgs::msg::Point p1, p2;
                    p1.x = vertices(0, edges[e][0]);
                    p1.y = vertices(1, edges[e][0]);
                    p1.z = vertices(2, edges[e][0]);
                    p2.x = vertices(0, edges[e][1]);
                    p2.y = vertices(1, edges[e][1]);
                    p2.z = vertices(2, edges[e][1]);
                    marker.points.push_back(p1);
                    marker.points.push_back(p2);
                }
                sfc_markers.markers.push_back(marker);
            }
            sfc_pub_->publish(sfc_markers);
            
            // Publish Path visualization
            nav_msgs::msg::Path traj_path;
            traj_path.header.stamp = this->now();
            traj_path.header.frame_id = "NED";
            for (int i = 0; i <= 100; ++i) {
                double t = t_start + (t_end - t_start) * (double)i / 100.0;
                Eigen::Vector3d pt = evaluate_bspline(t, active_trajectory, active_knots, active_degree);
                geometry_msgs::msg::PoseStamped pose;
                pose.header = traj_path.header;
                pose.pose.position.x = pt.x();
                pose.pose.position.y = pt.y();
                pose.pose.position.z = pt.z();
                traj_path.poses.push_back(pose);
            }
            path_pub_->publish(traj_path);

            last_reached_goal_ = result.reached_goal;
            last_plan_time_ = this->now().seconds();
            consecutive_replan_failures_ = 0;

            // Remember this route for next cycle's hysteresis. The corridor chain is the
            // smoothed path: each corridor spans one segment, so the primaries plus the final
            // secondary reproduce it exactly. Only stored on SUCCESS -- a route that failed to
            // optimise is not one worth biasing the next search toward.
            std::vector<Eigen::Vector3d> route;
            route.reserve(result.corridors.size() + 1);
            for (const auto& c : result.corridors) {
                route.push_back(c.primaryPosition);
            }
            if (!result.corridors.empty()) {
                route.push_back(result.corridors.back().secondaryPosition);
            }
            // How far ahead this route departs from the previous one. Diagnostic only -- it
            // decides whether committing to a route would help at all. See RouteChurn.
            if (metrics_active()) route_churn_.add(route);
            previous_route_ = route;
            // The goal this plan was optimised toward; a committed plan is void if it moves.
            committed_goal_ = end_pos;
            ++commit_stats_replans_;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Replanning failed.");
            // Nothing valid to show. Leaving the previous overlay up would misrepresent a
            // stale plan as the current one.
            clear_visualization();

            // Sustained failure means the planner does not know where to go. Stop flying the
            // stale trajectory.
            //
            // A SINGLE missed replan is cheap and must stay that way -- the committed trajectory
            // keeps flying and the next cycle retries against a fresher map, which is what stops
            // the vehicle stuttering. But nothing previously distinguished one miss from a
            // hundred, so a vehicle whose planner had given up kept flying its last plan at
            // speed, straight at whatever the planner had just failed to route around.
            //
            // That is a cascade, not a one-off: it flies into the obstacle it could not plan
            // around, ends up with its own position inside inflated space, and from there A*
            // starts on a blocked cell and EVERY subsequent replan fails too. Observed exactly
            // that -- "start idx [102 -1 -9] (INFLATED-BLOCKED)" after a 0.003 m cylinder graze
            // at ~4 m/s, followed by permanent stall in visibly open air.
            //
            // Braking short of the obstruction keeps the start free, which is what lets the next
            // replan actually consider the routes to either side.
            ++consecutive_replan_failures_;
            if (consecutive_replan_failures_ == replan_failures_before_brake_
                && setpoint_deque_.size() > kBrakeReactionSetpoints) {
                const std::size_t keep = kBrakeReactionSetpoints;
                setpoint_deque_.erase(setpoint_deque_.begin() + keep, setpoint_deque_.end());
                // The generator tags are indexed in lockstep with the setpoints; truncating one
                // without the other silently mis-attributes every later sample.
                if (setpoint_from_planner_.size() > keep) {
                    setpoint_from_planner_.erase(
                        setpoint_from_planner_.begin() + keep, setpoint_from_planner_.end());
                }
                const std::size_t before = setpoint_deque_.size();
                append_braking_tail();
                RCLCPP_WARN(this->get_logger(),
                            "%d consecutive replan failures -- braking to a stop instead of "
                            "flying the stale trajectory (kept %zu setpoints + %zu braking).",
                            consecutive_replan_failures_, keep,
                            setpoint_deque_.size() - before);
            }
        }
    }

    /**
     * Has the approach to the active waypoint stalled short of arrival_radius_?
     *
     * The decision itself lives in navigation/arrival_monitor.hpp, free of ROS types so its
     * must-fire / must-not-fire scenarios are covered on the bench by test/arrival_monitor_test.cpp
     * against that exact code. This wrapper only supplies the vehicle state and does the logging.
     */
    bool arrival_stalled(double dist) {
        const double now = this->now().seconds();
        const double speed = std::sqrt(state_.v_x * state_.v_x + state_.v_y * state_.v_y
                                       + state_.v_z * state_.v_z);
        const bool active = metrics_active();
        if (speed >= arrival_.stall_speed) {
            stall_since_ = 0.0;
            // A new episode deserves its own snapshot. Without this the latch would arm once
            // and every later stall in the same run would go unrecorded.
            stall_reported_ = false;
        }

        // Beyond the ceiling nothing will ever be skipped, so a vehicle that has genuinely given
        // up out here would hover silently until the battery decides the mission. Say so; it is
        // the same evidence the fallback would have logged, minus the action.
        if (active && !arrival_.disabled() && dist >= arrival_.fallback_radius
            && speed < arrival_.stall_speed) {
            // Start the clock the first time we notice, and reset it whenever the vehicle is
            // making way again -- see stall_timeout.
            if (stall_since_ <= 0.0) stall_since_ = this->now().seconds();
            const double stall_timeout = this->get_parameter("stall_timeout").as_double();
            const double stalled_for = this->now().seconds() - stall_since_;
            // Flag only. This is a predicate; ending the mission is the caller's job -- see the
            // mission_stalled_ guard at the top of replan_loop.
            // DETECTION AND TERMINATION ARE SEPARATE.
            //
            // They used to be one branch, so switching the timeout off to stop losing runs
            // also switched off the stall SNAPSHOT -- the only artefact that makes a stall
            // debuggable offline. That is exactly backwards: a run you are willing to sit
            // through is a run you most want evidence from.
            //
            // stall_timeout now only says WHEN a stall is declared. stall_ends_run says
            // whether declaring one stops the mission. The snapshot is written either way,
            // once per stall episode.
            if (stall_timeout > 0.0 && stalled_for > stall_timeout && !stall_reported_) {
                stall_reported_ = true;
                write_stall_snapshot(dist);
                if (this->get_parameter("stall_ends_run").as_bool()) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "MISSION STALLED: %.2f m from the waypoint for %.0f s with the "
                                 "planner reporting the goal unreachable. Ending the run as a "
                                 "FAILURE rather than waiting indefinitely. Set stall_ends_run "
                                 "false to keep flying, or stall_timeout 0 to disable entirely.",
                                 dist, stalled_for);
                    mission_stalled_ = true;
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "STALL DECLARED: %.2f m from the waypoint for %.0f s. Snapshot "
                                "written; CONTINUING because stall_ends_run is false. Stop the "
                                "run by hand if it does not recover.",
                                dist, stalled_for);
                }
            }
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Stopped %.2f m from the waypoint, beyond the %.2f m fallback ceiling "
                "(planner reports goal %s). Not skipping; the mission will wait.",
                dist, arrival_.fallback_radius,
                last_reached_goal_ ? "reachable" : "UNREACHABLE");
        }

        return arrival_.update(dist, speed, active, now);
    }

    /** Rearm the stalled-progress detector for a newly activated waypoint. */
    void reset_arrival_tracking() { arrival_.reset(this->now().seconds()); }

    /**
     * Watch the dispatch loop's own publish rate and complain when it is being starved.
     *
     * The node spins single-threaded, so the 10 Hz replan timer and this 50 Hz timer share one
     * thread: any replan that overruns its slice directly delays setpoint publishing. That
     * failure is invisible from inside the planner -- it keeps producing good corridors and good
     * trajectories, and every log line looks healthy -- while the vehicle is actually being flown
     * open-loop between sparse setpoints. It cost a full benchmark run to spot, and only because
     * an external tool happened to measure the received rate (15.4 Hz against 50 published, with
     * control error 10x its usual value and along-track lag near zero: off the path, not behind
     * it). Report it here so the next occurrence names itself.
     */
    void check_dispatch_rate() {
        const double now = this->now().seconds();
        if (last_dispatch_stamp_ > 0.0) {
            const double gap = now - last_dispatch_stamp_;
            // Ignore clock jumps and the first tick after a pause; they are not starvation.
            if (gap > 0.0 && gap < 5.0) {
                dispatch_gap_sum_ += gap;
                ++dispatch_gap_n_;
                if (gap > 1.5 * kDispatchDt) ++dispatch_late_n_;
            }
        }
        last_dispatch_stamp_ = now;

        // Report on a fixed sample count rather than a timer, so the check itself cannot be
        // starved by the very thing it is measuring.
        if (dispatch_gap_n_ >= 250) {
            const double mean_hz = dispatch_gap_sum_ > 0.0
                ? static_cast<double>(dispatch_gap_n_) / dispatch_gap_sum_ : 0.0;
            const double late_pct =
                100.0 * static_cast<double>(dispatch_late_n_) / dispatch_gap_n_;
            if (mean_hz < 0.8 / kDispatchDt) {
                RCLCPP_WARN(this->get_logger(),
                            "Dispatch starved: publishing at %.1f Hz against a nominal %.0f Hz "
                            "(%.0f%% of ticks late). The replan loop is overrunning its slice on "
                            "a single-threaded executor -- lower planning_horizon / "
                            "astar_max_nodes / max_solve_time_ms. Tracking error measured now is "
                            "not the controller's fault.",
                            mean_hz, 1.0 / kDispatchDt, late_pct);
            }
            dispatch_gap_sum_ = 0.0;
            dispatch_gap_n_ = 0;
            dispatch_late_n_ = 0;
        }
    }

    void dispatch_loop() {
        check_dispatch_rate();
        if (!has_state_) return;

        // Don't publish anything (and thus don't let downstream controller treat the
        // command as valid/trigger auto-takeoff) until a real mission has been commanded.
        if (!mission_started_) return;

        // Check if we arrived at active waypoint and should hold/complete
        if (active_waypoint_) {
            Eigen::Vector3d current_pos(state_.p_n, state_.p_e, state_.p_d);
            Eigen::Vector3d target_pos(current_wp_.w[0], current_wp_.w[1], current_wp_.w[2]);
            const double dist = (current_pos - target_pos).norm();
            // The FIRST waypoint additionally requires its altitude to be reached, not merely
            // a spherical radius.
            //
            // A 1.0 m sphere around a waypoint 3 m overhead is satisfied at 2 m altitude, so the
            // mission advanced to the far waypoint while the vehicle was still climbing -- and it
            // then flew the climb and the 105 m traverse simultaneously. That is the "hybrid"
            // takeoff: not a controller fault, just the mission moving on before the climb was
            // finished. Vertical tracking is worst exactly there, because the commanded
            // trajectory is turning horizontal underneath a vehicle that has not arrived yet.
            //
            // Applies only until the first waypoint is done. Later waypoints are flown through,
            // where a spherical radius is the right test and a tighter one would just stall.
            // Position AND settled. Testing position alone lets a vehicle climbing THROUGH the
            // target satisfy the gate for an instant, which is the hybrid takeoff this check
            // exists to prevent -- see takeoff_settle_speed.
            // AT waypoint altitude AND settled. Position alone lets a vehicle climbing THROUGH
            // the target satisfy this for one instant; vertical speed alone lets it depart still
            // carrying forward velocity. Both matter -- see takeoff_settle_speed.
            const double speed = std::sqrt(
                static_cast<double>(state_.v_x) * state_.v_x +
                static_cast<double>(state_.v_y) * state_.v_y +
                static_cast<double>(state_.v_z) * state_.v_z);
            const bool at_altitude =
                std::abs(current_pos.z() - target_pos.z()) < takeoff_altitude_tol_;
            const bool settled =
                takeoff_settle_speed_ <= 0.0 || speed < takeoff_settle_speed_;

            // Bound the wait. A vehicle that never settles would otherwise hang the mission --
            // the arrival backstop only fires within fallback_radius, and an overshoot exceeds it.
            // The clock starts when the vehicle is AIRBORNE, not when it is sitting on the
            // ground waiting to arm. Timing from the ground burns the whole budget before the
            // climb begins: the vehicle rests 3 m below a -3 m target, so `at_altitude` is false
            // from t = 0, the timeout expires ~8 s later just as it is climbing through, and the
            // waypoint is force-accepted mid-transient -- exactly what the gate exists to stop.
            // Measured: traverse began at t = 10.5 s with the vehicle at -4.32 m.
            const double agl = -current_pos.z();
            if (!first_waypoint_done_ && agl > takeoff_agl_ && !(at_altitude && settled)) {
                if (settle_wait_start_ <= 0.0) settle_wait_start_ = this->now().seconds();
            } else if (at_altitude && settled) {
                settle_wait_start_ = 0.0;
            }
            const bool settle_timed_out =
                settle_wait_start_ > 0.0 && takeoff_settle_timeout_ > 0.0 &&
                (this->now().seconds() - settle_wait_start_) > takeoff_settle_timeout_;
            if (settle_timed_out) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Proceeding past waypoint 1 unsettled after %.1f s: %.2f m off altitude at "
                    "%.2f m/s. The traverse will be planned from a state the mission did not ask "
                    "for.",
                    takeoff_settle_timeout_,
                    std::abs(current_pos.z() - target_pos.z()), speed);
            }

            const bool altitude_reached =
                first_waypoint_done_ || settle_timed_out || (at_altitude && settled);
            const bool arrived_nominally = dist < arrival_radius_ && altitude_reached;
            const bool arrived_by_stall = !arrived_nominally && arrival_stalled(dist);
            if (arrived_nominally || arrived_by_stall) {
                if (arrived_by_stall) {
                    // reached_goal is the discriminator between the two causes and is the first
                    // thing to look at post-flight: false means the front end could not route to
                    // the goal (obstacle, or one phantom-displaced by estimator drift between
                    // mapping and traversal); true means it planned there and the vehicle still
                    // did not close the gap (tracking, saturation, wind).
                    RCLCPP_WARN(this->get_logger(),
                                "Waypoint %zu accepted on stalled progress: stopped %.2f m away "
                                "(nominal radius %.2f m) with no %.2f m gain for %.1f s. Planner "
                                "reported the goal %s.",
                                leg_stats_.size() + 1, dist, arrival_radius_,
                                arrival_.progress_epsilon, arrival_.stall_time,
                                last_reached_goal_ ? "REACHABLE (tracking-side stall)"
                                                   : "UNREACHABLE (planner-side stall)");
                }
                arrival_was_fallback_ = arrived_by_stall;
                active_waypoint_ = false;
                // Start the hold. The empty-buffer path in this loop already publishes a hover
                // at the last commanded position, so holding needs nothing else -- just a gate
                // on pulling the next waypoint.
                const double hold_s = static_cast<double>(current_wp_.hold_seconds);
                if (hold_s > 0.0) {
                    hold_until_ = this->now().seconds() + hold_s;
                    RCLCPP_INFO(this->get_logger(),
                                "Holding %.1f s at waypoint %zu before continuing.",
                                hold_s, leg_stats_.size() + 1);
                }
                first_waypoint_done_ = true;

                // Record this leg and, if the mission is now finished, print the table.
                const double leg_wall = (leg_wall_start_ >= 0.0)
                    ? std::max(0.0, this->now().seconds() - leg_wall_start_) : 0.0;
                leg_wall_start_ = this->now().seconds();   // next leg starts now
                if (plan_time_stats_.count() > 0) {
                    leg_stats_.push_back({static_cast<int>(leg_stats_.size()) + 1,
                                          plan_time_stats_.count(),
                                          plan_time_stats_.min(),
                                          plan_time_stats_.mean(),
                                          plan_time_stats_.max(),
                                          worst_,
                                          leg_quality_,
                                          leg_planned_quality_,
                                          arrival_was_fallback_,
                                          dist,
                                          last_reached_goal_,
                                          leg_wall,
                                          std::max(0.0, leg_wall - leg_quality_.duration)});
                    plan_time_stats_.reset();   // per-leg, so legs stay comparable
                    leg_quality_.reset();
                    leg_planned_quality_.reset();
                    worst_ = WorstCycle{};
                }
                if (wp_queue_.empty()) print_planning_time_table();
            }
        }

        // Do not fly the trajectory while the autopilot is not in control.
        //
        // mission_started_ goes true when the waypoint MESSAGE arrives, which is not when the
        // vehicle starts flying -- the mission is loaded, then the pilot arms and hands over.
        // Consuming the buffer in between drains the entire takeoff trajectory into a disarmed
        // vehicle sitting on the ground, and the hover fallback then parks the command at the
        // trajectory's END. Measured from /tmp/track.csv: cmd_d pinned at exactly -3.000 m for
        // the whole 6.7 s pre-arm window while truth_d stayed at 0.000. At handover the vehicle
        // therefore sees a 3 m STEP, not a ramp, and climbs at full authority -- which is the
        // takeoff overshoot. It is not gains, windup, or authority asymmetry; the trajectory was
        // simply already over before the vehicle was allowed to move.
        //
        // Holding here keeps the plan intact until control is actually handed over. The hover is
        // published at the CURRENT position so there is no step when it is.
        if (!metrics_active()) {
            roscopter_msgs::msg::TrajectoryCommand hold;
            hold.header.stamp = this->now();
            hold.position[0] = state_.p_n;
            hold.position[1] = state_.p_e;
            hold.position[2] = state_.p_d;
            hold.psi = state_.psi;
            traj_cmd_pub_->publish(hold);
            // Keep the difference stencils open; the next real sample must not be differenced
            // against one from before the hold.
            splice_armed_ = true;
            splice_countdown_ = 0;
            last_source_ = -1;
            last_publish_stamp_ = this->now().seconds();
            return;
        }

        if (setpoint_deque_.empty()) {
            // Hover command
            roscopter_msgs::msg::TrajectoryCommand cmd;
            cmd.header.stamp = this->now();
            cmd.position[0] = init_x_;
            cmd.position[1] = init_y_;
            cmd.position[2] = init_z_; // Hold stable last-commanded/initial position until a waypoint arrives
            cmd.velocity[0] = 0.0; cmd.velocity[1] = 0.0; cmd.velocity[2] = 0.0;
            cmd.acceleration[0] = 0.0; cmd.acceleration[1] = 0.0; cmd.acceleration[2] = 0.0;
            cmd.psi = state_.psi;
            cmd.psi_dot = 0.0; cmd.psi_dot_dot = 0.0;
            traj_cmd_pub_->publish(cmd);
            return;
        }

        // Fire exactly once, on the pop that straddles the join with the newest plan, then
        // disarm. Testing (countdown == 0) alone latches: the counter bottoms out and stays
        // there through every skipped replan, so every later sample looks like a splice and the
        // jerk statistic collapses to zero samples.
        at_splice_ = false;
        if (splice_armed_) {
            if (splice_countdown_ == 0) { at_splice_ = true; splice_armed_ = false; }
            else --splice_countdown_;
        }

        // Advance the buffer by WALL TIME, not by one sample per call.
        //
        // Setpoints are sampled kDispatchDt apart in TRAJECTORY time. Popping exactly one per
        // callback only replays the trajectory at real speed while the timer actually achieves
        // its nominal rate -- and when the callback is late, the trajectory silently plays in
        // slow motion at exactly the ratio of achieved to nominal rate. That is not a tracking
        // nuisance, it is a speed bug: measured at 10.1 Hz against a nominal 50, the commanded
        // trajectory advanced at ONE FIFTH real time, so a plan sized for 1.75 m/s was flown at
        // roughly 0.35 m/s. It reads exactly like the vehicle stalling in open space, which is
        // what sent several investigations after the planner while the planner was fine.
        //
        // Consuming by elapsed time decouples commanded SPEED from callback rate: a late
        // callback publishes the setpoint the trajectory should be at now, having skipped the
        // ones whose moment has passed. Fewer, correctly-timed setpoints degrade tracking
        // smoothness; wrongly-timed ones degrade the mission.
        //
        // The skipped samples are still fed to the metrics below -- they are part of the
        // trajectory that was committed, and dropping them would understate duration and
        // distance. Capped so that one long stall cannot teleport the command a long way; past
        // the cap the trajectory does fall behind, which is the safer failure.
        std::size_t to_consume = 1;
        if (last_publish_stamp_ > 0.0) {
            const double elapsed = this->now().seconds() - last_publish_stamp_;
            if (elapsed > 0.0) {
                to_consume = static_cast<std::size_t>(std::lround(elapsed / kDispatchDt));
            }
        }
        to_consume = std::min<std::size_t>(std::max<std::size_t>(to_consume, 1), kMaxCatchUp);
        to_consume = std::min(to_consume, setpoint_deque_.size());
        last_publish_stamp_ = this->now().seconds();

        // Everything consumed but the last is accounted for and discarded; the last is flown.
        for (std::size_t skip = 0; skip + 1 < to_consume; ++skip) {
            record_dispatched(setpoint_deque_.front());
            setpoint_deque_.pop_front();
        }

        auto cmd = setpoint_deque_.front();
        setpoint_deque_.pop_front();
        cmd.header.stamp = this->now();
        traj_cmd_pub_->publish(cmd);

        record_dispatched(cmd);
    }

    /**
     * Book-keeping for one setpoint that has been consumed from the buffer.
     *
     * Called for the sample actually published AND for any the catch-up skipped: both were part
     * of the committed trajectory, so leaving the skipped ones out would understate duration and
     * path length and quietly flatter every quality metric. Measurement only -- it must never
     * gate publishing, which is why the caller publishes first.
     */
    void record_dispatched(const roscopter_msgs::msg::TrajectoryCommand& cmd) {
        // Kept so a replan with an empty buffer can continue from the last COMMAND rather than
        // teleporting to the vehicle -- see the anchor in replan_loop.
        last_dispatched_ = cmd;
        have_last_dispatched_ = true;
        // The per-setpoint generator tag has to be consumed in lockstep with the setpoint even
        // when metrics are off, or the two deques drift apart and every later sample is
        // attributed to the wrong generator.
        int source = 0;
        if (!setpoint_from_planner_.empty()) {
            source = setpoint_from_planner_.front();
            setpoint_from_planner_.pop_front();
        }

        if (!metrics_active()) {
            // Hold the difference stencils open so the first sample after control resumes is not
            // differenced against one from before the gap.
            splice_armed_ = true;
            splice_countdown_ = 0;
            last_source_ = -1;
            return;
        }

        // Start the leg's wall clock on the first setpoint that counts. Anchoring it here
        // rather than at waypoint receipt keeps pre-arm idling out of T_wall.
        if (leg_wall_start_ < 0.0) { leg_wall_start_ = this->now().seconds(); }

        const Eigen::Vector3d q_p(cmd.position[0], cmd.position[1], cmd.position[2]);
        const Eigen::Vector3d q_v(cmd.velocity[0], cmd.velocity[1], cmd.velocity[2]);
        const Eigen::Vector3d q_a(cmd.acceleration[0], cmd.acceleration[1], cmd.acceleration[2]);
        leg_quality_.add(q_p, q_v, q_a, kDispatchDt, at_splice_);

        // A change of generator is itself a discontinuity -- the quintic commands zero
        // acceleration while the spline does not -- so the first sample after a switch must not
        // be differenced against the previous sample of that bucket, which may be seconds old.
        const bool bucket_splice = at_splice_ || (source != last_source_);
        last_source_ = source;
        (source ? planned_quality_ : direct_quality_).add(q_p, q_v, q_a, kDispatchDt, bucket_splice);
        if (source) leg_planned_quality_.add(q_p, q_v, q_a, kDispatchDt, bucket_splice);
        at_splice_ = false;   // a splice applies to one sample, not to a whole catch-up burst

        // Update hover position to last commanded position so we don't snap back when empty
        init_x_ = cmd.position[0];
        init_y_ = cmd.position[1];
        init_z_ = cmd.position[2];
    }

    rclcpp::Subscription<roscopter_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<roscopter_msgs::msg::Waypoint>::SharedPtr wp_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    
    rclcpp::Publisher<roscopter_msgs::msg::TrajectoryCommand>::SharedPtr traj_cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sfc_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<roscopter_msgs::msg::ComputationTimes>::SharedPtr comp_times_pub_;

    rclcpp::TimerBase::SharedPtr dispatch_timer_;
    rclcpp::TimerBase::SharedPtr replan_timer_;

    roscopter_msgs::msg::State state_;
    bool has_state_;
    
    std::shared_ptr<trajectory_planner::mapping::SparseVoxelGrid> shared_grid_;
    std::unique_ptr<trajectory_planner::TrajectoryPlanner> planner_;
    trajectory_planner::FrontEndConfig fe_config_;

    std::vector<roscopter_msgs::msg::Waypoint> wp_queue_;
    std::deque<roscopter_msgs::msg::TrajectoryCommand> setpoint_deque_;
    roscopter_msgs::msg::Waypoint current_wp_;
    bool active_waypoint_;
    bool mission_started_;
    bool first_waypoint_done_;

    bool initial_pos_set_;
    double init_x_ = 0.0;
    double init_y_ = 0.0;
    double init_z_ = 0.0;
    bool grid_initialized_;
    
    trajectory_planner::metrics::RunningStats plan_time_stats_;
    /// OSQP stage alone, accumulated the same way as the total. The message carries opt_time_ms
    /// per replan but nothing summarised it, so the run summary could report an end-to-end replan
    /// time and only the WORST cycle's QP time -- not a mean QP time. That is the number a paper
    /// needs when it claims a solve-time result, so it is accumulated here.
    trajectory_planner::metrics::RunningStats opt_time_stats_;
    std::vector<double> opt_time_all_;   ///< for percentiles
    /// PLAN SHAPE, per replan. The offline harness reports median n_cp 9-10 with ~50% of plans
    /// single-corridor, and a plan-mean speed of 2.6 m/s on these same forest files; flight
    /// measures v_avg 1.13 m/s in TRAJECTORY time on the same maps. Dispatch timing is ruled out
    /// (kMaxCatchUp = 25 covers the observed 20.5 Hz, and T_trav tracks T_wall), so the gap has to
    /// be in the plans themselves -- and duration IS control-point count at unit knots. If these
    /// medians come out well above the harness's, the cause is map quality driving corridor count:
    /// every junction costs a `bridge` pool of `degree` points.
    std::vector<double> plan_cps_, plan_sfc_, plan_dur_speed_;
    /// Wall-clock at which the current leg became active; <0 means no leg in progress.
    /// See LegTiming::wall_s for why trajectory time cannot substitute for this.
    double leg_wall_start_ = -1.0;
    std::vector<LegTiming> leg_stats_;
    FlownQuality leg_quality_;
    FlownQuality leg_planned_quality_;
    // Mission-wide quality split by WHICH GENERATOR produced each setpoint, not by leg.
    // Attribution has to be per-setpoint: the non-spline generators are not confined to leg 1
    // any more -- min_planning_distance routes short goals to the quintic mid-mission, and the
    // braking tail appends constant-deceleration samples after any plan. Splitting by leg would
    // quietly mix them back together.
    FlownQuality planned_quality_;   ///< de Boor evaluation of the optimised B-spline
    FlownQuality direct_quality_;    ///< takeoff, vertical bypass, short-move quintic, braking tail
    int last_source_ = -1;
    std::deque<uint8_t> setpoint_from_planner_;
    WorstCycle worst_;
    // Dispatch period; must track the 20ms wall timer created in the constructor.
    static constexpr double kDispatchDt = 0.02;
    /// Replan timer rate; must track the 100ms wall timer created in the constructor.
    static constexpr double kReplanHz = 10.0;
    /// A plan moving less than this, with the goal further than kDegenerateGoalFar,
    /// is treated as degenerate. See the escape in replan_loop.
    static constexpr double kDegenerateNetTravel = 1.0;
    static constexpr double kDegenerateGoalFar = 5.0;
    // Set whenever setpoint_deque_ is refilled, so the next dispatched sample is excluded from
    // the jerk finite difference (see FlownQuality::add).
    // Dispatches remaining before the buffer reaches the join with the newest plan. The sample
    // AT the join is excluded from the jerk finite difference (see FlownQuality::add).
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
    CollisionMonitor collisions_;
    RouteChurn route_churn_{1.0};
    roscopter::RouteGate route_gate_;
    /// Last setpoint actually published; the anchor for a replan with an empty buffer.
    roscopter_msgs::msg::TrajectoryCommand last_dispatched_;
    bool have_last_dispatched_ = false;
    std::size_t degenerate_plans_ = 0, degenerate_escapes_ = 0;
    /// Goal the currently committed plan was optimised toward; see the commit gate.
    Eigen::Vector3d committed_goal_{std::numeric_limits<double>::quiet_NaN(),
                                    std::numeric_limits<double>::quiet_NaN(),
                                    std::numeric_limits<double>::quiet_NaN()};
    std::size_t commit_stats_replans_ = 0;
    rclcpp::Subscription<rosflight_msgs::msg::Status>::SharedPtr status_sub_;
    bool have_status_ = false;
    bool autopilot_in_control_ = false;

    /**
     * Should this sample count toward the mission metrics?
     *
     * mission_started_ goes true the moment a waypoint MESSAGE arrives, which is not the moment
     * the vehicle starts flying. Between the two the node is already dispatching setpoints while
     * the airframe sits on the ground under RC, so those samples accrue duration without
     * distance: leg 1 measured ~28.9 s of commanded setpoints covering ~0.32 m, which inflated
     * leg 1's duration, crushed its v_avg, and made its planning-time row incomparable with the
     * legs that followed.
     *
     * Gate per sample rather than latching, so a mid-flight RC override is excluded too -- those
     * samples are the pilot's, not the planner's.
     *
     * If no Status has ever arrived the gate opens, preserving the old behaviour rather than
     * silently reporting an empty table on a rig that does not publish it.
     */
    bool metrics_active() const { return !have_status_ || autopilot_in_control_; }

    double min_planning_distance_ = 1.5;
    double vertical_bypass_ratio_ = 0.35;
    /// Clearance demanded beyond the hull; folded into the grid's inflation radius. See the
    /// obstacle_margin parameter.
    double obstacle_margin_ = 0.30;
    /// Route committed by the last successful plan, in world coordinates. Fed back into A* each
    /// cycle for path hysteresis; cleared on a new waypoint.
    std::vector<Eigen::Vector3d> previous_route_;
    /// AGL above which short moves are committed; see the commit site in replan_loop.
    double takeoff_agl_ = 1.0;
    /// See takeoff_altitude_tol; gates advancing past the first waypoint.
    double takeoff_altitude_tol_ = 0.35;
    /// See takeoff_settle_speed; stops the gate opening mid-climb.
    double takeoff_settle_speed_ = 0.30;
    /// See takeoff_settle_timeout; bounds the wait so an unsettled vehicle cannot hang.
    double takeoff_settle_timeout_ = 8.0;
    /// Node time the settle wait began, 0 when not waiting.
    double settle_wait_start_ = 0.0;
    /// Node time the mission was first seen stalled beyond the fallback ceiling.
    double stall_since_ = 0.0;
    /// Latches once the run summary has been emitted, so the destructor cannot double-print
    /// after a normal mission end.
    bool summary_printed_ = false;
    /// Set when stall_timeout ends the run; reported in the summary.
    bool mission_stalled_ = false;
    /// Set once per stall episode so the snapshot is written once, not every cycle.
    /// Independent of mission_stalled_, which only tracks whether the run was ended.
    bool stall_reported_ = false;
    /// How many stall snapshots this run has written; suffixes the 2nd onward.
    int stall_episode_ = 0;
    /// Climb rate for the initial takeoff; see generate_direct_trajectory.
    double takeoff_speed_ = 1.0;
    /// Node time until which the mission holds before pulling the next waypoint; see the pull
    /// site in replan_loop and Waypoint::hold_seconds.
    double hold_until_ = 0.0;
    /// Consecutive failed replans; reset on any success. See the failure branch in replan_loop.
    int consecutive_replan_failures_ = 0;
    /// Failures tolerated before braking. At 10 Hz this is ~300 ms of stale flight, which keeps
    /// a one-off miss free while bounding how far a lost planner can carry the vehicle.
    static constexpr int replan_failures_before_brake_ = 3;
    /// Setpoints kept ahead of the brake (0.1 s), so deceleration starts from the committed
    /// state rather than as a velocity step.
    static constexpr std::size_t kBrakeReactionSetpoints = 5;
    /// Wall-clock stamp of the last published setpoint; drives the catch-up in dispatch_loop.
    double last_publish_stamp_ = 0.0;
    /// Most setpoints one dispatch may consume (0.5 s of trajectory). Past this the trajectory
    /// is allowed to fall behind rather than jumping the command a long way in one step.
    static constexpr std::size_t kMaxCatchUp = 25;
    // Map-update timing; see the instrumentation in lidar_cb().
    double scan_ms_sum_ = 0.0;
    double scan_ms_max_ = 0.0;
    std::size_t scan_points_sum_ = 0;
    std::size_t scan_n_ = 0;
    // Dispatch-rate watchdog; see check_dispatch_rate().
    double last_dispatch_stamp_ = 0.0;
    double dispatch_gap_sum_ = 0.0;
    std::size_t dispatch_gap_n_ = 0;
    std::size_t dispatch_late_n_ = 0;
    /**
     * Query radius for the buffered-trajectory collision re-check.
     *
     * Deliberately a half voxel, NOT the drone radius. is_occupied_radius flags a point when an
     * INFLATED voxel centre lies within the query radius, and the grid is already inflated by
     * (drone_physical_radius + obstacle_margin_) -- so the required clearance is encoded once, in
     * the inflation. Passing the drone radius here as well added it a second time: the check
     * demanded ~0.98 m of clearance while the corridors that produced the trajectory guaranteed
     * 0.49 m, so it could reject a trajectory the planner had just certified, and it got worse in
     * step with any increase to the margin. The half voxel only covers grid quantisation.
     */
    static constexpr double kClearanceQueryRadius = 0.25;
    // Waypoint arrival. Nominal radius is tested here; the stalled-progress backstop and its
    // tuning live in the monitor. See the parameter declarations and arrival_monitor.hpp.
    double arrival_radius_ = 1.0;
    roscopter::ArrivalMonitor arrival_;
    /// Whether the leg being recorded ended on the stalled-progress fallback rather than a real
    /// arrival. Reported per leg so a waypoint-accuracy claim is never built on a timeout.
    bool arrival_was_fallback_ = false;
    bool direct_move_active_ = false;
    std::size_t splice_countdown_ = 0;
    bool splice_armed_ = false;
    bool at_splice_ = false;
    bool last_reached_goal_ = false;
    // Wall-clock seconds when the currently-buffered trajectory was produced. Drives the
    // max_plan_age freshness check in replan_loop.
    double last_plan_time_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimumSnapManager>());
    rclcpp::shutdown();
    return 0;
}
