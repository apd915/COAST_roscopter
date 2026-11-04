#ifndef ROSCOPTER_NAVIGATION_ROUTE_GATE_HPP
#define ROSCOPTER_NAVIGATION_ROUTE_GATE_HPP

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace roscopter
{

/**
 * Directional filter on candidate plans -- discard a replan that would send the vehicle
 * somewhere meaningfully different NOW, keep the ones that merely refine the current route.
 *
 * WHY THIS AND NOT COMMITMENT. Three commitment designs were measured on hard s0 at v_max 8:
 *
 *     no commitment          T_trav med 101 s [74-405]    v_avg 1.74   wedges present
 *     hold the whole plan    T_trav med 135 s [82-146]    v_avg 1.69   no wedges, visible stutter
 *     hold route, refresh    T_trav med 484 s [341-627]   v_avg 0.58   far worse, 18 collisions
 *
 * Both commitment designs FROZE GEOMETRY, and that is what hurt. Holding the whole plan means
 * flying its decelerating tail, where replanning every cycle had only ever executed each plan's
 * fast opening. Holding the corridor chain is worse still: the chain lives in WORLD coordinates
 * while the state estimate drifts ~1.5 m over the hold window, so the held corridors stop
 * corresponding to the map being built in that same drifting frame.
 *
 * This freezes nothing. The planner runs in full every cycle and every ACCEPTED plan is fresh,
 * so the vehicle keeps flying openings and the world-frame staleness cannot arise. The only
 * change is that a flip-flop candidate is declined.
 *
 * WHAT IS COMPARED. Not corridor 0's axis. When the vehicle passes a turn waypoint, corridor 0
 * is consumed and the next corridor becomes corridor 0 -- a legitimate direction jump that a
 * naive ux[0] comparison reads as a flip-flop. Instead both routes are sampled over a common
 * arc-length window ahead of the vehicle and the WORST heading difference in that window is
 * taken. Arc length is immune to corridor consumption and changes smoothly as the vehicle
 * advances; taking the worst over a window rather than a single point means a divergence just
 * past the lookahead is still caught.
 *
 * A far-field change is harmless -- the vehicle replans past it long before reaching it -- which
 * is why the window is bounded rather than covering the whole route.
 */
class RouteGate
{
public:
  struct Config
  {
    /// Heading difference above which a candidate is declined. 0 DISABLES the gate entirely,
    /// so the default build is bit-identical to having no gate at all.
    double reject_angle_deg = 0.0;
    /// How far ahead of the vehicle the comparison window extends.
    double lookahead_m = 3.0;
    /// Force-accept after this many consecutive rejections. Without it, sustained rejection
    /// degenerates into plan-level commitment and brings back the stutter it exists to avoid.
    int max_consecutive_rejects = 10;

    /// How much better aimed at the goal a candidate must be to BYPASS the gate entirely,
    /// as a difference of direction cosines in [-2, 2].
    ///
    /// The override exists so one bad plan cannot self-defend (see the note at its use site).
    /// But at a junction offering two branches that are comparably goal-ward, the vehicle
    /// shifts a little each cycle and that flips which branch scores better -- so the
    /// anti-lock-in mechanism becomes the thing that lets the planner flip-flop.
    ///
    /// Measured across ten medium seeds: passing runs took this override 0-43 times, while
    /// the one wedged run (medium s4: 642 m flown on a 105 m route, 6 collisions, p90 route
    /// divergence 150.95 deg) took it 213 times. Raising it demands a MEANINGFULLY better
    /// candidate, so a genuine tie is declined and held.
    double alignment_margin = 0.10;
  };

  struct Decision
  {
    bool accept = true;
    /// Worst heading difference over the window; NaN when it could not be evaluated.
    double angle_deg = std::numeric_limits<double>::quiet_NaN();
    /// One of: "no-gate", "forced", "no-current", "too-short", "aligned", "diverged",
    /// "reject-bound". Reported, not parsed.
    const char * reason = "no-gate";
  };

  RouteGate() = default;
  explicit RouteGate(const Config & c) : cfg_(c) {}

  void set_config(const Config & c) { cfg_ = c; }
  const Config & config() const { return cfg_; }

  /**
   * @param current    route currently being flown (world coords); empty if none
   * @param candidate  newly planned route (world coords)
   * @param vehicle    stitch point the candidate was planned from
   * @param force      caller-side override -- collision on the current trajectory, goal moved,
   *                   or no trajectory buffered. Always accepts, and is counted separately.
   */
  Decision evaluate(
    const std::vector<Eigen::Vector3d> & current,
    const std::vector<Eigen::Vector3d> & candidate,
    const Eigen::Vector3d & vehicle,
    const Eigen::Vector3d & goal,
    bool force)
  {
    Decision d;
    if (cfg_.reject_angle_deg <= 0.0) { d.reason = "no-gate"; return d; }

    ++n_evaluated_;
    if (force) {
      ++n_forced_;
      end_streak();
      d.reason = "forced";
      return d;
    }
    if (current.size() < 2 || candidate.size() < 2) {
      end_streak();
      d.reason = "no-current";
      return d;
    }
    // The bound exists so sustained rejection cannot become plan-level commitment. But it must
    // NOT force the vehicle onto a route pointing AWAY from the goal: at a junction offering
    // north (goalward) and south-east (away), the gate correctly declines the south-east
    // candidate ten times and the bound then hands the vehicle to it -- the one plan it had most
    // consistently refused. Observed in flight as the planner deadlocking between two opposite
    // branches and eventually committing to the wrong one.
    //
    // A better-aimed candidate is already accepted outright below, so the bound is only ever
    // reached by candidates that are NOT better aimed. Releasing it for ones that are merely
    // COMPARABLE keeps its anti-deadlock purpose; releasing it for ones that point away defeats
    // the gate. The three hard overrides -- collision on the current trajectory, goal moved, no
    // buffered trajectory -- remain the escape hatches, and none of them can be blocked here.
    const bool bound_reached = streak_ >= cfg_.max_consecutive_rejects;

    // Align BOTH routes on the vehicle before comparing. A fresh plan normally begins at the
    // stitch point, so its own s0 is ~0 -- but assuming that silently mis-registers the two
    // windows by however far the routes' start points differ, and the symptom is a legitimate
    // corner reading as a 90 deg flip-flop because one window has crossed the corner and the
    // other has not. Projecting both costs one call and removes the assumption.
    const double s0_cur = arc_length_of_closest_point(vehicle, current);
    const double s0_cand = arc_length_of_closest_point(vehicle, candidate);
    const double angle =
      worst_heading_difference(current, s0_cur, candidate, s0_cand, cfg_.lookahead_m);
    if (std::isnan(angle)) {
      end_streak();
      d.reason = "too-short";      // not enough route ahead on one side to judge
      return d;
    }

    d.angle_deg = angle;
    angles_.push_back(angle);

    // NEVER DECLINE A CANDIDATE THAT IS BETTER AIMED AT THE GOAL.
    //
    // The gate knows which route is DIFFERENT, not which is BETTER, and previous_route_ only
    // updates on an accepted plan. So one bad plan getting through is self-defending: every
    // correction afterwards diverges from it and is declined, for up to
    // max_consecutive_rejects cycles -- a full second at 10 Hz -- and the vehicle flies the bad
    // route for all of it. Then the bound force-accepts, the streak resets, and it can recur.
    //
    // Observed as the vehicle departing diagonally upward and holding it while the planner was
    // commanding level flight, with 13 bound hits in one run and p90 divergence at 86.75 deg.
    // No other safeguard covers it: the route was not colliding, the goal had not moved, and
    // the buffer was full.
    //
    // Comparing goal alignment fixes it without weakening flip-flop suppression, because two
    // branches of a genuine junction are comparably goal-ward -- neither is MEANINGFULLY better,
    // so the tie still gets declined. A route pointing away from the goal, or climbing when the
    // goal is level, is not a tie.
    const double gain = goal_alignment_gain(current, candidate, vehicle, goal);
    if (gain > cfg_.alignment_margin) {
      ++n_better_aimed_;
      end_streak();
      d.reason = "better-aimed";
      return d;
    }

    // RELEASE THE BOUND UNCONDITIONALLY. Withholding it for candidates aimed away from the
    // goal was tried and REVERTED -- it is the lesser of two failures by a wide margin.
    //
    // The intent was sound: at a junction offering north (goalward) and south-east (away), the
    // gate declines south-east ten times and the bound then hands the vehicle to the one plan it
    // had most consistently refused. Withholding the release stops that.
    //
    // But it recreates, worse, the plan-level commitment the bound exists to prevent. Measured
    // on hard s0 at v_max 8: longest rejection streak went 10 -> 45, the bound was held 138
    // times, and T_trav 146 -> 312 s with L_path 170 -> 403 m (3.8x the direct distance) and
    // 13.0 m of along-track lag. The vehicle flew a route while refusing every correction to it.
    //
    // The asymmetry that makes this the right call: a bound that occasionally commits to a worse
    // branch costs one bad decision, recoverable on the next replan. A bound that can be held
    // indefinitely costs the whole run.
    if (bound_reached) {
      ++n_bound_;
      end_streak();
      d.reason = "reject-bound";
      return d;
    }
    if (bound_reached) ++n_bound_held_;

    if (angle > cfg_.reject_angle_deg) {
      ++n_rejected_;
      ++streak_;
      longest_streak_ = std::max(longest_streak_, static_cast<std::size_t>(streak_));
      d.accept = false;
      d.reason = "diverged";
    } else {
      end_streak();
      d.reason = "aligned";
    }
    return d;
  }

  /// How much better the candidate is aimed at the goal than the current route, as a
  /// difference of direction cosines in [-2, 2]. Positive means the candidate heads more
  /// directly toward the goal.
  static double goal_alignment_gain(
    const std::vector<Eigen::Vector3d> & current,
    const std::vector<Eigen::Vector3d> & candidate,
    const Eigen::Vector3d & vehicle,
    const Eigen::Vector3d & goal)
  {
    const Eigen::Vector3d to_goal = goal - vehicle;
    if (to_goal.norm() < 1e-6) return 0.0;
    const Eigen::Vector3d g = to_goal.normalized();
    const Eigen::Vector3d hc = heading_at(current, arc_length_of_closest_point(vehicle, current));
    const Eigen::Vector3d hn =
      heading_at(candidate, arc_length_of_closest_point(vehicle, candidate));
    if (hc.squaredNorm() < 0.5 || hn.squaredNorm() < 0.5) return 0.0;
    return hn.dot(g) - hc.dot(g);
  }

  /// Alignment gain above which the candidate counts as meaningfully better aimed. ~0.1 is
  /// about 8 deg of heading at typical angles: enough to ignore noise, small enough to catch a
  /// route that is genuinely pointing somewhere else.
  // alignment_margin moved to Config so it can be swept; see Config::alignment_margin.

  std::size_t better_aimed() const { return n_better_aimed_; }

  /// Worst heading difference (degrees) between the two routes over [0, window] ahead of s0
  /// on `a`. NaN when either route has too little length in the window to compare.
  static double worst_heading_difference(
    const std::vector<Eigen::Vector3d> & a, double s0_a,
    const std::vector<Eigen::Vector3d> & b, double s0_b, double window)
  {
    constexpr double kStep = 0.5;
    const double len_a = polyline_length(a) - s0_a;
    const double len_b = polyline_length(b) - s0_b;
    const double span = std::min({window, len_a, len_b});
    if (span < kStep) return std::numeric_limits<double>::quiet_NaN();

    double worst = 0.0;
    for (double s = 0.0; s <= span + 1e-9; s += kStep) {
      const Eigen::Vector3d ha = heading_at(a, s0_a + s);
      const Eigen::Vector3d hb = heading_at(b, s0_b + s);
      if (ha.squaredNorm() < 0.5 || hb.squaredNorm() < 0.5) continue;
      const double c = std::clamp(ha.dot(hb), -1.0, 1.0);
      worst = std::max(worst, std::acos(c) * 180.0 / M_PI);
    }
    return worst;
  }

  static double polyline_length(const std::vector<Eigen::Vector3d> & p)
  {
    double s = 0.0;
    for (std::size_t i = 1; i < p.size(); ++i) s += (p[i] - p[i - 1]).norm();
    return s;
  }

  /// Unit tangent of the polyline at arc length s. Zero vector if s is off the end.
  static Eigen::Vector3d heading_at(const std::vector<Eigen::Vector3d> & p, double s)
  {
    double acc = 0.0;
    for (std::size_t i = 1; i < p.size(); ++i) {
      const Eigen::Vector3d seg = p[i] - p[i - 1];
      const double len = seg.norm();
      if (len < 1e-9) continue;
      if (s <= acc + len) return seg / len;
      acc += len;
    }
    return Eigen::Vector3d::Zero();
  }

  /// Arc length along the polyline of the point closest to q -- where the vehicle sits on the
  /// route it is flying. The route begins where the last accepted plan began, which is behind
  /// the vehicle by now, so the two routes must be aligned on this before comparing.
  static double arc_length_of_closest_point(
    const Eigen::Vector3d & q, const std::vector<Eigen::Vector3d> & p)
  {
    double acc = 0.0, best_s = 0.0, best_d = std::numeric_limits<double>::max();
    for (std::size_t i = 1; i < p.size(); ++i) {
      const Eigen::Vector3d ab = p[i] - p[i - 1];
      const double len2 = ab.squaredNorm();
      double t = 0.0;
      if (len2 > 1e-12) t = std::clamp((q - p[i - 1]).dot(ab) / len2, 0.0, 1.0);
      const double d = (q - (p[i - 1] + t * ab)).norm();
      if (d < best_d) { best_d = d; best_s = acc + t * std::sqrt(len2); }
      acc += std::sqrt(len2);
    }
    return best_s;
  }

  std::size_t evaluated() const { return n_evaluated_; }
  std::size_t rejected() const { return n_rejected_; }
  std::size_t forced() const { return n_forced_; }
  std::size_t hit_bound() const { return n_bound_; }
  /// Times the bound was reached but WITHHELD because the candidate pointed away.
  std::size_t bound_held() const { return n_bound_held_; }
  std::size_t longest_reject_streak() const { return longest_streak_; }
  double reject_pct() const
  {
    return n_evaluated_ ? 100.0 * static_cast<double>(n_rejected_) / n_evaluated_ : 0.0;
  }
  /// Percentile of ALL measured angles, accepted and rejected. If the gate never fires, this is
  /// what tells you the threshold is wrong rather than leaving you guessing.
  double angle_percentile(double q) const
  {
    if (angles_.empty()) return -1.0;
    std::vector<double> v = angles_;
    std::sort(v.begin(), v.end());
    const std::size_t i =
      std::min(v.size() - 1, static_cast<std::size_t>(q * static_cast<double>(v.size())));
    return v[i];
  }

  void reset()
  {
    angles_.clear();
    n_evaluated_ = n_rejected_ = n_forced_ = n_bound_ = n_better_aimed_ = n_bound_held_ = 0;
    streak_ = longest_streak_ = 0;
  }

private:
  void end_streak() { streak_ = 0; }

  Config cfg_;
  std::vector<double> angles_;
  std::size_t n_evaluated_ = 0, n_rejected_ = 0, n_forced_ = 0, n_bound_ = 0;
  std::size_t n_better_aimed_ = 0, n_bound_held_ = 0;
  int streak_ = 0;
  std::size_t longest_streak_ = 0;
};

}  // namespace roscopter

#endif  // ROSCOPTER_NAVIGATION_ROUTE_GATE_HPP
