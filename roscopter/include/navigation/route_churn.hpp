#ifndef ROSCOPTER_NAVIGATION_ROUTE_CHURN_HPP
#define ROSCOPTER_NAVIGATION_ROUTE_CHURN_HPP

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace roscopter
{

/**
 * How far ahead consecutive plans start to disagree.
 *
 * WHY THIS EXISTS. Travel time is being eaten by the planner changing its mind -- at v_max 8 the
 * vehicle averaged 1.61 m/s, 20% of the limit, on 955 replans and a path 1.68x the direct
 * distance. The obvious fix is to commit to a route and only replan when new sensor data
 * invalidates it. But two commitment mechanisms have already been built and BOTH measured worse
 * (the anti-stutter gate cost +31% travel time; path hysteresis collapsed v_avg to 1.02 m/s), so
 * a third guess is not worth another revert.
 *
 * This measures the thing the guess depends on. Each replan, it asks at what ARC LENGTH along the
 * new route the new route departs from the previous one:
 *
 *   near-field divergence (small arc length, e.g. < 3 m) -- consecutive plans disagree about
 *       where to go RIGHT NOW. That is junction indecision, and committing to a route is the
 *       right fix.
 *   far-field divergence -- plans agree on the near route and differ only far ahead, which the
 *       vehicle never flies before the next replan. Committing fixes nothing, and the churn is
 *       coming from somewhere else.
 *
 * Arc length rather than corridor index: corridor count varies between plans and the stitch point
 * moves every cycle, so index 1 is not the same place twice. Distance is comparable.
 */
class RouteChurn
{
public:
  /// @param tolerance_m  lateral distance beyond which two routes count as diverged
  explicit RouteChurn(double tolerance_m = 1.0) : tol_(tolerance_m) {}

  /**
   * Record one replan. Pass the newly planned route as a polyline of waypoints in world
   * coordinates; the previous route is remembered internally.
   */
  void add(const std::vector<Eigen::Vector3d> & route)
  {
    if (route.size() >= 2 && previous_.size() >= 2) {
      ++n_compared_;
      const double d = divergence_distance(route, previous_);
      bool near = false;
      if (d < 0.0) {
        ++n_same_;                      // agreed over the whole overlap
      } else {
        divergences_.push_back(d);
        near = (d <= near_field_m_);
        if (near) ++n_near_;
      }
      // EPISODES, not just a rate. The failure that costs travel time is not churn spread
      // thinly across a run -- it is a WEDGE: the planner flipping between two routes for
      // seconds on end while the vehicle crawls. A rate cannot distinguish the two. Measured on
      // hard s0 at v_max 8, a run with 17.9% near-field churn took 405 s and flew 407 m, against
      // 74 s / 164 m for a run at 4.6% -- and the bad run's divergences were close in (median
      // 1.19 m vs 5.50 m). Averages hide that; streak length does not.
      if (near) {
        ++streak_;
        longest_streak_ = std::max(longest_streak_, streak_);
      } else {
        if (streak_ >= kEpisodeMin) { ++n_episodes_; episode_replans_ += streak_; }
        streak_ = 0;
      }
    }
    previous_ = route;
  }

  /// Close an open streak; call before reading episode stats at end of run.
  void finalize()
  {
    if (streak_ >= kEpisodeMin) { ++n_episodes_; episode_replans_ += streak_; }
    streak_ = 0;
  }

  std::size_t episodes() const { return n_episodes_; }
  std::size_t longest_streak() const { return longest_streak_; }
  std::size_t episode_replans() const { return episode_replans_; }
  /// Share of all replans spent inside a sustained near-field churn episode.
  double episode_pct() const
  {
    return n_compared_ ? 100.0 * static_cast<double>(episode_replans_) / n_compared_ : 0.0;
  }
  static constexpr std::size_t kEpisodeMin = 5;   // >= 0.5 s at the 10 Hz replan rate

  /// Arc length along `a` at which it first leaves `b` by more than the tolerance.
  /// Returns -1 when the routes never diverge.
  ///
  /// Marches along the segments rather than testing only the vertices. Vertex-only testing
  /// reports the divergence at the first vertex PAST the departure, which on a route that peels
  /// off immediately after a shared waypoint reads as several metres of agreement that never
  /// existed -- biasing the measurement late and under-reporting exactly the near-field churn
  /// this class is built to detect.
  double divergence_distance(
    const std::vector<Eigen::Vector3d> & a, const std::vector<Eigen::Vector3d> & b) const
  {
    constexpr double kStep = 0.25;   // half a voxel
    double s = 0.0;
    if (point_to_polyline(a.front(), b) > tol_) return 0.0;
    for (std::size_t i = 1; i < a.size(); ++i) {
      const Eigen::Vector3d seg = a[i] - a[i - 1];
      const double len = seg.norm();
      if (len < 1e-9) continue;
      const int steps = std::max(1, static_cast<int>(std::ceil(len / kStep)));
      for (int k = 1; k <= steps; ++k) {
        const double t = static_cast<double>(k) / steps;
        if (point_to_polyline(a[i - 1] + t * seg, b) > tol_) return s + t * len;
      }
      s += len;
    }
    return -1.0;
  }

  std::size_t compared() const { return n_compared_; }
  std::size_t unchanged() const { return n_same_; }
  std::size_t near_field() const { return n_near_; }

  double unchanged_pct() const
  {
    return n_compared_ ? 100.0 * static_cast<double>(n_same_) / n_compared_ : 0.0;
  }
  /// Share of ALL compared replans whose route changed within near_field_m of the vehicle.
  double near_field_pct() const
  {
    return n_compared_ ? 100.0 * static_cast<double>(n_near_) / n_compared_ : 0.0;
  }

  /// Percentile of the divergence distances (0..1). Returns -1 when nothing ever diverged.
  double percentile(double q) const
  {
    if (divergences_.empty()) return -1.0;
    std::vector<double> v = divergences_;
    std::sort(v.begin(), v.end());
    const std::size_t i =
      std::min(v.size() - 1, static_cast<std::size_t>(q * static_cast<double>(v.size())));
    return v[i];
  }

  double near_field_threshold() const { return near_field_m_; }

  void reset()
  {
    previous_.clear();
    divergences_.clear();
    n_compared_ = n_same_ = n_near_ = 0;
    streak_ = longest_streak_ = n_episodes_ = episode_replans_ = 0;
  }

private:
  /// Shortest distance from p to the polyline b (segment-wise, not vertex-wise -- a route with
  /// sparse waypoints would otherwise read as diverged simply for being sparse).
  static double point_to_polyline(const Eigen::Vector3d & p, const std::vector<Eigen::Vector3d> & b)
  {
    double best = std::numeric_limits<double>::max();
    for (std::size_t i = 1; i < b.size(); ++i) {
      const Eigen::Vector3d ab = b[i] - b[i - 1];
      const double len2 = ab.squaredNorm();
      double t = 0.0;
      if (len2 > 1e-12) {
        t = std::clamp((p - b[i - 1]).dot(ab) / len2, 0.0, 1.0);
      }
      best = std::min(best, (p - (b[i - 1] + t * ab)).norm());
    }
    return best;
  }

  double tol_;
  double near_field_m_ = 3.0;
  std::vector<Eigen::Vector3d> previous_;
  std::vector<double> divergences_;
  std::size_t n_compared_ = 0, n_same_ = 0, n_near_ = 0;
  std::size_t streak_ = 0, longest_streak_ = 0, n_episodes_ = 0, episode_replans_ = 0;
};

}  // namespace roscopter

#endif  // ROSCOPTER_NAVIGATION_ROUTE_CHURN_HPP
