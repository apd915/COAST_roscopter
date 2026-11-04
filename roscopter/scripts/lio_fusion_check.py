#!/usr/bin/env python3
"""Score the estimator against sim truth during one flight, with and without LIO fusion.

WHY THIS EXISTS: "the EKF fuses LIO" is not verified by the EKF running, or even by the vehicle
flying. With GPS on and trusted to 1 cm (estimator.yaml), LIO at 5 cm barely moves the estimate,
so a broken LIO path flies exactly like a working one. The only honest check is error against
TRUTH, compared across configurations -- in particular GPS-denied, where LIO is the only
horizontal position source and its absence is obvious.

Measures, while airborne (truth altitude > 0.5 m):
  estimate - truth   horizontal and vertical, RMS / max / final
  bridge   - truth   lio/odom_ned against truth at the same stamp: checks the frame
                     conversion and lever arm independently of the EKF
  map error          for the cloud the PLANNER receives (--cloud-topic), each point's distance
                     to the nearest true obstacle surface in --env. ~0 for the mock's truth-pose
                     cloud and for lidar_preprocessor fed truth; ~the estimate error otherwise.
                     This is how far the planner's map is displaced from the world.
  collisions         contact events reported by the sim on mock_env/collision

Stops when the vehicle has held within 2 m of the goal for 3 s, or at --timeout.

  python3 lio_fusion_check.py --label lio_gps_denied --goal 105 0 -3 --timeout 120 \
      --env .../forests/forest_easy_s0.yaml --cloud-topic lidar/points_ned
"""
import argparse
import bisect
import json
import math
import sys

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from roscopter_msgs.msg import State
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool


class World:
    """The true obstacle surfaces, from the same environment file the sim loads."""

    def __init__(self, env_file):
        with open(env_file) as f:
            params = yaml.safe_load(f)["/mock_environment"]["ros__parameters"]
        cyl = np.array(params.get("cylinders", []), dtype=float).reshape(-1, 5)
        box = np.array(params.get("boxes", []), dtype=float).reshape(-1, 6)
        self.cyl_c, self.cyl_r, self.cyl_h = cyl[:, :3], cyl[:, 3], cyl[:, 4]
        self.box_min, self.box_max = box[:, :3], box[:, 3:]
        # standalone_sim_params.yaml enables the ground at z=0; environment files may turn it off.
        self.ground_z = params.get("ground_z_ned", 0.0) if params.get("ground_enabled", True) else None

    def distance(self, pts):
        """Unsigned distance from each point (N x 3, NED) to the nearest obstacle surface."""
        best = np.full(len(pts), np.inf)
        if len(self.cyl_c):
            d = pts[:, None, :] - self.cyl_c[None, :, :]
            radial = np.hypot(d[..., 0], d[..., 1]) - self.cyl_r
            axial = np.abs(d[..., 2]) - self.cyl_h
            outside = np.hypot(np.maximum(radial, 0), np.maximum(axial, 0))
            inside = np.minimum(np.maximum(radial, axial), 0)
            best = np.minimum(best, np.abs(outside + inside).min(axis=1))
        if len(self.box_min):
            lo = self.box_min[None] - pts[:, None]
            hi = pts[:, None] - self.box_max[None]
            q = np.maximum(lo, hi)
            outside = np.linalg.norm(np.maximum(q, 0), axis=2)
            inside = np.minimum(q.max(axis=2), 0)
            best = np.minimum(best, np.abs(outside + inside).min(axis=1))
        if self.ground_z is not None:
            best = np.minimum(best, np.abs(pts[:, 2] - self.ground_z))
        return best


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Checker(Node):
    def __init__(self, args):
        super().__init__("lio_fusion_check")
        self.args = args
        self.truth = None
        self.truth_hist_t = []   # truth header stamps, for matching bridge output
        self.truth_hist_p = []
        self.est_err = []        # (horizontal, vertical) per sample
        self.bridge_err = []
        self.n_est = 0
        self.n_bridge = 0
        self.at_goal_since = None
        self.done = False
        self.start = self.get_clock().now()
        self.world = World(args.env) if args.env else None
        self.map_err = []        # per-point surface distance, sampled
        self.n_clouds = 0
        self.cloud_pts = []      # points per cloud, to confirm the planner is being fed
        self.collisions = 0
        self.in_contact = False
        self.rng = np.random.default_rng(0)

        self.create_subscription(State, "sim/roscopter/state", self.on_truth, 50)
        self.create_subscription(State, "estimated_state", self.on_estimate, 50)
        self.create_subscription(Odometry, "lio/odom_ned", self.on_bridge, 50)
        self.create_subscription(Bool, "mock_env/collision", self.on_collision, 10)
        if self.world is not None:
            self.create_subscription(PointCloud2, args.cloud_topic, self.on_cloud, 5)
        self.create_timer(0.5, self.check_done)

    def airborne(self):
        return self.truth is not None and -self.truth.p_d > 0.5

    def on_truth(self, msg):
        self.truth = msg
        self.truth_hist_t.append(stamp_s(msg.header.stamp))
        self.truth_hist_p.append((msg.p_n, msg.p_e, msg.p_d))
        if len(self.truth_hist_t) > 4000:
            del self.truth_hist_t[:2000]
            del self.truth_hist_p[:2000]

    def on_estimate(self, msg):
        # The estimator publishes at ~390 Hz; every 10th sample is plenty and keeps Python up.
        self.n_est += 1
        if self.n_est % 10 or not self.airborne():
            return
        t = self.truth
        self.est_err.append((math.hypot(msg.p_n - t.p_n, msg.p_e - t.p_e), msg.p_d - t.p_d))

    def on_bridge(self, msg):
        self.n_bridge += 1
        if not self.airborne() or not self.truth_hist_t:
            return
        # Match on stamp, not arrival: the bridge output is ~30 ms old by the time it lands.
        s = stamp_s(msg.header.stamp)
        i = min(bisect.bisect_left(self.truth_hist_t, s), len(self.truth_hist_t) - 1)
        if abs(self.truth_hist_t[i] - s) > 0.05:
            return
        n, e, d = self.truth_hist_p[i]
        p = msg.pose.pose.position
        self.bridge_err.append((math.hypot(p.x - n, p.y - e), p.z - d))

    def on_collision(self, msg):
        # Count entries into contact, not samples, matching the sim's own accounting.
        if msg.data and not self.in_contact and self.airborne():
            self.collisions += 1
        self.in_contact = msg.data

    def on_cloud(self, msg):
        self.n_clouds += 1
        self.cloud_pts.append(msg.width * msg.height)
        # Every 5th scan, 300 points: plenty for the statistics, cheap enough for Python.
        if self.n_clouds % 5 or not self.airborne():
            return
        pts = np.array([(p[0], p[1], p[2]) for p in
                        point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])
        if len(pts) == 0:
            return
        if len(pts) > 300:
            pts = pts[self.rng.choice(len(pts), 300, replace=False)]
        self.map_err.extend(self.world.distance(pts).tolist())

    def check_done(self):
        elapsed = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        if elapsed > self.args.timeout:
            self.get_logger().warn(f"Timeout after {elapsed:.0f} s -- mission did not complete.")
            self.done = True
            return
        if self.truth is None:
            return
        g = self.args.goal
        dist = math.dist((self.truth.p_n, self.truth.p_e, self.truth.p_d), g)
        if dist < 2.0:
            self.at_goal_since = self.at_goal_since or elapsed
            if elapsed - self.at_goal_since > 3.0:
                self.done = True
        else:
            self.at_goal_since = None


def stats(errs):
    if not errs:
        return None
    h = [e[0] for e in errs]
    v = [abs(e[1]) for e in errs]
    rms = lambda xs: math.sqrt(sum(x * x for x in xs) / len(xs))
    return {"n": len(errs), "hor_rms": rms(h), "hor_max": max(h), "hor_final": h[-1],
            "ver_rms": rms(v), "ver_max": max(v), "ver_final": v[-1]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--goal", nargs=3, type=float, default=[105.0, 0.0, -3.0])
    ap.add_argument("--timeout", type=float, default=150.0)
    ap.add_argument("--env", help="Environment YAML the sim loaded; enables the map-error check.")
    ap.add_argument("--cloud-topic", default="/mock_env/lidar",
                    help="The cloud the planner subscribes to (its lidar_topic).")
    args = ap.parse_args()

    rclpy.init()
    node = Checker(args)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)

    reached = node.at_goal_since is not None
    map_stats = None
    if node.map_err:
        e = np.array(node.map_err)
        map_stats = {"n": len(e), "median": float(np.median(e)), "p95": float(np.percentile(e, 95)),
                     "max": float(e.max()), "pts_per_scan": float(np.mean(node.cloud_pts))}
    result = {"label": args.label, "reached_goal": reached, "collisions": node.collisions,
              "bridge_msgs": node.n_bridge,
              "estimate": stats(node.est_err), "bridge": stats(node.bridge_err), "map": map_stats}
    node.destroy_node()
    rclpy.shutdown()

    print(json.dumps(result))
    print(f"  goal {'REACHED' if reached else 'NOT reached'}, {node.collisions} collision(s)")
    if map_stats:
        print(f"  map     : surface distance median {map_stats['median']:.3f}  p95 {map_stats['p95']:.3f}  "
              f"max {map_stats['max']:.3f} m   ({map_stats['n']} pts sampled, "
              f"{map_stats['pts_per_scan']:.0f} pts/scan on {args.cloud_topic})")
    elif args.env:
        print(f"  map     : no airborne clouds received on {args.cloud_topic}")
    for name in ("estimate", "bridge"):
        s = result[name]
        if s is None:
            print(f"  {name:8s}: no airborne samples")
            continue
        print(f"  {name:8s}: horizontal rms {s['hor_rms']:.3f}  max {s['hor_max']:.3f}  "
              f"final {s['hor_final']:.3f} m | vertical rms {s['ver_rms']:.3f}  "
              f"max {s['ver_max']:.3f} m   ({s['n']} samples)")
    return 0 if reached else 1


if __name__ == "__main__":
    sys.exit(main())
