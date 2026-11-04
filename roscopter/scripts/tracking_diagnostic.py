#!/usr/bin/env python3
"""Compare every link in the trajectory-following chain at once.

WHY THIS EXISTS
---------------
`tracking_error` alone cannot answer "why are we flying high", because it is computed from
`estimated_state` -- the same signal the controller regulates. If the state estimate is biased,
the controller drives the ESTIMATED error to zero while the vehicle sits somewhere else, and
tracking_error reads small the whole time. Tightening gains in that case tracks the wrong
number harder.

This subscribes to the whole chain and lines it up on one row so the failure localises:

    planner --trajectory_command--> follower --high_level_command--> controller
                                       ^  ^
                        estimated_state |  | sim/truth_state (sim only)

READING THE OUTPUT (all NED, so p_d MORE NEGATIVE = HIGHER)
  err_est   = cmd_d - est_d    what the controller believes and is acting on
  err_true  = cmd_d - truth_d  where the vehicle actually is (what safety depends on)
  est_bias  = truth_d - est_d  the estimator's own error

  est_bias small, err_true ~ err_est  -> real CONTROL error. Gains/limits are the place to look.
  est_bias carries most of err_true   -> ESTIMATOR error. Gains cannot fix it.

Also watches message RATES. trajectory_command is published at 50 Hz with queue depth 10 but
subscribed with depth 1, so a busy follower silently drops setpoints and acts on a stale
command -- which looks like tracking error but is not.

USAGE
  source install/setup.bash
  python3 src/roscopter/roscopter/scripts/tracking_diagnostic.py
  python3 src/roscopter/roscopter/scripts/tracking_diagnostic.py --csv /tmp/track.csv

Ctrl-C prints a summary and a verdict.
"""

import argparse
import math
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from roscopter_msgs.msg import (TrajectoryCommand, State, ControllerCommand,
                                TrackingError, ComputationTimes)
try:
    from rosflight_msgs.msg import SimState, Status
    HAVE_SIM = True
except ImportError:      # hardware build without the sim messages
    HAVE_SIM = False


class RateMeter:
    """Message rate over a sliding window, so a stalled topic is visible immediately."""

    def __init__(self, window=2.0):
        self.window = window
        self.stamps = []

    def tick(self):
        now = time.monotonic()
        self.stamps.append(now)
        cutoff = now - self.window
        while self.stamps and self.stamps[0] < cutoff:
            self.stamps.pop(0)

    def hz(self):
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / span if span > 1e-6 else 0.0


class LegAcc:
    """Per-leg accumulation. Mission-wide means hide the shape of the flight: a clean 60 s
    followed by a bad 20 s reads as uniformly mediocre, which is exactly the case that needs
    telling apart."""

    __slots__ = ("n", "xt", "xt_h", "xt_v", "lag", "max_xt", "exceeded", "rc_bad", "bias_h",
                 "bias_v")

    def __init__(self):
        self.n = 0
        self.xt = self.xt_h = self.xt_v = self.lag = 0.0
        self.max_xt = 0.0
        self.exceeded = 0
        self.rc_bad = 0
        self.bias_h = self.bias_v = 0.0

    def add(self, xt, xt_h, xt_v, lag, exceeded, rc_bad, bias_h, bias_v):
        self.n += 1
        self.xt += xt
        self.xt_h += xt_h
        self.xt_v += xt_v
        self.lag += lag
        self.max_xt = max(self.max_xt, xt)
        self.exceeded += 1 if exceeded else 0
        self.rc_bad += 1 if rc_bad else 0
        self.bias_h += bias_h
        self.bias_v += bias_v


class TrackingDiagnostic(Node):
    def __init__(self, csv_path=None, rate_hz=5.0, min_agl=0.5):
        super().__init__("tracking_diagnostic")

        # Height above ground (m) below which samples are treated as pre-liftoff and
        # discarded, even though the vehicle is already armed+offboard. See the gate in
        # the update path.
        self.min_agl = min_agl

        # Match the follower's own QoS so we see what IT sees, not a more forgiving view.
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        self.cmd = None
        self.state = None
        self.truth = None
        self.hlc = None
        self.terr = None
        self.status = None

        # Rolling history of commanded positions, so we can measure distance to the PATH rather
        # than to the reference POINT. Those are different quantities and conflating them is
        # misleading: a vehicle lagging 1.3 m along a straight leg shows 1.3 m of "position
        # error" while sitting exactly on the path -- visually perfect, and still inside the
        # corridor, because the corridor contains the whole path. Only the CROSS-TRACK part is
        # a deviation from the proven-clear volume.
        self.cmd_hist = deque(maxlen=600)   # ~12 s at 50 Hz

        self.r_cmd = RateMeter()
        self.r_state = RateMeter()
        self.r_hlc = RateMeter()

        self.create_subscription(TrajectoryCommand, "trajectory_command", self._cmd, qos)
        self.create_subscription(State, "estimated_state", self._state, qos)
        self.create_subscription(ControllerCommand, "high_level_command", self._hlc, qos)
        self.create_subscription(TrackingError, "tracking_error", self._terr, qos)
        # The planner stamps each replan with its leg index, so both tools cut the flight at
        # exactly the same boundaries.
        self.create_subscription(ComputationTimes, "computation_times", self._ct, qos)
        if HAVE_SIM:
            self.create_subscription(SimState, "sim/truth_state", self._truth, qos)
            self.create_subscription(Status, "status", self._status, qos)

        # Accumulators for the closing summary.
        self.n = 0
        # Per axis (N, E, D). Summarising only D was how the first version of this tool missed
        # a 1.3 m HORIZONTAL estimator bias while reporting the vertical axis as "36% estimator".
        self.sum_abs_est = [0.0, 0.0, 0.0]
        self.sum_abs_true = [0.0, 0.0, 0.0]
        self.sum_abs_bias = [0.0, 0.0, 0.0]
        self.sum_signed_true = [0.0, 0.0, 0.0]
        self.max_abs_true = [0.0, 0.0, 0.0]
        self.sum_horiz_true = 0.0
        self.sum_horiz_bias = 0.0
        self.sum_norm_true = 0.0
        self.max_norm_true = 0.0
        self.n_margin_exceeded = 0
        self.sum_xt = 0.0
        self.sum_xt_h = 0.0
        self.sum_xt_v = 0.0
        self.sum_lag = 0.0
        self.max_xt = 0.0
        self.n_xt_exceeded = 0
        # rc_override is a BITFIELD. trajectory_follower.cpp does `if (rc_override)
        # clear_integrators()`, so ANY bit set -- including OFFBOARD_*_INACTIVE, which is about
        # stale offboard commands, not the pilot -- wipes the integrators every cycle. That
        # would hold a steady-state error open indefinitely. Count how often each bit is set.
        self.rc_bits_seen = {}
        self.n_rc_nonzero = 0
        # Samples discarded because the vehicle was armed+offboard but still on the ground.
        self.n_preflight = 0
        self.legs = {}
        self.leg = 0
        self.sum_hz_cmd = 0.0

        self.csv = open(csv_path, "w") if csv_path else None
        if self.csv:
            # All three N/E/D channels, so a departure can be attributed to an axis. The
            # vertical-only form could not show the "up AND forward" transient at all.
            self.csv.write("t,"
                           "cmd_n,est_n,truth_n,cmd_e,est_e,truth_e,cmd_d,est_d,truth_d,"
                           "cmd_vn,cmd_ve,cmd_vd,"
                           "err_est_d,err_true_d,est_bias_d,"
                           "err_est_norm,err_true_norm,thrust,hz_cmd,hz_state\n")

        self.t0 = time.monotonic()
        self.rows = 0
        self.create_timer(1.0 / rate_hz, self._report)

    # --- callbacks ---------------------------------------------------------------------
    def _cmd(self, m):
        self.cmd = m
        self.cmd_hist.append((float(m.position[0]), float(m.position[1]), float(m.position[2])))
        self.r_cmd.tick()

    def _state(self, m):
        self.state = m
        self.r_state.tick()

    def _hlc(self, m):
        self.hlc = m
        self.r_hlc.tick()

    def _terr(self, m):
        self.terr = m

    def _ct(self, m):
        self.leg = int(m.leg)

    def _truth(self, m):
        self.truth = m

    def _status(self, m):
        self.status = m
        if m.rc_override:
            self.n_rc_nonzero += 1
            for bit, name in ((0x01, "ATT_SWITCH"), (0x02, "THR_SWITCH"), (0x04, "X"),
                              (0x08, "Y"), (0x10, "Z"), (0x20, "T"),
                              (0x40, "OFFBOARD_X_INACTIVE"), (0x80, "OFFBOARD_Y_INACTIVE"),
                              (0x100, "OFFBOARD_Z_INACTIVE"), (0x200, "OFFBOARD_T_INACTIVE")):
                if m.rc_override & bit:
                    self.rc_bits_seen[name] = self.rc_bits_seen.get(name, 0) + 1

    def _cross_track(self, p):
        """Shortest vector from the commanded PATH to point p, and the along-track lag.

        Returns (cross_vec, cross_norm, along_lag). along_lag is how far back along the path the
        closest point is from the current reference point -- i.e. how late the vehicle is,
        expressed in metres of path. A large lag with a small cross_norm means the tracking is
        geometrically fine and merely behind schedule.
        """
        if len(self.cmd_hist) < 2:
            return (0.0, 0.0, 0.0), 0.0, 0.0

        pts = list(self.cmd_hist)
        best_d2 = float("inf")
        best_vec = (0.0, 0.0, 0.0)
        best_i = len(pts) - 1
        best_t = 0.0
        for i in range(len(pts) - 1):
            ax, ay, az = pts[i]
            bx, by, bz = pts[i + 1]
            dx, dy, dz = bx - ax, by - ay, bz - az
            seg2 = dx * dx + dy * dy + dz * dz
            if seg2 < 1e-12:
                t = 0.0
            else:
                t = ((p[0] - ax) * dx + (p[1] - ay) * dy + (p[2] - az) * dz) / seg2
                t = max(0.0, min(1.0, t))
            cx, cy, cz = ax + t * dx, ay + t * dy, az + t * dz
            vx, vy, vz = cx - p[0], cy - p[1], cz - p[2]
            d2 = vx * vx + vy * vy + vz * vz
            if d2 < best_d2:
                best_d2 = d2
                best_vec = (vx, vy, vz)
                best_i = i
                best_t = t

        # Path length from the closest point forward to the newest commanded point.
        lag = 0.0
        for j in range(best_i, len(pts) - 1):
            ax, ay, az = pts[j]
            bx, by, bz = pts[j + 1]
            seg = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2 + (bz - az) ** 2)
            lag += seg * (1.0 - best_t) if j == best_i else seg
        return best_vec, math.sqrt(best_d2), lag

    # --- reporting ---------------------------------------------------------------------
    def _report(self):
        if self.cmd is None or self.state is None:
            waiting = []
            if self.cmd is None:
                waiting.append("trajectory_command")
            if self.state is None:
                waiting.append("estimated_state")
            self.get_logger().info("waiting for: %s" % ", ".join(waiting), throttle_duration_sec=3.0)
            return

        cmd_d = float(self.cmd.position[2])
        est_d = float(self.state.p_d)
        err_est_d = cmd_d - est_d

        # SimState pose is inertial NED, directly comparable to p_d.
        truth_d = float(self.truth.pose.position.z) if self.truth else float("nan")
        err_true_d = cmd_d - truth_d if self.truth else float("nan")
        est_bias_d = truth_d - est_d if self.truth else float("nan")

        err_est_norm = math.sqrt(sum((float(self.cmd.position[i]) - v) ** 2
                                     for i, v in enumerate((self.state.p_n,
                                                            self.state.p_e,
                                                            self.state.p_d))))
        if self.truth:
            tp = self.truth.pose.position
            err_true_norm = math.sqrt(sum((float(self.cmd.position[i]) - v) ** 2
                                          for i, v in enumerate((tp.x, tp.y, tp.z))))
        else:
            err_true_norm = float("nan")

        if self.truth:
            tp = self.truth.pose.position
            xt_vec, xt, lag = self._cross_track((tp.x, tp.y, tp.z))
            xt_h = math.hypot(xt_vec[0], xt_vec[1])
            xt_v = abs(xt_vec[2])
        else:
            xt_vec, xt, lag, xt_h, xt_v = (0.0, 0.0, 0.0), float("nan"), float("nan"), float("nan"), float("nan")

        thrust = float(self.hlc.cmd4) if self.hlc else float("nan")
        hz_cmd = self.r_cmd.hz()
        hz_state = self.r_state.hz()

        if self.rows % 20 == 0:
            print("\n{:>7} {:>8} {:>8} {:>8} | {:>8} {:>9} | {:>7} {:>7} {:>7} {:>7} | {:>6}"
                  .format("t[s]", "cmd_d", "est_d", "truth_d",
                          "err_true", "est_bias",
                          "xtrack", "xt_hor", "xt_ver", "lag", "hz_cmd"))
            print("  (xtrack = distance to the PATH -- the corridor number.  "
                  "lag = metres behind along it)")
            print("-" * 110)
        self.rows += 1

        print("{:7.1f} {:8.2f} {:8.2f} {:8.2f} | {:8.2f} {:9.2f} | {:7.2f} {:7.2f} {:7.2f} {:7.2f} | "
              "{:6.1f}".format(
                  time.monotonic() - self.t0, cmd_d, est_d, truth_d,
                  err_true_d, est_bias_d,
                  xt, xt_h, xt_v, lag, hz_cmd))

        if self.csv:
            cmd_n = float(self.cmd.position[0])
            cmd_e = float(self.cmd.position[1])
            est_n = float(self.state.p_n)
            est_e = float(self.state.p_e)
            truth_n = float(self.truth.pose.position.x) if self.truth else float("nan")
            truth_e = float(self.truth.pose.position.y) if self.truth else float("nan")
            cmd_vn = float(self.cmd.velocity[0])
            cmd_ve = float(self.cmd.velocity[1])
            cmd_vd = float(self.cmd.velocity[2])
            self.csv.write(
                ("{:.3f}," + ",".join(["{:.4f}"] * 12) + ",{:.4f},{:.4f},{:.4f},"
                 "{:.4f},{:.4f},{:.4f},{:.2f},{:.2f}\n").format(
                    time.monotonic() - self.t0,
                    cmd_n, est_n, truth_n, cmd_e, est_e, truth_e, cmd_d, est_d, truth_d,
                    cmd_vn, cmd_ve, cmd_vd,
                    err_est_d, err_true_d, est_bias_d,
                    err_est_norm, err_true_norm, thrust, hz_cmd, hz_state))
            self.csv.flush()

        # Only accumulate while the autopilot is actually flying, so time on the ground does
        # not dilute the statistics (the same trap the planner's metrics had).
        # Gate on armed+offboard ONLY. Including "rc_override == 0" here made the per-leg
        # rc_override column structurally impossible to read: samples with the bit set were
        # filtered out before the accumulator saw them, so it always reported 0% and the
        # correlation it exists to test could never appear.
        flying = True
        if self.status is not None:
            flying = bool(self.status.armed) and bool(self.status.offboard)

        # armed+offboard goes true BEFORE the vehicle physically leaves the ground, and the
        # commanded altitude is already at the first waypoint by then. Those samples report
        # |cmd_d - truth_d| = the full commanded altitude: with waypoints at -4.0 the summary
        # showed peak_true, the 3-D peak and the cross-track peak all pinned at ~4.000 m, which
        # is not an excursion at all, just the vehicle sitting on the ground. It also inflated
        # the "% over margin" line. Require the vehicle to actually be off the ground.
        if flying and self.truth:
            if -float(self.truth.pose.position.z) < self.min_agl:
                self.n_preflight += 1
                flying = False

        if flying and self.truth:
            self.n += 1
            tp = self.truth.pose.position
            truth_v = (tp.x, tp.y, tp.z)
            est_v = (self.state.p_n, self.state.p_e, self.state.p_d)
            for i in range(3):
                e_est = float(self.cmd.position[i]) - float(est_v[i])
                e_true = float(self.cmd.position[i]) - float(truth_v[i])
                bias = float(truth_v[i]) - float(est_v[i])
                self.sum_abs_est[i] += abs(e_est)
                self.sum_abs_true[i] += abs(e_true)
                self.sum_abs_bias[i] += abs(bias)
                self.sum_signed_true[i] += e_true
                self.max_abs_true[i] = max(self.max_abs_true[i], abs(e_true))
            hn = float(self.cmd.position[0]) - tp.x
            he = float(self.cmd.position[1]) - tp.y
            self.sum_horiz_true += math.hypot(hn, he)
            self.sum_horiz_bias += math.hypot(tp.x - self.state.p_n, tp.y - self.state.p_e)
            self.sum_norm_true += err_true_norm
            self.max_norm_true = max(self.max_norm_true, err_true_norm)
            if self.terr is not None and err_true_norm > self.terr.safety_margin:
                self.n_margin_exceeded += 1
            if self.leg:
                acc = self.legs.setdefault(self.leg, LegAcc())
                rc_bad = bool(self.status is not None and self.status.rc_override)
                acc.add(xt, xt_h, xt_v, lag,
                        self.terr is not None and xt > self.terr.safety_margin,
                        rc_bad,
                        math.hypot(tp.x - self.state.p_n, tp.y - self.state.p_e),
                        abs(tp.z - self.state.p_d))
            self.sum_xt += xt
            self.sum_xt_h += xt_h
            self.sum_xt_v += xt_v
            self.sum_lag += lag
            self.max_xt = max(self.max_xt, xt)
            if self.terr is not None and xt > self.terr.safety_margin:
                self.n_xt_exceeded += 1
            self.sum_hz_cmd += hz_cmd

    # --- summary -----------------------------------------------------------------------
    def summary(self):
        print("\n" + "=" * 78)
        print("  TRACKING SUMMARY  (%d samples while flying%s)"
              % (self.n,
                 ", %d pre-liftoff discarded" % self.n_preflight if self.n_preflight else ""))
        print("=" * 78)
        if self.n == 0:
            print("  No samples with truth available.")
            if not HAVE_SIM:
                print("  rosflight_msgs SimState not importable -- hardware build?")
            else:
                print("  Is sim/truth_state publishing, and was the autopilot in control?")
            return

        n = float(self.n)
        names = ("north", "east ", "down ")
        print("  axis    |err_est|  |err_true|   |bias|   peak_true   signed_true")
        print("  " + "-" * 68)
        for i in range(3):
            signed = self.sum_signed_true[i] / n
            if i == 2:
                # NED: err = cmd - truth, so err > 0 means the command is LOWER than the
                # vehicle, i.e. the vehicle is ABOVE it. The first version of this tool had
                # this backwards and reported the opposite of what was happening.
                note = "vehicle ABOVE cmd" if signed > 0 else "vehicle BELOW cmd"
            else:
                note = ""
            print("  %s  %8.3f  %10.3f  %8.3f  %10.3f  %+10.3f  %s"
                  % (names[i], self.sum_abs_est[i] / n, self.sum_abs_true[i] / n,
                     self.sum_abs_bias[i] / n, self.max_abs_true[i], signed, note))
        print("  " + "-" * 68)

        h_true = self.sum_horiz_true / n
        h_bias = self.sum_horiz_bias / n
        d_true = self.sum_abs_true[2] / n
        d_bias = self.sum_abs_bias[2] / n
        norm_true = self.sum_norm_true / n
        margin = self.terr.safety_margin if self.terr is not None else float("nan")

        print("  horizontal:  true %.3f m   estimator bias %.3f m  (%.0f%% of it)"
              % (h_true, h_bias, 100.0 * h_bias / h_true if h_true > 1e-6 else 0.0))
        print("  vertical  :  true %.3f m   estimator bias %.3f m  (%.0f%% of it)"
              % (d_true, d_bias, 100.0 * d_bias / d_true if d_true > 1e-6 else 0.0))
        print("  3-D norm  :  mean %.3f m   peak %.3f m" % (norm_true, self.max_norm_true))
        if self.legs:
            print("")
            print("  PER LEG (cross-track = the corridor number)")
            print("   leg      n   xtrack   xt_hor   xt_ver     peak      lag   >margin   est_bias")
            print("  " + "-" * 76)
            for lg in sorted(self.legs):
                a = self.legs[lg]
                if a.n == 0:
                    continue
                k = float(a.n)
                print("  %4d %6d %8.3f %8.3f %8.3f %8.3f %8.3f %7.0f%% %10.3f"
                      % (lg, a.n, a.xt / k, a.xt_h / k, a.xt_v / k, a.max_xt, a.lag / k,
                         100.0 * a.exceeded / k, (a.bias_h + a.bias_v) / k))
            print("  " + "-" * 76)
            print("   leg   rc_override nonzero during leg   (integrators wiped while true)")
            for lg in sorted(self.legs):
                a = self.legs[lg]
                if a.n:
                    print("  %4d %28.0f%%" % (lg, 100.0 * a.rc_bad / a.n))

        print("")
        print("  CROSS-TRACK (distance to the commanded PATH -- this is the corridor number):")
        print("    mean %.3f m   peak %.3f m   horizontal %.3f m   vertical %.3f m"
              % (self.sum_xt / n, self.max_xt, self.sum_xt_h / n, self.sum_xt_v / n))
        print("    exceeds safety_margin (%.3f m) on %.0f%% of samples"
              % (margin, 100.0 * self.n_xt_exceeded / n))
        print("  ALONG-TRACK LAG (behind schedule ON the path, not off it):")
        print("    mean %.3f m   -- shows up in position error but is NOT a corridor violation"
              % (self.sum_lag / n))
        print("")
        print("  (instantaneous |cmd - truth| mean %.3f m -- mixes both, %.0f%% over margin)"
              % (norm_true, 100.0 * self.n_margin_exceeded / n))
        print("  trajectory_command received at %.1f Hz (published at 50)" % (self.sum_hz_cmd / n))
        print("")
        print("  rc_override nonzero on %d status messages." % self.n_rc_nonzero)
        if self.rc_bits_seen:
            for name, cnt in sorted(self.rc_bits_seen.items(), key=lambda kv: -kv[1]):
                print("    %-24s set %d times" % (name, cnt))
            # Only warn if the bits were set DURING a leg. Counting every nonzero status made
            # this fire on every run: ATT_SWITCH/THR_SWITCH are set pre-arm, when the RC
            # switches legitimately sit in manual, so the warning appeared while the per-leg
            # table right above it read 0% on every leg -- i.e. the tool contradicted itself
            # and cried wolf about a bug the same run had already ruled out.
            in_leg = sum(lg.rc_bad for lg in self.legs.values())
            if in_leg > 0:
                print("    NOTE: %d of these landed DURING a leg. trajectory_follower.cpp"
                      % in_leg)
                print("    clears its PID integrators whenever rc_override is nonzero --")
                print("    treating a BITFIELD as a bool -- so a steady-state offset can never")
                print("    be nulled while that holds. Only the pilot bits should clear them.")
            else:
                print("    All of these fell OUTSIDE the flight legs (pre-arm switch state);")
                print("    the per-leg table above reads 0%, so the integrators were never")
                print("    wiped in flight. No action needed.")
        print("-" * 78)

        # Attribute the DOMINANT axis, not just the vertical one.
        xt_h_mean, xt_v_mean = self.sum_xt_h / n, self.sum_xt_v / n
        if xt_h_mean >= xt_v_mean:
            dom, dom_true, dom_bias = "HORIZONTAL", h_true, h_bias
        else:
            dom, dom_true, dom_bias = "VERTICAL", d_true, d_bias
        print("  Cross-track is %s-dominated (h %.3f vs v %.3f)."
              % ("horizontally" if xt_h_mean >= xt_v_mean else "vertically", xt_h_mean, xt_v_mean))
        share = dom_bias / dom_true if dom_true > 1e-6 else 0.0

        print("  Dominant axis: %s (%.3f m of %.3f m total)" % (dom, dom_true, norm_true))
        if dom_true < 0.1:
            print("  VERDICT: tracking is good. Look elsewhere.")
        elif share > 0.5:
            print("  VERDICT: ESTIMATOR dominates (%.0f%% of the %s error)." % (share * 100, dom))
            print("           Controller gains CANNOT fix this -- the controller is already")
            print("           nulling the error it can see. Investigate the EKF on that axis.")
        elif share < 0.25:
            print("  VERDICT: CONTROL error dominates (estimator only %.0f%%)." % (share * 100))
            print("           Gains/limits are the place to look.")
        else:
            print("  VERDICT: MIXED (estimator %.0f%% of the %s error)." % (share * 100, dom))
            print("           Fix the estimator bias first, then re-measure.")
        print("=" * 78)
        if self.csv:
            print("  CSV written.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=None, help="also write a CSV here")
    ap.add_argument("--rate", type=float, default=5.0, help="print rate, Hz")
    ap.add_argument("--min-agl", type=float, default=0.5,
                    help="ignore samples below this height AGL (pre-liftoff), m")
    args = ap.parse_args()

    rclpy.init()
    node = TrackingDiagnostic(args.csv, args.rate, args.min_agl)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
