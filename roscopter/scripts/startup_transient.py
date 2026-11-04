#!/usr/bin/env python3
"""Where does the vehicle first leave the commanded trajectory?

WHY THIS EXISTS. The vehicle departs the first waypoint climbing and running ahead, and the
first 10-25 s are spent recovering. Six hypotheses have been proposed and measured wrong
(takeoff gate, route-gate defence, thrust/tilt coupling, settle-gate regression, corridor
chain growth, A* geofence). Every one of them was reasoning about aggregate statistics.

This reads the per-sample timeline instead and answers three questions directly:

  WHEN does command and truth first diverge, per axis?
  Is the command itself moving (planner asked for it) or flat (vehicle left on its own)?
  Does the estimate track truth, or does the vehicle fly the estimate onto a wrong command?

Usage:
    python3 src/roscopter/roscopter/scripts/tracking_diagnostic.py --csv /tmp/track.csv --rate 20
    # fly the first ~40 s, Ctrl-C
    python3 src/roscopter/roscopter/scripts/startup_transient.py /tmp/track.csv
"""
import csv
import math
import sys

# Divergence threshold: half the corridor clearance (hull + margin + half a voxel ~ 0.94 m),
# so it trips well before the geometric guarantee is at risk.
THRESH = 0.47


def f(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def main(path, window=40.0):
    with open(path) as fh:
        rows = [r for r in csv.DictReader(fh)]
    if not rows:
        print("empty csv")
        return
    if "cmd_n" not in rows[0]:
        print("This CSV is the old vertical-only format. Re-record with the updated\n"
              "tracking_diagnostic.py to get the north/east channels.")
        return

    rows = [r for r in rows if f(r, "t") <= window]
    print(f"{len(rows)} samples over the first {window:.0f} s\n")

    # First divergence per axis, and whether the COMMAND was moving when it happened.
    for axis, c, t in (("north", "cmd_n", "truth_n"),
                       ("east", "cmd_e", "truth_e"),
                       ("down", "cmd_d", "truth_d")):
        first = None
        for i, r in enumerate(rows):
            e = f(r, c) - f(r, t)
            if not math.isnan(e) and abs(e) > THRESH:
                first = (i, r, e)
                break
        if first is None:
            print(f"  {axis:<6} never diverged more than {THRESH:.2f} m")
            continue
        i, r, e = first
        # Command rate over the half second before divergence, to say whether the planner was
        # asking for the motion or the vehicle produced it.
        j = max(0, i - int(0.5 / max(1e-3, f(rows[1], "t") - f(rows[0], "t"))))
        dc = f(r, c) - f(rows[j], c)
        dt = f(r, "t") - f(rows[j], "t")
        rate = dc / dt if dt > 1e-6 else 0.0
        print(f"  {axis:<6} first diverged {e:+.2f} m at t = {f(r,'t'):5.2f} s | "
              f"command was moving {rate:+.2f} m/s at that moment")

    # Did the vehicle fly the ESTIMATE onto the command while truth went elsewhere?
    print()
    worst = max(rows, key=lambda r: abs(f(r, "est_bias_d")) if not math.isnan(f(r, "est_bias_d"))
                else -1)
    print(f"  worst vertical estimator bias in window: {f(worst,'est_bias_d'):+.2f} m "
          f"at t = {f(worst,'t'):.2f} s")
    print(f"    at that moment: cmd_d {f(worst,'cmd_d'):+.2f}  est_d {f(worst,'est_d'):+.2f}  "
          f"truth_d {f(worst,'truth_d'):+.2f}")
    print("    est tracking cmd while truth does not => the vehicle is obeying a wrong estimate,")
    print("    not disobeying a good command. That is an ESTIMATOR fault, not control.")

    # Compact timeline so the shape is visible without plotting.
    print(f"\n{'t':>6} {'cmd_d':>7} {'truth_d':>8} {'err_d':>7} {'cmd_n':>8} {'truth_n':>8} "
          f"{'err_n':>7} {'thrust':>7}")
    step = max(1, len(rows) // 30)
    for r in rows[::step]:
        print(f"{f(r,'t'):>6.1f} {f(r,'cmd_d'):>7.2f} {f(r,'truth_d'):>8.2f} "
              f"{f(r,'cmd_d')-f(r,'truth_d'):>7.2f} {f(r,'cmd_n'):>8.2f} {f(r,'truth_n'):>8.2f} "
              f"{f(r,'cmd_n')-f(r,'truth_n'):>7.2f} {f(r,'thrust'):>7.2f}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 40.0)
