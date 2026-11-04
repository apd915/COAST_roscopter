#!/usr/bin/env python3
"""Summarise why a run stalled, from the planner's captured console output.

  stdbuf -oL -eL ros2 launch roscopter_sim sim.launch.py ... 2>&1 | tee run.log
  ./stall_report.py run.log

The decisive line is front_end.cpp's "short path" diagnostic, printed whenever A* returns
3 cells or fewer -- too short to be a real plan. It labels the START and GOAL cells, and that
pair separates causes that look identical from outside:

  start INFLATED-BLOCKED  the planner believes the vehicle is INSIDE an obstacle. Every
                          neighbour is rejected, so it cannot move however open the field
                          looks. Estimator bias larger than the inflation radius does this.
  goal blocked/unknown    the horizon sub-goal landed somewhere unreachable; the search spent
                          its budget probing a dead end.
  both free               geometry is fine and the search still failed -- look at
                          astar_max_nodes / astar_max_time_ms.
"""
import collections, re, sys

SHORT = re.compile(r"short path: start idx \[\s*([-\d ]+?)\s*\] \((\S+)\) -> goal idx \[\s*([-\d ]+?)\s*\] \((\S+?)\),\s*([\d.]+) m apart")
SEARCH = re.compile(r"A\* search ([\d.]+) ms \((\d+) cells\)")
STALL = re.compile(r"STALL DECLARED: ([\d.]+) m from the waypoint for ([\d.]+) s")
NOWHERE = re.compile(r"Plan went nowhere \(([\d.]+) m of travel, goal ([\d.]+) m away\)")
SNAP = re.compile(r"Stall snapshot written to (\S+)")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    text = open(sys.argv[1], errors="replace").read()

    pairs = collections.Counter()
    starts, cells, stalls, nowhere, snaps = [], [], [], [], []
    for m in SHORT.finditer(text):
        pairs[(m.group(2), m.group(4))] += 1
        starts.append((m.group(1), m.group(2)))
    for m in SEARCH.finditer(text):
        cells.append(int(m.group(2)))
    for m in STALL.finditer(text):
        stalls.append((float(m.group(1)), float(m.group(2))))
    for m in NOWHERE.finditer(text):
        nowhere.append((float(m.group(1)), float(m.group(2))))
    for m in SNAP.finditer(text):
        snaps.append(m.group(1))

    print("=" * 72)
    print("  STALL REPORT -- %s" % sys.argv[1])
    print("=" * 72)

    if cells:
        short = sum(1 for c in cells if c <= 3)
        cs = sorted(cells)
        print("\nA* PATH LENGTH over %d searches" % len(cells))
        print("  p10 %d   med %d   p90 %d   max %d cells" % (
            cs[len(cs) // 10], cs[len(cs) // 2], cs[9 * len(cs) // 10], cs[-1]))
        print("  searches returning <=3 cells (too short to fly): %d (%.1f%%)"
              % (short, 100.0 * short / len(cells)))
    else:
        print("\nNo '[Front-End] A* search' lines found.")
        print("  Either the log is not terminal 2, or stdout was buffered -- rerun with")
        print("  'stdbuf -oL -eL ros2 launch ...' so the library's cout is line-buffered.")

    if pairs:
        print("\nWHY THE PATH WAS SHORT  (start state -> goal state, %d occurrences)"
              % sum(pairs.values()))
        for (s, g), n in pairs.most_common():
            print("  %-18s -> %-18s  %5d" % (s, g, n))
        top = pairs.most_common(1)[0][0]
        print("\n  VERDICT: ", end="")
        if top[0] == "INFLATED-BLOCKED":
            print("START is inside inflated space.")
            print("  The planner thinks the vehicle is in an obstacle, so every neighbour is")
            print("  rejected and it cannot move -- this is the 'stalled in an open field' case.")
            print("  Check estimator bias against the inflation radius (hull + obstacle_margin);")
            print("  bias larger than that radius puts the believed position inside a trunk.")
        elif top[1] in ("INFLATED-BLOCKED", "unknown"):
            print("GOAL is %s." % top[1])
            print("  The horizon sub-goal is unreachable, so the search burns its budget probing")
            print("  a dead end and returns a stub. Look at planning_horizon and the two-pass")
            print("  soft-obstacle sub-goal (astar_soft_obstacle_cost).")
        else:
            print("both endpoints are free and the search still failed.")
            print("  Geometry is not the problem -- suspect astar_max_nodes (4000) or")
            print("  astar_max_time_ms (20.0) running out before a route is found.")
    else:
        print("\nNo short-path diagnostics -- A* always returned a usable path.")
        print("  The stalls are NOT the search. Look downstream: the QP, the allocation")
        print("  (see 'Plan went nowhere' below), or the directional gate.")

    if nowhere:
        t = [a for a, _ in nowhere]
        print("\nDEGENERATE PLANS (solved, but went nowhere): %d" % len(nowhere))
        print("  net travel min %.2f  max %.2f m; goal was %.0f-%.0f m away"
              % (min(t), max(t), min(b for _, b in nowhere), max(b for _, b in nowhere)))

    if stalls:
        print("\nSTALLS DECLARED: %d" % len(stalls))
        for d, s in stalls:
            print("  %6.2f m from the waypoint, stopped %.0f s" % (d, s))
    else:
        print("\nNo 'STALL DECLARED' lines -- either it never stalled for 45 s, or it was")
        print("  creeping fast enough to stay under stall_speed and never tripped the clock.")

    if snaps:
        print("\nSNAPSHOTS (replay with planner_reference/cpp/build/replay_stall):")
        for s in snaps:
            print("  %s" % s)
    print()


if __name__ == "__main__":
    main()
