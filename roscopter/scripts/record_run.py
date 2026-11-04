#!/usr/bin/env python3
"""Parse one benchmark run's console output into a CSV row, and aggregate cells.

WHY THIS EXISTS: each run prints ~120 lines across two summaries, and NOTHING in them
names the environment or the speed. Transcribing 90 of those by hand is both slow and a
silent-error machine -- a mistyped T_trav looks exactly like a real one. This reads the
text the run already printed and records it verbatim.

RECORD every run, including failures. R_succ is passes/attempts, so a ledger containing
only the good runs reports 100% success no matter what actually happened.

  # paste the run output into a file, or pipe it
  ./record_run.py hard 1 8 < run.log
  ./record_run.py hard 1 8 --log benchmark_results/raw/hard_s1_v8.log --note "run 3 of 3"

  # a run that stalled and printed no summary
  ./record_run.py hard 1 8 --stall --note "45 s stall at x=80.7"

  # the paper table
  ./record_run.py --summary
"""
import argparse, csv, os, re, statistics as st, sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
CSV = os.path.join(ROOT, "benchmark_results", "runs.csv")

FIELDS = [
    "level", "seed", "v_max", "verdict", "collisions", "contact_s",
    "T_trav", "L_path", "S_jerk", "v_inf", "a_inf", "rho_vel", "rho_acc",
    "v_avg", "v_peak", "a_peak", "jerk_rms",
    "replans", "plan_min", "plan_mean", "plan_max",
    "worst_frontend", "worst_osqp", "worst_tries", "worst_sfc", "worst_cps",
    "gate_judged", "gate_declined_pct", "gate_forced", "gate_streak",
    "div_p50", "div_p90", "div_p99",
    "churn_nearfield_pct", "churn_episodes",
    "xtrack_mean", "xtrack_peak", "xtrack_hor", "over_margin_pct", "lag_mean",
    "est_bias_hor", "err3d_mean", "cmd_hz", "note",
]


def num(m, i=1):
    return float(m.group(i)) if m else ""


def parse(text):
    # Strip the ros2 launch prefix; the tracking summary has no prefix, the planner one does.
    lines = [re.sub(r"^\[[^\]]+\]\s?", "", ln) for ln in text.splitlines()]
    t = "\n".join(lines)
    r = {}

    m = re.search(r"RUN VERDICT:\s*(?:\*+\s*)?(PASS|FAIL)", t)
    r["verdict"] = m.group(1) if m else ""
    m = re.search(r"(\d+)\s+collision event\(s\),\s*([\d.]+)\s*s in contact", t)
    r["collisions"], r["contact_s"] = (int(m.group(1)), float(m.group(2))) if m else (0, 0.0)

    m = re.search(r"^\s*ALL\s+(\d+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s*$", t, re.M)
    if m:
        r["replans"], r["plan_min"] = int(m.group(1)), float(m.group(2))
        r["plan_mean"], r["plan_max"] = float(m.group(3)), float(m.group(4))

    # Worst cycle: 8 numbers, the only row shaped like this.
    m = re.search(r"^\s*\d+\s+[\d.]+\s+([\d.]+)\s+[\d.]+\s+([\d.]+)\s+[\d.]+\s+(\d+)\s+(\d+)\s+(\d+)\s*$",
                  t, re.M)
    if m:
        r["worst_frontend"], r["worst_osqp"] = float(m.group(1)), float(m.group(2))
        r["worst_tries"] = int(m.group(3))
        r["worst_sfc"], r["worst_cps"] = int(m.group(4)), int(m.group(5))

    # Two different rows start with PLN. Tell them apart by column count:
    # BY GENERATOR has 8 numbers, BENCHMARK METRICS has 7.
    for m in re.finditer(r"^\s*PLN\s+((?:[\d.]+\s+){6,7}[\d.]+)\s*$", t, re.M):
        v = [float(x) for x in m.group(1).split()]
        if len(v) == 8:
            r["v_avg"], r["v_peak"], r["a_peak"], r["jerk_rms"] = v[2], v[3], v[4], v[5]
        elif len(v) == 7:
            (r["T_trav"], r["L_path"], r["S_jerk"], r["v_inf"], r["a_inf"],
             r["rho_vel"], r["rho_acc"]) = v

    m = re.search(r"(\d+) judged, (\d+) declined \(([\d.]+)%\)", t)
    if m:
        r["gate_judged"], r["gate_declined_pct"] = int(m.group(1)), float(m.group(3))
    r["gate_forced"] = num(re.search(r"forced accepts (\d+)", t))
    r["gate_streak"] = num(re.search(r"longest streak (\d+)", t))
    m = re.search(r"ALL candidates: p50 ([\d.]+)\s+p90 ([\d.]+)\s+p99 ([\d.]+) deg", t)
    if m:
        r["div_p50"], r["div_p90"], r["div_p99"] = (float(m.group(i)) for i in (1, 2, 3))

    r["churn_nearfield_pct"] = num(
        re.search(r"changed within [\d.]+ m of the vehicle ([\d.]+)%", t))
    r["churn_episodes"] = num(re.search(r"flip-flopping\): (\d+)", t))

    m = re.search(r"mean ([\d.]+) m\s+peak ([\d.]+) m\s+horizontal ([\d.]+) m\s+vertical ([\d.]+) m", t)
    if m:
        r["xtrack_mean"], r["xtrack_peak"] = float(m.group(1)), float(m.group(2))
        r["xtrack_hor"] = float(m.group(3))
    r["over_margin_pct"] = num(
        re.search(r"exceeds safety_margin \([\d.]+ m\) on (\d+)% of samples", t))
    m = re.search(r"ALONG-TRACK LAG.*?\n\s*mean ([\d.]+) m", t, re.S)
    r["lag_mean"] = num(m)
    r["est_bias_hor"] = num(
        re.search(r"horizontal:\s+true [\d.]+ m\s+estimator bias ([\d.]+) m", t))
    r["err3d_mean"] = num(re.search(r"3-D norm\s*:\s*mean ([\d.]+) m", t))
    r["cmd_hz"] = num(re.search(r"trajectory_command received at ([\d.]+) Hz", t))
    return r


def read_rows():
    if not os.path.exists(CSV):
        return []
    with open(CSV) as f:
        return list(csv.DictReader(f))


def summary():
    rows = read_rows()
    if not rows:
        print("no runs recorded yet")
        return
    cells = {}
    for row in rows:
        cells.setdefault((row["level"], row["v_max"]), []).append(row)

    def agg(vals):
        v = [float(x) for x in vals if x not in ("", None)]
        if not v:
            return "--"
        if len(v) == 1:
            return "%.2f" % v[0]
        return "%.2f+/-%.2f" % (st.mean(v), st.stdev(v))

    print("PAPER TABLE -- metrics over PASSING runs only; R_succ over ALL recorded attempts.")
    print("(a collision run flies THROUGH obstacles, so its T_trav/L_path are optimistic and")
    print(" must not be averaged with clean runs -- see the RUN VERDICT text.)\n")
    hdr = ("level", "v_max", "n", "R_succ", "T_trav[s]", "L_path[m]", "S_jerk",
           "plan_mean[ms]", "plan_max[ms]")
    print("  %-7s %5s %4s %7s  %14s %14s %14s %14s %12s" % hdr)
    for (lvl, v) in sorted(cells, key=lambda k: (k[0], float(k[1]))):
        rs = cells[(lvl, v)]
        ok = [x for x in rs if x["verdict"] == "PASS"]
        print("  %-7s %5s %4d %6.0f%%  %14s %14s %14s %14s %12s" % (
            lvl, v, len(rs), 100.0 * len(ok) / len(rs),
            agg([x["T_trav"] for x in ok]), agg([x["L_path"] for x in ok]),
            agg([x["S_jerk"] for x in ok]), agg([x["plan_mean"] for x in ok]),
            agg([x["plan_max"] for x in ok])))
    print("\n  %d run(s) recorded, %d passing." % (
        len(rows), sum(1 for x in rows if x["verdict"] == "PASS")))
    missing = [(l, s, v) for l in ("easy", "medium", "hard") for s in range(10)
               for v in (2, 5, 8)
               if not any(x["level"] == l and x["seed"] == str(s) and x["v_max"] == str(v)
                          for x in rows)]
    print("  %d of 90 cells still empty." % len(missing))
    if missing and len(missing) <= 12:
        print("   " + ", ".join("%s/s%d/v%d" % m for m in missing))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("level", nargs="?", choices=["easy", "medium", "hard"])
    p.add_argument("seed", nargs="?", type=int)
    p.add_argument("v_max", nargs="?", type=int, choices=[2, 5, 8])
    p.add_argument("--log", help="file with the run output (default: stdin)")
    p.add_argument("--note", default="")
    p.add_argument("--stall", action="store_true",
                   help="run stalled and printed no summary; record as FAIL with no metrics")
    p.add_argument("--summary", action="store_true")
    a = p.parse_args()

    if a.summary:
        summary()
        return
    if a.level is None or a.seed is None or a.v_max is None:
        p.error("level, seed and v_max are required unless --summary")

    if a.stall:
        r = {"verdict": "FAIL", "collisions": 0, "contact_s": 0.0}
    else:
        text = open(a.log).read() if a.log else sys.stdin.read()
        r = parse(text)
        if not r.get("verdict"):
            sys.exit("could not find 'RUN VERDICT' -- is this the full run output? "
                     "(use --stall for a run that printed no summary)")
        if r.get("T_trav") in ("", None):
            sys.exit("found a verdict but no PLN benchmark row -- output looks truncated")

    r.update(level=a.level, seed=a.seed, v_max=a.v_max, note=a.note)
    row = {k: r.get(k, "") for k in FIELDS}

    os.makedirs(os.path.dirname(CSV), exist_ok=True)
    new = not os.path.exists(CSV)
    with open(CSV, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow(row)
    print("recorded %s/s%d/v%d -> %s  (T_trav %s, L_path %s)" % (
        a.level, a.seed, a.v_max, r["verdict"], row["T_trav"] or "--", row["L_path"] or "--"))
    print("  %s" % CSV)


if __name__ == "__main__":
    main()
