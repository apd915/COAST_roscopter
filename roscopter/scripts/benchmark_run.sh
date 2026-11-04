#!/usr/bin/env bash
# Print the exact commands for one benchmark run, or the whole 90-run running order.
#
# WHY THIS EXISTS: the sweep is 3 densities x 10 seeds x 3 speeds across three terminals, and
# NOTHING in any summary names the environment or the speed it was flown at. A mistyped
# environment_file or a stale v_max produces a run that looks completely normal and is silently
# unusable. This resolves every path from `ros2 pkg prefix` and REFUSES to print run commands
# unless the installed v_max matches the one asked for.
#
#   ./benchmark_run.sh                 # the full 90-run running order
#   ./benchmark_run.sh hard 0 8        # the three terminals for one run, at v_max 8
#
# PARAMS ARE READ FROM install/, NOT src/. After editing any YAML under
# src/roscopter/roscopter/params/, `colcon build --packages-select roscopter` or it does nothing.
set -euo pipefail

SIM_SHARE="$(ros2 pkg prefix rosflight_sim)/share/rosflight_sim"
RC_SHARE="$(ros2 pkg prefix roscopter)/share/roscopter"
PLANNER="$RC_SHARE/params/forest_benchmark.yaml"
MISSION="$RC_SHARE/params/forest_mission.yaml"
SRC_PLANNER="src/roscopter/roscopter/params/forest_benchmark.yaml"

installed() { grep -m1 -E "^[[:space:]]*$1:" "$PLANNER" | awk '{print $2}'; }

if [[ $# -eq 0 ]]; then
  echo "90-run order -- speed outer, density next, seed inner (v_max 8 on hard first):"
  n=1
  for v in 8 5 2; do
    echo
    echo "  ===== v_max $v  (a_max 20) -- set it once, then run all 30 ====="
    echo "  sed -i 's/^    v_max: .*/    v_max: ${v}.0/' $SRC_PLANNER"
    echo "  colcon build --packages-select roscopter"
    for lvl in hard medium easy; do
      for s in 0 1 2 3 4 5 6 7 8 9; do
        printf "   %2d. %-6s seed %d  ->  ./benchmark_run.sh %s %d %s\n" "$n" "$lvl" "$s" "$lvl" "$s" "$v"
        n=$((n+1))
      done
    done
  done
  echo
  echo "Record per run: RUN VERDICT, T_trav, L_path, S_jerk, v_inf, a_inf, rho_vel, rho_acc,"
  echo "mean/max replan, and >margin. Report mean +/- std over the 10 seeds, never a single value."
  exit 0
fi

[[ $# -eq 3 ]] || { echo "usage: $0 <easy|medium|hard> <0-9> <2|5|8>   (no args = running order)" >&2; exit 1; }
LVL="$1"; SEED="$2"; VREQ="$3"
case "$LVL" in easy|medium|hard) ;; *) echo "level must be easy|medium|hard" >&2; exit 1;; esac
[[ "$SEED" =~ ^[0-9]$ ]] || { echo "seed must be 0-9" >&2; exit 1; }
case "$VREQ" in 2|5|8) ;; *) echo "v_max must be 2, 5 or 8" >&2; exit 1;; esac

ENVFILE="$SIM_SHARE/params/forests/forest_${LVL}_s${SEED}.yaml"
[[ -f "$ENVFILE" ]] || { echo "missing: $ENVFILE" >&2; exit 1; }

# The guard. A stale v_max is invisible in every summary the run produces, so refuse rather
# than print commands that would waste the run.
VNOW="$(installed v_max)"
if [[ "$VNOW" != "${VREQ}.0" && "$VNOW" != "$VREQ" ]]; then
  echo "REFUSING: installed v_max is $VNOW but you asked for $VREQ." >&2
  echo "Nothing in the run output would tell you. Fix it with:" >&2
  echo "  sed -i 's/^    v_max: .*/    v_max: ${VREQ}.0/' $SRC_PLANNER" >&2
  echo "  colcon build --packages-select roscopter" >&2
  exit 1
fi

echo "=============================================================================="
echo "  RUN: $LVL  seed $SEED   v_max $VNOW   a_max $(installed a_max)"
echo "=============================================================================="
echo "  environment : $ENVFILE"
{ grep -m1 "route exists" "$ENVFILE" || echo "#   route exists: UNKNOWN"; } | sed 's/^# */  traversable: /'
{ grep -m1 "trunks" "$ENVFILE" || echo "#   trunks: UNKNOWN"; } | sed 's/^# */  density    : /'
echo "  planner     : max_sfc_count $(installed max_sfc_count)   obstacle_margin $(installed obstacle_margin)"
echo
echo "--- TERMINAL 1: simulator + obstacle world -----------------------------------"
echo "source install/setup.bash"
echo "ros2 launch rosflight_sim multirotor_standalone.launch.py \\"
echo "  use_vimfly:=true \\"
echo "  environment_file:=$ENVFILE"
echo
echo "--- TERMINAL 2: autopilot + planner ------------------------------------------"
echo "source install/setup.bash"
echo "ros2 launch roscopter_sim sim.launch.py \\"
echo "  planner_param_file:=$PLANNER"
echo
echo "--- TERMINAL 3: mission, then tracking ---------------------------------------"
echo "source install/setup.bash"
echo "ros2 service call /path_planner/load_mission_from_file \\"
echo "  rosflight_msgs/srv/ParamFile \"{filename: '$MISSION'}\""
echo "python3 src/roscopter/roscopter/scripts/tracking_diagnostic.py"
echo
echo "  Then arm in the vimfly window. Ctrl-C terminal 3 for the tracking summary;"
echo "  the planner summary prints on waypoint arrival in terminal 2."
echo "=============================================================================="
