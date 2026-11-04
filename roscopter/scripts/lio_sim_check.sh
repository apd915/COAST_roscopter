#!/usr/bin/env bash
# One headless sim flight, scored against truth by lio_fusion_check.py.
#
#   ./lio_sim_check.sh <label> <use_lio true|false> <use_gnss_updates true|false> [lever_arm_frd]
#
#   ./lio_sim_check.sh baseline         false true
#   ./lio_sim_check.sh lio_gps          true  true
#   ./lio_sim_check.sh gps_denied       false false   # control: no horizontal position source
#   ./lio_sim_check.sh lio_gps_denied   true  false
#   ./lio_sim_check.sh lio_lever        true  false "[0.05, 0.0, -0.15]"
#
# Run from the workspace root after sourcing install/setup.bash. Arms through rc.py's
# toggle_arm / toggle_override services (no vimfly), so nothing needs a keyboard. RViz is
# started without a display and exits; the rest of the sim does not depend on it.
#
# Order matters: arm, wait for the bridge to align (align_settle_s, 3 s) while still on the
# ground, THEN hand over to the autopilot. Aligning in flight works but inherits more EKF error.
set -euo pipefail

LABEL="$1"; USE_LIO="$2"; USE_GNSS="$3"; LEVER="${4:-[0.0, 0.0, 0.0]}"
ENV_LEVEL="${ENV_LEVEL:-easy}"; ENV_SEED="${ENV_SEED:-0}"
# LIDAR_PIPELINE=old           mock publishes world NED from truth; planner reads it directly
# LIDAR_PIPELINE=preprocessor  mock publishes the raw sensor frame; lidar_preprocessor places it
#                              using LIDAR_POSE (estimated_state, or sim/roscopter/state = truth)
LIDAR_PIPELINE="${LIDAR_PIPELINE:-old}"; LIDAR_POSE="${LIDAR_POSE:-estimated_state}"
case "$LIDAR_PIPELINE" in
  old)          LIDAR_FRAME=ned;    USE_PRE=false; CLOUD_TOPIC=/mock_env/lidar ;;
  preprocessor) LIDAR_FRAME=sensor; USE_PRE=true;  CLOUD_TOPIC=lidar/points_ned ;;
  *) echo "LIDAR_PIPELINE must be old or preprocessor" >&2; exit 1 ;;
esac
LOGDIR="${LOGDIR:-/tmp/lio_sim_check}"
mkdir -p "$LOGDIR"

SIM_SHARE="$(ros2 pkg prefix rosflight_sim)/share/rosflight_sim"
RC_SHARE="$(ros2 pkg prefix roscopter)/share/roscopter"
SCRIPTS="$(cd "$(dirname "$0")" && pwd)"

cleanup() {
  # Launch can leave children behind when its parent dies; kill the whole process groups.
  for pid in ${SIM_PID:-} ${AP_PID:-}; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 5
  for pid in ${SIM_PID:-} ${AP_PID:-}; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT

ENVFILE="$SIM_SHARE/params/forests/forest_${ENV_LEVEL}_s${ENV_SEED}.yaml"
DISPLAY= setsid ros2 launch rosflight_sim multirotor_standalone.launch.py \
  environment_file:="$ENVFILE" lidar_output_frame:="$LIDAR_FRAME" \
  > "$LOGDIR/${LABEL}_sim.log" 2>&1 &
SIM_PID=$!

# forest_benchmark.yaml, not planner_default.yaml: the default config stalls in the forests
# ("goal UNREACHABLE", A* flood warning) independently of the estimator, which would make every
# configuration time out and leave nothing to compare.
PLANNER="${PLANNER:-$RC_SHARE/params/forest_benchmark.yaml}"
setsid ros2 launch roscopter_sim sim.launch.py \
  planner_param_file:="$PLANNER" \
  use_lidar_preprocessor:="$USE_PRE" lidar_pose_topic:="$LIDAR_POSE" \
  lio_position_noise:="${LIO_NOISE:-0.02}" lio_drift:="${LIO_DRIFT:-0.05}" \
  use_lio:="$USE_LIO" use_gnss_updates:="$USE_GNSS" lio_lever_arm_frd:="$LEVER" \
  > "$LOGDIR/${LABEL}_autopilot.log" 2>&1 &
AP_PID=$!

wait_for_service() {
  # --no-daemon: the ROS 2 CLI daemon caches discovery and can miss nodes that just started.
  for _ in $(seq 1 30); do
    ros2 service list --no-daemon 2>/dev/null | grep -qx "$1" && return 0
    sleep 2
  done
  echo "service $1 never appeared; visible services were:" >&2
  ros2 service list --no-daemon >&2 || true
  return 1
}
wait_for_service /toggle_arm
wait_for_service /path_planner/load_mission_from_file
sleep 5   # let the estimator get its GPS fix and the SIL finish starting

# Same as the manual procedure: load firmware params, calibrate the IMU, write params. Its last
# step sleeps 10 s before /param_write, so wait it out before arming.
ros2 launch rosflight_sim multirotor_init_firmware.launch.py > "$LOGDIR/${LABEL}_init_firmware.log" 2>&1

ros2 service call /path_planner/load_mission_from_file rosflight_msgs/srv/ParamFile \
  "{filename: '$RC_SHARE/params/forest_mission.yaml'}" > /dev/null

python3 "$SCRIPTS/lio_fusion_check.py" --label "$LABEL" --goal 105 0 -3 --timeout 150 \
  --env "$ENVFILE" --cloud-topic "$CLOUD_TOPIC" &
CHECK_PID=$!
sleep 2

ros2 service call /toggle_arm std_srvs/srv/Trigger > /dev/null
sleep 6   # > align_settle_s, on the ground
ros2 service call /toggle_override std_srvs/srv/Trigger > /dev/null

set +e
wait $CHECK_PID
STATUS=$?
set -e
grep -h "LIO aligned" "$LOGDIR/${LABEL}_autopilot.log" || true
grep -h "lidar_preprocessor.*scans:" "$LOGDIR/${LABEL}_autopilot.log" | tail -1 || true
exit $STATUS
