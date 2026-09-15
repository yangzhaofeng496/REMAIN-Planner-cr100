#!/usr/bin/env bash
# Send a 2D goal and, whenever planning fails three times in a row, raise the
# static locomotive cloud by 0.1 m and retry, until planning succeeds.
#
#   ./run_goal_auto_height.sh [goal_x goal_y goal_yaw]
#
# Defaults match the tested far goal.  Environment overrides:
#   REMANI_CONTAINER_NAME  default remani_pcd_demo
#   REMANI_LOG             roslaunch stdout log, default /tmp/opencode/run_pcd_demo.log
#   REMANI_HEIGHT_STEP     default 0.1
#   REMANI_HEIGHT_MAX      default 3.0
set -uo pipefail

CONTAINER="${REMANI_CONTAINER_NAME:-remani_pcd_demo}"
LOG="${REMANI_LOG:-/tmp/opencode/run_pcd_demo.log}"
STEP="${REMANI_HEIGHT_STEP:-0.1}"
MAX_Z="${REMANI_HEIGHT_MAX:-3.0}"

GOAL_X="${1:-24.310}"
GOAL_Y="${2:-0.9304}"
GOAL_YAW="${3:-0.038}"

# Cloud placement keeps the saved XY/yaw; only z changes.
CLOUD_X="3.9905"
CLOUD_Y="-0.111"
CLOUD_YAW="-1.5329"

ros() { docker exec "${CONTAINER}" bash -lc "source /opt/remani_ws/devel/setup.bash 2>/dev/null; $*"; }

get_cloud_z() {
  ros "timeout 8 rostopic echo -n1 /static_pcd_publisher/current_pose/pose/position/z 2>/dev/null" \
    | grep -oE -- '-?[0-9]+(\.[0-9]+)?' | head -1
}

set_cloud_z() {
  local z="$1"
  local qz qw
  qz=$(python3 -c "import math;print(math.sin(${CLOUD_YAW}/2))")
  qw=$(python3 -c "import math;print(math.cos(${CLOUD_YAW}/2))")
  ros "timeout 8 rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
      '{header: {frame_id: world}, pose: {pose: {position: {x: ${CLOUD_X}, y: ${CLOUD_Y}, z: ${z}}, orientation: {z: ${qz}, w: ${qw}}}}}'" >/dev/null 2>&1
}

send_goal() {
  local qz qw
  qz=$(python3 -c "import math;print(math.sin(${GOAL_YAW}/2))")
  qw=$(python3 -c "import math;print(math.cos(${GOAL_YAW}/2))")
  ros "timeout 8 rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
      '{header: {frame_id: world}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}, orientation: {z: ${qz}, w: ${qw}}}}'" >/dev/null 2>&1
}

z="$(get_cloud_z)"
if [[ -z "${z}" ]]; then
  echo "run_goal_auto_height: could not read cloud z" >&2
  exit 1
fi
printf 'run_goal_auto_height: start z=%s goal=(%s, %s, %s)\n' "${z}" "${GOAL_X}" "${GOAL_Y}" "${GOAL_YAW}"

while :; do
  start_line=$(wc -l < "${LOG}")
  sleep 2
  send_goal
  printf 'run_goal_auto_height: sent goal at z=%s, waiting for outcome ...\n' "${z}"

  # Tie the outcome to the goal we just sent, ignoring stale log lines.
  goal_line=""
  for _ in $(seq 1 60); do
    sleep 1
    goal_line="$(grep -nE "new goal: x=$(printf '%.3f' "${GOAL_X}")" "${LOG}" \
                 | tail -1 | cut -d: -f1)"
    if [[ -n "${goal_line}" && "${goal_line}" -gt "${start_line}" ]]; then break; fi
    goal_line=""
  done

  outcome=""
  if [[ -n "${goal_line}" ]]; then
    for _ in $(seq 1 240); do
      sleep 1
      new="$(tail -n +"$((goal_line + 1))" "${LOG}")"
      if grep -q "reach goal" <<<"${new}"; then outcome="success"; break; fi
      if grep -q "planning failed after 3 attempts" <<<"${new}"; then outcome="fail"; break; fi
    done
  fi

  if [[ "${outcome}" == "success" ]]; then
    printf 'run_goal_auto_height: planning succeeded at z=%s\n' "${z}"
    exit 0
  fi
  if [[ "${outcome}" != "fail" ]]; then
    printf 'run_goal_auto_height: timed out waiting for an outcome at z=%s\n' "${z}" >&2
    exit 1
  fi

  z="$(python3 -c "print(round(${z} + ${STEP}, 3))")"
  if python3 -c "import sys;sys.exit(0 if ${z} > ${MAX_Z} else 1)"; then
    echo "run_goal_auto_height: reached max z=${MAX_Z} without success" >&2
    exit 1
  fi
  printf 'run_goal_auto_height: planning failed, raising cloud to z=%s\n' "${z}"
  set_cloud_z "${z}"
  sleep 4
done
