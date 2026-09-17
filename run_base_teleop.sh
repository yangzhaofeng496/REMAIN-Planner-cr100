#!/usr/bin/env bash
# Keyboard base + arm teleop with live collision detection.
#
#   ./run_base_teleop.sh
#
# Base: W/S move, A/D rotate.  Arm: 1..6 nudge the matching joint by one
# step, Space flips the nudge direction.  Q quits.  If the demo container is
# not running it is started first (set REMANI_AUTOSTART=0 to disable).  Stops
# /mm_controller_node so it does not override the manual commands.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${REMANI_CONTAINER_NAME:-remani_pcd_demo}"
MASTER_URI="${REMANI_MASTER_URI:-${ROS_MASTER_URI:-http://127.0.0.1:11312}}"

running() {
  docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"
}

if ! running; then
  if [[ "${REMANI_AUTOSTART:-1}" != "1" ]]; then
    echo "run_base_teleop: container ${CONTAINER} is not running (REMANI_AUTOSTART=0)" >&2
    exit 1
  fi
  echo "[run_base_teleop] starting demo container ..."
  ( cd "${REPO_ROOT}" && setsid ./run_pcd_demo.sh >/tmp/opencode/run_pcd_demo.log 2>&1 </dev/null & )
  for _ in $(seq 1 40); do
    sleep 2
    if running && docker exec -e ROS_MASTER_URI="${MASTER_URI}" "${CONTAINER}" bash -lc 'source /opt/remani_ws/devel/setup.bash 2>/dev/null; rostopic list 2>/dev/null | grep -q collision_type' 2>/dev/null; then
      break
    fi
  done
  if ! running; then
    echo "run_base_teleop: failed to start ${CONTAINER}, see /tmp/opencode/run_pcd_demo.log" >&2
    exit 1
  fi
  echo "[run_base_teleop] waiting for the point cloud to load ..."
  for _ in $(seq 1 30); do
    sleep 2
    if docker exec -e ROS_MASTER_URI="${MASTER_URI}" "${CONTAINER}" bash -lc 'source /opt/remani_ws/devel/setup.bash; timeout 5 rostopic echo -n1 /map_generator/global_cloud/width 2>/dev/null | grep -qE "^[0-9]+$"'; then
      break
    fi
  done
fi

docker exec -it -e ROS_MASTER_URI="${MASTER_URI}" "${CONTAINER}" bash -lc '
source /opt/remani_ws/devel/setup.bash
exec python3 /opt/remani_ws/src/REMANI-Planner/remani_planner/plan_manage/scripts/base_teleop_collision.py'
