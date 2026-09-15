#!/usr/bin/env bash
# Watch the REMANI-Planner planning state (FSM, collisions, trajectory, goals).
#
#   ./monitor_planner.sh
#
# Requires the demo container to be running. Ctrl-C to stop.
set -euo pipefail

CONTAINER="${REMANI_CONTAINER_NAME:-remani_pcd_demo}"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "monitor_planner: container ${CONTAINER} is not running" >&2
  exit 1
fi

docker exec -it "${CONTAINER}" bash -lc '
source /opt/remani_ws/devel/setup.bash
exec python3 -u /opt/remani_ws/src/REMANI-Planner/remani_planner/plan_manage/scripts/planner_monitor.py'
