#!/usr/bin/env bash
# Single entry point for the IR100 + CR10 real-locomotive point-cloud demo.
#
#   ./run_pcd_demo.sh [extra roslaunch args]
#
# It wraps docker/run_ir100_cr10_pcd.sh and fills in recommended defaults for
# the 2.7M-point 1 cm cloud:
#   - voxel downsample at 3 cm so GridMap stays light;
#   - keep the world-frame routing on /map_generator/global_cloud.
#
# Overridable environment variables:
#   DISPLAY                    X11 display, default :0
#   REMANI_BUILD=1             force catkin_make inside the container
#   REMANI_PCD_FILE=<path>     PCD path as seen inside the container
#   REMANI_VOXEL_LEAF_SIZE=r   voxel leaf size in metres, 0 disables it
#                              (default 0.03)
#   REMANI_KILL_OLD=1          remove a previous demo container first
#
# Any extra arguments are forwarded to roslaunch, for example:
#   ./run_pcd_demo.sh twc_tx:=-8.0 twc_yaw:=0.3 voxel_leaf_size:=0.05
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${REPO_ROOT}"

export DISPLAY="${DISPLAY:-:0}"
export REMANI_VOXEL_LEAF_SIZE="${REMANI_VOXEL_LEAF_SIZE:-0.03}"

if [[ "${REMANI_KILL_OLD:-0}" == "1" ]]; then
  echo "[run_pcd_demo] stopping previous demo container (if any)"
  docker rm -f remani_pcd_demo >/dev/null 2>&1 || true
fi

echo "[run_pcd_demo] container PCD  : ${REMANI_PCD_FILE:-<default 1 cm PCD>}"
echo "[run_pcd_demo] voxel leaf     : ${REMANI_VOXEL_LEAF_SIZE} m"
echo "[run_pcd_demo] DISPLAY        : ${DISPLAY}"
echo "[run_pcd_demo] routing topic  : /map_generator/global_cloud"
echo "[run_pcd_demo] starting ..."

exec "${REPO_ROOT}/docker/run_ir100_cr10_pcd.sh" "$@"
