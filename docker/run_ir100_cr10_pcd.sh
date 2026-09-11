#!/usr/bin/env bash
# Launch the IR100 + CR10 real-PCD scenario inside Docker.
#
# All command-line arguments are forwarded to roslaunch, e.g.
#   ./docker/run_ir100_cr10_pcd.sh voxel_leaf_size:=0.05 twc_tx:=-8.0
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

IMAGE_NAME="${REMANI_IMAGE:-remani-planner:noetic}"
CONTAINER_NAME="${REMANI_CONTAINER_NAME:-remani_pcd_demo}"
PCD_FILE="${REMANI_PCD_FILE:-/opt/remani_ws/src/REMANI-Planner/scans_bottom_xyzRemove_filtered/scansdibu_xyzRemove_filtered_1cm.pcd}"
VOXEL_LEAF_SIZE="${REMANI_VOXEL_LEAF_SIZE:-0.0}"
ROSLAUNCH_ARGS=("$@")

xhost +local:root
trap 'xhost -local:root >/dev/null 2>&1 || true' EXIT

docker run --rm -i \
  --name "${CONTAINER_NAME}" \
  --net=host \
  --ipc=host \
  -e DISPLAY="${DISPLAY:-}" \
  -e DISABLE_ROS1_EOL_WARNINGS=1 \
  -e REMANI_BUILD="${REMANI_BUILD:-0}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${REPO_ROOT}:/opt/remani_ws/src/REMANI-Planner:rw" \
  -v remani_planner_build:/opt/remani_ws/build \
  -v remani_planner_devel:/opt/remani_ws/devel \
  "${IMAGE_NAME}" \
  bash -lc "cd /opt/remani_ws && \
    if [ \"\${REMANI_BUILD:-0}\" = 1 ] || [ ! -x /opt/remani_ws/devel/lib/remani_planner/remani_planner_node ]; then \
      catkin_make -DCMAKE_BUILD_TYPE=Release; \
    fi && \
    source /opt/remani_ws/devel/setup.bash && \
    export ROS_PACKAGE_PATH=/opt/remani_ws/src/REMANI-Planner/local_robot:/opt/remani_ws/src/REMANI-Planner/TCP-IP-ROS-6AXis:\${ROS_PACKAGE_PATH:-} && \
    roslaunch remani_planner exp0_ir100_cr10_pcd.launch pcd_file:='${PCD_FILE}' voxel_leaf_size:='${VOXEL_LEAF_SIZE}' ${ROSLAUNCH_ARGS[*]:-}"
