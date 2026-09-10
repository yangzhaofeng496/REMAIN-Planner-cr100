#!/bin/bash
set -euo pipefail

IMAGE_NAME="${1:-remani-planner:noetic}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

xhost +local:root
trap 'xhost -local:root >/dev/null 2>&1 || true' EXIT

docker run --rm -i \
  --net=host \
  --ipc=host \
  -e DISPLAY="${DISPLAY}" \
  -e DISABLE_ROS1_EOL_WARNINGS=1 \
  -e REMANI_BUILD="${REMANI_BUILD:-0}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${REPO_ROOT}:/opt/remani_ws/src/REMANI-Planner:rw" \
  -v remani_planner_build:/opt/remani_ws/build \
  -v remani_planner_devel:/opt/remani_ws/devel \
  "${IMAGE_NAME}" \
  bash -lc "cd /opt/remani_ws && if [ \"\${REMANI_BUILD:-0}\" = 1 ] || [ ! -x /opt/remani_ws/devel/lib/remani_planner/remani_planner_node ]; then catkin_make -DCMAKE_BUILD_TYPE=Release; fi && source /opt/remani_ws/devel/setup.bash && export ROS_PACKAGE_PATH=/opt/remani_ws/src/REMANI-Planner/local_robot:/opt/remani_ws/src/REMANI-Planner/TCP-IP-ROS-6AXis:\${ROS_PACKAGE_PATH:-} && roslaunch remani_planner exp0_ir100_cr10_manual.launch load_map:=true"
