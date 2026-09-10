#!/bin/bash
set -euo pipefail

IMAGE_NAME="${1:-remani-planner:noetic}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

xhost +local:root 2>/dev/null || true
trap 'xhost -local:root >/dev/null 2>&1 || true' EXIT

docker run --rm \
  --net=host \
  --ipc=host \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${REPO_ROOT}:/opt/remani_ws/src/REMANI-Planner:rw" \
  "${IMAGE_NAME}" \
  bash -lc "cd /opt/remani_ws && catkin_make -DCMAKE_BUILD_TYPE=Release && source /opt/remani_ws/devel/setup.bash && roslaunch remani_planner exp0.launch load_map:=false"
