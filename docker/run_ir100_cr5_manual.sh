#!/bin/bash
set -euo pipefail

IMAGE_NAME="${1:-remani-planner:noetic}"

xhost +local:root
trap 'xhost -local:root >/dev/null 2>&1 || true' EXIT

docker run --rm -it \
  --net=host \
  --ipc=host \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  "${IMAGE_NAME}" \
  roslaunch remani_planner exp0_ir100_cr5_manual.launch
