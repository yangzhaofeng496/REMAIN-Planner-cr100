# Docker Deployment

This directory provides a reproducible ROS Noetic container for `REMANI-Planner`.

## What it does

- Uses `osrf/ros:noetic-desktop-full-focal` as the base image.
- Installs the system libraries required by the planner.
- Resolves ROS dependencies with `rosdep`.
- Skips stale package keys declared in upstream manifests but not required by the default simulation path:
  - `cmake_utils`
  - `svo_msgs`
  - `vikit_ros`
- Builds the workspace with `catkin_make`.

## Build

Run from the repository root:

```bash
cd /home/yang/tongji/ros1_ws/src/REMANI-Planner
docker build -f docker/Dockerfile -t remani-planner:noetic .
```

## Run `exp0`

If you want RViz on the host X11 display:

```bash
cd /home/yang/tongji/ros1_ws/src/REMANI-Planner
chmod +x docker/run_exp0.sh
./docker/run_exp0.sh remani-planner:noetic
```

Equivalent raw command:

```bash
xhost +local:root
docker run --rm -it \
  --net=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  remani-planner:noetic \
  roslaunch remani_planner exp0.launch
```

## Run `exp1`

```bash
docker run --rm -it \
  --net=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  remani-planner:noetic \
  roslaunch remani_planner exp1.launch
```

## Notes

- The image expects a Linux host with Docker and an X11 display server.
- If your host uses Wayland, enable XWayland or adapt the run command for your display setup.
- The custom RViz goal tool is kept enabled; an unused `multi_map_server` build dependency was removed because it blocked container builds but is not referenced by the plugin sources.
