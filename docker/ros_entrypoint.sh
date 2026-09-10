#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /opt/remani_ws/devel/setup.bash

exec "$@"
