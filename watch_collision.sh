#!/usr/bin/env bash
# Print the planner's real-time collision status for the measured robot state.
#
#   ./watch_collision.sh
#
# Types: -1 clear, 0 car-env, 1 arm-env, 2 arm-car, 3 arm-arm.
set -euo pipefail

CONTAINER="${REMANI_CONTAINER_NAME:-remani_pcd_demo}"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "watch_collision: container ${CONTAINER} is not running" >&2
  exit 1
fi

docker exec -i "${CONTAINER}" bash -lc '
source /opt/remani_ws/devel/setup.bash
python3 -u - <<PY
import rospy
from std_msgs.msg import Int32
NAMES = {-1: "clear", 0: "car-env", 1: "arm-env", 2: "arm-car", 3: "arm-arm"}
last = [None]
def cb(msg):
    if msg.data != last[0]:
        print("[collision_type] %s (%d)" % (NAMES.get(msg.data, "?"), msg.data), flush=True)
        last[0] = msg.data
rospy.init_node("watch_collision", anonymous=True)
rospy.Subscriber("/remani_planner_node/collision_type", Int32, cb)
print("[watch_collision] watching /remani_planner_node/collision_type; Ctrl-C to stop", flush=True)
rospy.spin()
PY
'
