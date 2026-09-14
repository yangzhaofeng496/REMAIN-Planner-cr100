#!/usr/bin/env bash
# Save the current interactive PCD placement as the startup initial pose.
#
# Reads the latched /static_pcd_publisher/current_pose from the running demo
# container and writes pcd_initial_pose.env, which run_pcd_demo.sh replays on
# the next startup.
#
#   ./save_pcd_pose.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${REMANI_CONTAINER_NAME:-remani_pcd_demo}"
OUT="${REPO_ROOT}/pcd_initial_pose.env"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "save_pcd_pose: container ${CONTAINER} is not running" >&2
  exit 1
fi

pose="$(docker exec "${CONTAINER}" bash -lc '
source /opt/remani_ws/devel/setup.bash
python3 - <<PY
import math, rospy
from geometry_msgs.msg import PoseStamped
rospy.init_node("read_current_pose", anonymous=True)
m = rospy.wait_for_message("/static_pcd_publisher/current_pose", PoseStamped, timeout=15)
q = m.pose.orientation
yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
print("%.4f %.4f %.4f %.4f" % (m.pose.position.x, m.pose.position.y, m.pose.position.z, yaw))
PY
')"

if [[ -z "${pose}" ]]; then
  echo "save_pcd_pose: failed to read the current pose" >&2
  exit 1
fi

printf 'REMANI_INITIAL_POSE="%s"\n' "${pose}" > "${OUT}"
echo "save_pcd_pose: saved [${pose}] to ${OUT}"
