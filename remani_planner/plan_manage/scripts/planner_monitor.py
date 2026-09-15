#!/usr/bin/env python3
"""Print REMANI-Planner planning state and key events from ROS topics."""
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PolynomialTraj
from rosgraph_msgs.msg import Log
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

PLANNER_NODE = "/remani_planner_node"
LEVELS = {1: "DEBUG", 2: "INFO", 4: "WARN", 8: "ERROR", 16: "FATAL"}
KEYWORDS = ("fsm", "collision", "no_path", "reach goal", "initialization",
            "warmstart", "profile", "safety", "traj", "path", "plan", "status",
            "failed", "retry", "timeout")


def _yaw(orientation):
    return math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))


def on_log(msg):
    if PLANNER_NODE not in msg.name:
        return
    text = msg.msg
    if any(keyword in text.lower() for keyword in KEYWORDS):
        print("[%s] %s" % (LEVELS.get(msg.level, msg.level), text), flush=True)


def on_goal(msg):
    print("[GOAL] x=%.2f y=%.2f yaw=%.2f" %
          (msg.pose.position.x, msg.pose.position.y, _yaw(msg.pose.orientation)), flush=True)


def on_start(msg):
    if msg.data:
        print("[PLAN] === start ===", flush=True)


def on_finish(msg):
    if msg.data:
        print("[PLAN] === finish ===", flush=True)


def on_trajectory(msg):
    durations = [piece.duration for piece in msg.trajectory]
    print("[TRAJ] id=%d action=%d seg=%d duration=%.3fs" %
          (msg.trajectory_id, msg.action, len(msg.trajectory), sum(durations)), flush=True)


def on_failed(msg):
    if msg.points:
        print("[FAILED] marker '%s' with %d points" % (msg.ns, len(msg.points)), flush=True)


def main():
    rospy.init_node("planner_monitor", anonymous=True)
    rospy.Subscriber("/rosout", Log, on_log, queue_size=100)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, on_goal, queue_size=10)
    rospy.Subscriber("/planning/start", Bool, on_start, queue_size=10)
    rospy.Subscriber("/planning/finish", Bool, on_finish, queue_size=10)
    rospy.Subscriber("/planning/trajectory", PolynomialTraj, on_trajectory, queue_size=10)
    rospy.Subscriber("%s/failed_list" % PLANNER_NODE, Marker, on_failed, queue_size=10)
    print("[monitor] watching %s planning state; Ctrl-C to stop" % PLANNER_NODE, flush=True)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit(0)
