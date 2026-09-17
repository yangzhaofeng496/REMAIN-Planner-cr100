#!/usr/bin/env python3
"""Keyboard base + arm teleop with live collision status from the planner.

Base: W/S forward/back, A/D rotate.
Arm : 1..6 nudge the matching joint by one step; Space flips the nudge
      direction (increase <-> decrease).
Q   : quit.

Base commands go to /mm_controller_node/car_cmd (absolute pose setpoint, as
expected by fake_mm) and arm commands to /mm_controller_node/joint_cmd.
Collision status comes from /remani_planner_node/collision_type:
  -1 safe, 0 car-env, 1 arm-env, 2 arm-car, 3 arm-arm.
"""
import math
import select
import subprocess
import sys
import termios
import tty

import rospy
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

STEP_LINEAR = 0.05
STEP_YAW = 0.10
STEP_ARM = 0.10
ARM_DOF = 6
JOINT_MIN, JOINT_MAX = -3.14, 3.14
STATUS = {-1: "SAFE", 0: "COLLISION car-env", 1: "COLLISION arm-env",
          2: "COLLISION arm-car", 3: "COLLISION arm-arm"}


def main():
    rospy.init_node("base_teleop_collision")
    # Keep mm_controller_node alive so planner trajectories can still be
    # executed while teleop is active.  Teleop publishes manual setpoints to
    # fake_mm through the same command topics; killing the controller would
    # disconnect /planning/trajectory from the simulated robot.
    if rospy.get_param("~kill_controller", False):
        subprocess.call(["rosnode", "kill", "/mm_controller_node"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    base_pub = rospy.Publisher("/mm_controller_node/car_cmd", Twist, queue_size=1)
    arm_pub = rospy.Publisher("/mm_controller_node/joint_cmd",
                              JointTrajectoryControllerState, queue_size=1)
    state = {"x": 0.0, "y": 0.0, "yaw": 0.0, "have": False}
    arm = {"q": [0.0] * ARM_DOF, "sel": 0, "have": False, "dir": 1}
    coll = {"type": -1}

    def on_odom(msg):
        q = msg.pose.pose.orientation
        state["x"] = msg.pose.pose.position.x
        state["y"] = msg.pose.pose.position.y
        state["yaw"] = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        state["have"] = True

    def on_joint(msg):
        if len(msg.position) >= ARM_DOF:
            arm["q"] = [msg.position[i] for i in range(ARM_DOF)]
            arm["have"] = True

    def on_collision(msg):
        coll["type"] = msg.data

    rospy.Subscriber("/mm/car/odom", Odometry, on_odom)
    rospy.Subscriber("/mm/mani/joint_state", JointState, on_joint)
    rospy.Subscriber("/remani_planner_node/collision_type", Int32, on_collision)
    while (not state["have"] or not arm["have"]) and not rospy.is_shutdown():
        rospy.sleep(0.05)

    print("Base: W/S move, A/D rotate", flush=True)
    print("Arm : 1..6 nudge matching joint, Space flips nudge direction, Q quit", flush=True)
    print("[arm] nudge direction: + (increase)", flush=True)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        rate = rospy.Rate(20)
        last = None
        while not rospy.is_shutdown():
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            key = sys.stdin.read(1) if ready else ""
            key_l = key.lower()
            if key_l == "w":
                state["x"] += STEP_LINEAR * math.cos(state["yaw"])
                state["y"] += STEP_LINEAR * math.sin(state["yaw"])
            elif key_l == "s":
                state["x"] -= STEP_LINEAR * math.cos(state["yaw"])
                state["y"] -= STEP_LINEAR * math.sin(state["yaw"])
            elif key_l == "a":
                state["yaw"] += STEP_YAW
            elif key_l == "d":
                state["yaw"] -= STEP_YAW
            elif key in ("1", "2", "3", "4", "5", "6"):
                i = int(key) - 1
                arm["sel"] = i
                arm["q"][i] = min(JOINT_MAX, max(JOINT_MIN,
                                  arm["q"][i] + arm["dir"] * STEP_ARM))
                print("[arm] joint%d = %.2f rad (dir %s)" %
                      (i + 1, arm["q"][i], "+" if arm["dir"] > 0 else "-"),
                      flush=True)
            elif key == " ":
                arm["dir"] = -arm["dir"]
                print("[arm] nudge direction: %s (%s)" %
                      ("+" if arm["dir"] > 0 else "-",
                       "increase" if arm["dir"] > 0 else "decrease"), flush=True)
            elif key_l == "q":
                break

            cmd = Twist()
            cmd.linear.x = state["x"]
            cmd.linear.y = state["y"]
            cmd.linear.z = state["yaw"]
            base_pub.publish(cmd)

            arm_cmd = JointTrajectoryControllerState()
            arm_cmd.joint_names = ["joint%d" % (i + 1) for i in range(ARM_DOF)]
            arm_cmd.desired.positions = list(arm["q"])
            arm_cmd.desired.velocities = [0.0] * ARM_DOF
            arm_cmd.desired.effort = [0.0] * ARM_DOF
            arm_pub.publish(arm_cmd)

            if coll["type"] != last:
                print("[%s] pose=(%.2f %.2f yaw=%.2f) q=[%s]" %
                      (STATUS.get(coll["type"], "?"), state["x"], state["y"],
                       state["yaw"], " ".join("%.2f" % v for v in arm["q"])),
                      flush=True)
                last = coll["type"]
            rate.sleep()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


if __name__ == "__main__":
    main()
