#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def main():
    rospy.init_node('simple_joint_state_publisher')
    pub = rospy.Publisher('/mm/mani/joint_state', JointState, queue_size=10)

    rate = rospy.Rate(30)  # 30 Hz

    # CR10 joint names (from URDF)
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    # 初始姿态：全零
    joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    rospy.loginfo("Publishing joint states to /mm/mani/joint_state")

    while not rospy.is_shutdown():
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.name = joint_names
        msg.position = joint_positions
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
