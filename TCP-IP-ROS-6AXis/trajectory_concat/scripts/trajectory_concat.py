#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import threading
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class TrajectoryPointPublisher:
    def __init__(self):
        rospy.init_node('trajectory_point_publisher')

        # 发布当前时间对应的目标点
        self.point_pub = rospy.Publisher('/current_target_point', JointTrajectoryPoint, queue_size=10)

        # 订阅 MoveIt 规划的轨迹
        self.trajectory_sub = rospy.Subscriber(
            '/move_group/display_planned_path',
            DisplayTrajectory,
            self.trajectory_callback,
            queue_size=10
        )

        self.trajectory = None       # 存储当前轨迹
        self.start_time = 0.0        # 轨迹启动的系统时间
        self.publishing = False      # 发布标志

        rospy.loginfo("Node started: waiting for trajectory...")

        # 启动发布线程
        self.start_publish_thread()

    def trajectory_callback(self, display_msg):
        # 接收 MoveIt 发送的轨迹
        self.trajectory = display_msg.trajectory[0].joint_trajectory
        self.start_time = rospy.Time.now().to_sec()
        self.publishing = True
        rospy.loginfo("Received new trajectory, point count: %d" % len(self.trajectory.points))

    def publish_loop(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if not self.publishing or self.trajectory is None:
                rate.sleep()
                continue

            # 计算轨迹已运行的时间
            current_time = rospy.Time.now().to_sec()
            elapsed = current_time - self.start_time

            points = self.trajectory.points
            target_point = points[-1]

            # 找到当前 elapsed 时间对应的轨迹点
            for pt in points:
                t = pt.time_from_start.to_sec()
                if elapsed <= t:
                    target_point = pt
                    break

            # 发布当前目标点
            self.point_pub.publish(target_point)
            rate.sleep()

    def start_publish_thread(self):
        thread = threading.Thread(target=self.publish_loop)
        thread.daemon = True
        thread.start()

if __name__ == '__main__':
    try:
        TrajectoryPointPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass