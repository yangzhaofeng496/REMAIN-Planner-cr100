#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include "mm_controller/mm_controller_fsm.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "MMctrl");
    ros::NodeHandle nh("~");
    ros::Duration(1.0).sleep();
    MMController::MMControllerFSM fsm(nh);

    ros::Subscriber car_odom_sub = 
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&MMController::State_Data_t::feed_odom, &fsm.state_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber joint_state_sub =
        nh.subscribe<sensor_msgs::JointState>("joint_states",
                                         100,
                                         boost::bind(&MMController::State_Data_t::feed_joint, &fsm.state_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber traj_sub =
        nh.subscribe<quadrotor_msgs::PolynomialTraj>("planning/trajectory",
                                                      100,
                                                      boost::bind(&MMController::Trajectory_Data_t::feed, &fsm.trajectory_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    // RViz 2D Pose Estimate teleports the base to the requested pose (the
    // controller holds that pose until a new trajectory arrives).
    ros::Subscriber init_pose_sub =
        nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            "/initialpose", 1,
            [&fsm](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
                fsm.stay_pos_(0) = msg->pose.pose.position.x;
                fsm.stay_pos_(1) = msg->pose.pose.position.y;
                fsm.stay_yaw_ = tf::getYaw(msg->pose.pose.orientation);
                ROS_WARN("[MMctrl] initial pose set to (%.2f, %.2f, yaw=%.2f)",
                         fsm.stay_pos_(0), fsm.stay_pos_(1), fsm.stay_yaw_);
            });

    ros::Duration(0.5).sleep();

    ros::Rate r(100);
    while (ros::ok()){
        r.sleep();
        ros::spinOnce();
        fsm.process();  // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }

    return 0;
}

