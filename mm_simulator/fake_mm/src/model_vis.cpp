#include "fake_mm/model_vis.hpp"

namespace model_vis{

    void ModelManager::init(ros::NodeHandle &nh){
        mm_config_.reset(new remani_planner::MMConfig);
        mm_config_->setParam(nh);
        nh.param("mm/use_robot_description_tf", use_robot_description_tf_, false);

        have_car_odom_ = false;
        have_mani_odom_ = false;
        /*  param  */
        nh.param("mm/manipulator_dof", manipulator_dof_, -1);
        
        mm_state_.setParam(manipulator_dof_);
        his_traj_.clear();
        
        his_traj_sphere_.header.frame_id = his_traj_line_strip_.header.frame_id = "world";
        his_traj_sphere_.header.stamp = his_traj_line_strip_.header.stamp = ros::Time::now();
        his_traj_sphere_.type = visualization_msgs::Marker::SPHERE_LIST;
        his_traj_line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
        his_traj_sphere_.action = his_traj_line_strip_.action = visualization_msgs::Marker::ADD;
        his_traj_sphere_.id = 0;
        his_traj_line_strip_.id = 1000;

        his_traj_sphere_.pose.orientation.w = his_traj_line_strip_.pose.orientation.w = 1.0;
        his_traj_sphere_.color.r = his_traj_line_strip_.color.r = 1.0;
        his_traj_sphere_.color.g = his_traj_line_strip_.color.g = 0.0;
        his_traj_sphere_.color.b = his_traj_line_strip_.color.b = 1.0;
        his_traj_sphere_.color.a = his_traj_line_strip_.color.a = 1.0;
        his_traj_sphere_.scale.x = 0.1;
        his_traj_sphere_.scale.y = 0.1;
        his_traj_sphere_.scale.z = 0.1;
        his_traj_line_strip_.scale.x = 0.1 / 2;

        have_gripper_state_ = false;
        reset_actual_path_ = true;

        /* callback */
        vis_mm_pub_            = nh.advertise<visualization_msgs::MarkerArray>("vis_mm", 100, true);
        vis_mm_check_ball_pub_ = nh.advertise<visualization_msgs::Marker>("vis_mm_check_ball", 100, true);
        joint_state_sub_        = nh.subscribe("joint_state", 1, &ModelManager::jointStateCallback, this);
        gripper_state_sub_      = nh.subscribe("gripper_state", 1, &ModelManager::gripperStateCallback, this);
        planning_start_sub_     = nh.subscribe("/planning/start", 1, &ModelManager::planningStartCallback, this);
        trajectory_sub_         = nh.subscribe("/planning/trajectory", 1, &ModelManager::trajectoryCallback, this);
        planned_ee_path_sub_    = nh.subscribe("/remani_planner_node/kinoastar/ee_path_nav", 1, &ModelManager::plannedEePathCallback, this);
        odom_sub_               = nh.subscribe("odometry", 1, &ModelManager::odomCallback, this);
        vis_his_traj_pub_       = nh.advertise<visualization_msgs::Marker>("mm_his_traj", 100, true);
        ee_path_pub_            = nh.advertise<nav_msgs::Path>("/remani_planner_node/kinoastar/ee_path_actual", 1, true);
        ee_path_.header.frame_id = "world";

        tf_timer_ = nh.createTimer(ros::Duration(0.01), &ModelManager::tfTimerCallback, this);

        std::vector<Eigen::Vector3d> car_pts;
        mm_config_->getCarPts(Eigen::Vector3d(0, 0, 0), car_pts, Eigen::Vector3d(0, 0, 0));
    }

    void ModelManager::tfTimerCallback(const ros::TimerEvent & event){ 
        if(!have_car_odom_ || !have_mani_odom_) return;
        ros::Time time = ros::Time::now();
        if(use_robot_description_tf_){
            broadcaster_.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(mm_state_.car_q.x(), mm_state_.car_q.y(), mm_state_.car_q.z(), mm_state_.car_q.w()),
                    tf::Vector3(mm_state_.car_p(0), mm_state_.car_p(1), 0.0)),
                    time, "world", "base_link"));
            return;
        }
        broadcaster_.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(mm_state_.car_q.x(), mm_state_.car_q.y(), mm_state_.car_q.z(), mm_state_.car_q.w()), 
                tf::Vector3(mm_state_.car_p(0), mm_state_.car_p(1), 0.0)),
                time, "world", "mm_base"));
        Eigen::Matrix4d mat = mm_config_->getTq0(), mat_nouse;
        Eigen::Matrix3d rot = mat.block(0, 0, 3, 3);
        Eigen::Quaterniond quaternion(rot);
        broadcaster_.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()), 
                tf::Vector3(mat(0, 3), mat(1, 3), mat(2, 3))),
                time, "mm_base", "mani_0"));
        for(int i = 0; i < 6; ++i){
            mm_config_->getAJointTran(i, mm_state_.joint_p(i), mat, mat_nouse);
            rot = mat.block(0, 0, 3, 3);
            Eigen::Quaterniond quaternion1(rot);
            std::string frame1 = "mani_" + std::to_string(i);
            std::string frame2 = "mani_" + std::to_string(i + 1);
            broadcaster_.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(quaternion1.x(), quaternion1.y(), quaternion1.z(), quaternion1.w()), 
                    tf::Vector3(mat(0, 3), mat(1, 3), mat(2, 3))),
                    time, frame1, frame2));
        }
        
    }

    void ModelManager::odomCallback(const nav_msgs::OdometryConstPtr& odom){
        have_car_odom_ = true;
        Eigen::Vector3d new_car_p(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
        if((mm_state_.car_p - new_car_p).norm() > 1e-2){
            vis_his_traj(new_car_p.head(2));
        }
        mm_state_.feed_odom(odom);
        if(have_mani_odom_ && !use_robot_description_tf_){
            mm_config_->visMM(vis_mm_pub_, "vis_mm_odom", 0, -0.9, Eigen::Vector3d(mm_state_.car_p(0), mm_state_.car_p(1), mm_state_.car_yaw), mm_state_.joint_p, gripper_state_);
            mm_config_->visMMCheckBall(vis_mm_check_ball_pub_, "vis_mm_check_ball", 0, 0.7, Eigen::Vector3d(mm_state_.car_p(0), mm_state_.car_p(1), mm_state_.car_yaw), mm_state_.joint_p);
        }
        his_traj_.push_back(Eigen::Vector2d(mm_state_.car_p(0), mm_state_.car_p(1)));
    }

    void ModelManager::jointStateCallback(const sensor_msgs::JointState::ConstPtr& state){
        have_mani_odom_ = true;
        mm_state_.feed_joint(state);
        if (have_car_odom_) publishActualEePath();
        if(have_car_odom_ && !use_robot_description_tf_){
            mm_config_->visMM(vis_mm_pub_, "vis_mm_odom", 0, -0.9, Eigen::Vector3d(mm_state_.car_p(0), mm_state_.car_p(1), mm_state_.car_yaw), mm_state_.joint_p, gripper_state_);
            mm_config_->visMMCheckBall(vis_mm_check_ball_pub_, "vis_mm_check_ball", 0, 0.7, Eigen::Vector3d(mm_state_.car_p(0), mm_state_.car_p(1), mm_state_.car_yaw), mm_state_.joint_p);
        }
    }

    void ModelManager::publishActualEePath(){
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp = ros::Time::now();
        tf::StampedTransform ee_tf;
        try {
          tf_listener_.lookupTransform("world", "arm_gripper_link", ros::Time(0), ee_tf);
          pose.header.stamp = ee_tf.stamp_;
          pose.pose.position.x = ee_tf.getOrigin().x();
          pose.pose.position.y = ee_tf.getOrigin().y();
          pose.pose.position.z = ee_tf.getOrigin().z();
          pose.pose.orientation.x = ee_tf.getRotation().x();
          pose.pose.orientation.y = ee_tf.getRotation().y();
          pose.pose.orientation.z = ee_tf.getRotation().z();
          pose.pose.orientation.w = ee_tf.getRotation().w();
        } catch (const tf::TransformException &ex) {
          ROS_WARN_THROTTLE(2.0, "Waiting for world->arm_gripper_link TF: %s", ex.what());
          return;
        }
        if (reset_actual_path_) {
          ee_path_.poses.clear();
          reset_actual_path_ = false;
        }
        if (ee_path_.poses.empty() ||
            (Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) -
             Eigen::Vector3d(ee_path_.poses.back().pose.position.x, ee_path_.poses.back().pose.position.y,
                             ee_path_.poses.back().pose.position.z)).norm() > 0.005) {
          ee_path_.header.stamp = pose.header.stamp;
          ee_path_.poses.push_back(pose);
          if (ee_path_.poses.size() > 5000) ee_path_.poses.erase(ee_path_.poses.begin(), ee_path_.poses.begin() + 1000);
          ee_path_pub_.publish(ee_path_);

          if (!planned_ee_path_.poses.empty()) {
            const double actual_t = pose.header.stamp.toSec();
            size_t best = 0;
            double best_dt = std::numeric_limits<double>::max();
            for (size_t i = 0; i < planned_ee_path_.poses.size(); ++i) {
              double dt = std::abs(planned_ee_path_.poses[i].header.stamp.toSec() - actual_t);
              if (dt < best_dt) { best_dt = dt; best = i; }
            }
            const auto &ref = planned_ee_path_.poses[best].pose.position;
            const double dx = pose.pose.position.x - ref.x;
            const double dy = pose.pose.position.y - ref.y;
            const double dz = pose.pose.position.z - ref.z;
            ROS_WARN_THROTTLE(1.0, "EE tracking error dt=%.3f dx=%.4f dy=%.4f dz=%.4f dist=%.4f",
                              best_dt, dx, dy, dz, std::sqrt(dx*dx + dy*dy + dz*dz));
          }
        }
    }

    void ModelManager::gripperStateCallback(const std_msgs::Bool::ConstPtr& state){
        if(gripper_state_ != state->data || (!have_gripper_state_)){
            have_gripper_state_ = true;
            gripper_state_ = state->data;
            mm_config_->setGripperPoint(gripper_state_);
        }
    }

    void ModelManager::planningStartCallback(const std_msgs::Bool::ConstPtr& state){
        if (!state->data) return;
        reset_actual_path_ = true;
        ee_path_.poses.clear();
        ee_path_.header.frame_id = "world";
        ee_path_pub_.publish(ee_path_);
    }

    void ModelManager::trajectoryCallback(const quadrotor_msgs::PolynomialTraj::ConstPtr& msg){
        if (msg->action != quadrotor_msgs::PolynomialTraj::ACTION_ADD || msg->trajectory_id != 1)
            return;
        // A new controller trajectory defines a new comparison episode.  The
        // trajectory message is the authoritative synchronization event;
        // planning/start can arrive before the controller receives it.
        reset_actual_path_ = true;
        ee_path_.poses.clear();
        ee_path_.header.frame_id = "world";
        ee_path_pub_.publish(ee_path_);
    }

    void ModelManager::plannedEePathCallback(const nav_msgs::Path::ConstPtr& msg){
        planned_ee_path_ = *msg;
        ROS_WARN_STREAM("[EE compare] received planned path poses=" << planned_ee_path_.poses.size());
    }

    void ModelManager::vis_his_traj(Eigen::Vector2d pt){
        geometry_msgs::Point vis_pt;
        vis_pt.x = pt(0);
        vis_pt.y = pt(1);
        vis_pt.z = 0.0;
        his_traj_sphere_.points.push_back(vis_pt);
        his_traj_line_strip_.points.push_back(vis_pt);
        vis_his_traj_pub_.publish(his_traj_sphere_);
        if(his_traj_line_strip_.points.size() > 1)
            vis_his_traj_pub_.publish(his_traj_line_strip_);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "model_vis");
    ros::NodeHandle nh("~");
    model_vis::ModelManager mm;
    mm.init(nh);
    ros::spin();
    return 0;
}
