#pragma once

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/time.h>
#include <string>

namespace remani_planner
{

// Convert a homogeneous end-effector transform into a stamped pose that uses
// the path's frame.
inline geometry_msgs::PoseStamped transformToEePose(const Eigen::Matrix4d &T,
                                                    const std::string &frame_id,
                                                    const ros::Time &stamp)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = stamp;
  pose.pose.position.x = T(0, 3);
  pose.pose.position.y = T(1, 3);
  pose.pose.position.z = T(2, 3);
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  q.normalize();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  return pose;
}

// Append one measured end-effector pose to the actual path, keeping at most
// `max_poses` samples so a long run cannot grow without bound.
inline void appendEePose(nav_msgs::Path &path, const Eigen::Matrix4d &T,
                         const ros::Time &stamp, size_t max_poses = 4000)
{
  path.poses.push_back(transformToEePose(T, path.header.frame_id, stamp));
  if (path.poses.size() > max_poses)
  {
    path.poses.erase(path.poses.begin(),
                     path.poses.begin() + (path.poses.size() - max_poses));
  }
}

inline void resetEePath(nav_msgs::Path &path, const std::string &frame_id,
                        const ros::Time &stamp)
{
  path.poses.clear();
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
}

} // namespace remani_planner
