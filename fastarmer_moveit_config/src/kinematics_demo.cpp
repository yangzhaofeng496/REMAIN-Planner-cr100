#include <ros/ros.h>

#include <Eigen/Geometry>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <XmlRpcValue.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace
{
bool readArray(const ros::NodeHandle& nh, const std::string& name, std::size_t size,
              std::vector<double>& values)
{
  XmlRpc::XmlRpcValue raw;
  if (!nh.getParam(name, raw) || raw.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      raw.size() != static_cast<int>(size))
    return false;

  values.clear();
  for (int i = 0; i < raw.size(); ++i)
  {
    if (raw[i].getType() != XmlRpc::XmlRpcValue::TypeInt &&
        raw[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
      return false;
    if (raw[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
      values.push_back(static_cast<int>(raw[i]));
    else
      values.push_back(static_cast<double>(raw[i]));
  }
  return true;
}

void printPose(const Eigen::Isometry3d& pose)
{
  const Eigen::Vector3d rpy = pose.rotation().eulerAngles(0, 1, 2);
  ROS_INFO_STREAM("arm_gripper_link position [m]: " << pose.translation().transpose()
                  << "; rpy [rad]: " << rpy.transpose());
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fastarmer_kinematics_demo");
  ros::NodeHandle nh("~");

  std::string mode;
  nh.param<std::string>("mode", mode, "fk");

  robot_model_loader::RobotModelLoader loader("robot_description");
  const moveit::core::RobotModelPtr model = loader.getModel();
  if (!model)
  {
    ROS_ERROR("Could not load robot model from robot_description");
    return 1;
  }

  const moveit::core::JointModelGroup* group = model->getJointModelGroup("arm");
  if (!group || group->getVariableCount() != 6 || model->getLinkModel("arm_gripper_link") == nullptr)
  {
    ROS_ERROR("Expected arm group with six variables and terminal arm_gripper_link");
    return 1;
  }

  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  std::vector<double> joints;
  if (!readArray(nh, "joint_values", 6, joints))
  {
    ROS_ERROR("~joint_values must be an array of six numeric values");
    return 1;
  }

  state.setJointGroupPositions(group, joints);
  state.update();
  const Eigen::Isometry3d base_fk = state.getGlobalLinkTransform("arm_gripper_link");

  if (mode == "fk")
  {
    printPose(base_fk);
    return 0;
  }
  if (mode != "ik")
  {
    ROS_ERROR("~mode must be either fk or ik");
    return 1;
  }

  std::vector<double> target;
  if (!readArray(nh, "target_pose", 6, target))
  {
    ROS_ERROR("~target_pose must be [x, y, z, roll, pitch, yaw]");
    return 1;
  }
  Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
  target_pose.translation() = Eigen::Vector3d(target[0], target[1], target[2]);
  target_pose.linear() = (Eigen::AngleAxisd(target[3], Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(target[4], Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(target[5], Eigen::Vector3d::UnitZ())).toRotationMatrix();

  state.setJointGroupPositions(group, joints);
  if (!state.setFromIK(group, target_pose, "arm_gripper_link"))
  {
    ROS_ERROR("Inverse kinematics failed");
    return 1;
  }
  state.update();
  const Eigen::Isometry3d achieved = state.getGlobalLinkTransform("arm_gripper_link");
  const double translation_error = (achieved.translation() - target_pose.translation()).norm();
  const double rotation_error = Eigen::AngleAxisd(target_pose.rotation().transpose() * achieved.rotation()).angle();
  if (translation_error > 1e-4 || rotation_error > 1e-3)
  {
    ROS_ERROR_STREAM("IK round-trip error too large: translation=" << translation_error
                     << " m, rotation=" << rotation_error << " rad");
    return 1;
  }

  std::vector<double> solution;
  state.copyJointGroupPositions(group, solution);
  ROS_INFO_STREAM("IK solution: " << Eigen::Map<Eigen::VectorXd>(solution.data(), solution.size()).transpose());
  printPose(achieved);
  ROS_INFO_STREAM("IK error: translation=" << translation_error << " m, rotation=" << rotation_error << " rad");
  return 0;
}
