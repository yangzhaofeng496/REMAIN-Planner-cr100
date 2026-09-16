#include <gtest/gtest.h>
#include <plan_manage/ee_path_utils.h>

using remani_planner::appendEePose;
using remani_planner::resetEePath;
using remani_planner::transformToEePose;

TEST(EePathUtils, PoseUsesWorldFrameAndTransform) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 3) = 1.5;
  T(1, 3) = -2.0;
  T(2, 3) = 0.75;
  const auto pose = transformToEePose(T, "world", ros::Time(3.0));
  EXPECT_EQ(pose.header.frame_id, "world");
  EXPECT_DOUBLE_EQ(pose.header.stamp.toSec(), 3.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, -2.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.z, 0.75);
  EXPECT_NEAR(pose.pose.orientation.w, 1.0, 1e-9);
}

TEST(EePathUtils, AppendInheritsFrameAndCapsLength) {
  nav_msgs::Path path;
  resetEePath(path, "world", ros::Time(0.0));
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 10; ++i) {
    T(0, 3) = 0.1 * i;
    appendEePose(path, T, ros::Time(i), 4);
  }
  EXPECT_EQ(path.header.frame_id, "world");
  ASSERT_EQ(path.poses.size(), 4u);
  EXPECT_EQ(path.poses.front().header.frame_id, "world");
  // Oldest samples were dropped; the newest sample x = 0.9 is retained.
  EXPECT_NEAR(path.poses.back().pose.position.x, 0.9, 1e-9);
}

TEST(EePathUtils, RotationIsConverted) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  const double half = std::sin(0.25 * M_PI);
  const double cos_half = std::cos(0.25 * M_PI);
  T.block<3, 3>(0, 0) = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ())
                            .toRotationMatrix();
  const auto pose = transformToEePose(T, "world", ros::Time(0.0));
  EXPECT_NEAR(pose.pose.orientation.z, half, 1e-9);
  EXPECT_NEAR(pose.pose.orientation.w, cos_half, 1e-9);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
