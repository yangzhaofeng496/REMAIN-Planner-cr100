#include <gtest/gtest.h>
#include <path_searching/sample_mani_RRT.h>

using mani_sample::LayeredManiWaypoint;
using mani_sample::interpolateJointSegment;

TEST(LayeredWaypoint, StoresLayerEeAndJoints) {
  LayeredManiWaypoint wp;
  wp.layer = 7;
  wp.ee_world = Eigen::Vector3d(1.0, 2.0, 3.0);
  wp.joint_state = Eigen::VectorXd::Constant(6, 0.25);
  EXPECT_EQ(wp.layer, 7);
  EXPECT_TRUE(wp.ee_world.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_EQ(wp.joint_state.size(), 6);
}

TEST(InterpolateJointSegment, PreservesEndpointsAndCount) {
  const Eigen::VectorXd q0 = Eigen::VectorXd::LinSpaced(6, -1.0, -0.5);
  const Eigen::VectorXd q1 = Eigen::VectorXd::LinSpaced(6, 0.5, 1.0);
  std::vector<Eigen::VectorXd> out;
  ASSERT_TRUE(interpolateJointSegment(q0, q1, 5, out));
  ASSERT_EQ(out.size(), 5u);
  EXPECT_TRUE(out.front().isApprox(q0));
  EXPECT_TRUE(out.back().isApprox(q1));
}

TEST(InterpolateJointSegment, LinearMidpoint) {
  const Eigen::VectorXd q0 = Eigen::VectorXd::Constant(6, 0.0);
  const Eigen::VectorXd q1 = Eigen::VectorXd::Constant(6, 2.0);
  std::vector<Eigen::VectorXd> out;
  ASSERT_TRUE(interpolateJointSegment(q0, q1, 3, out));
  ASSERT_EQ(out.size(), 3u);
  EXPECT_TRUE(out[1].isApprox(Eigen::VectorXd::Constant(6, 1.0)));
}

TEST(InterpolateJointSegment, RejectsMismatchedDimensions) {
  const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(6);
  const Eigen::VectorXd q1 = Eigen::VectorXd::Zero(5);
  std::vector<Eigen::VectorXd> out;
  EXPECT_FALSE(interpolateJointSegment(q0, q1, 4, out));
  EXPECT_TRUE(out.empty());
}

TEST(InterpolateJointSegment, RejectsTooFewSamples) {
  const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(6);
  const Eigen::VectorXd q1 = Eigen::VectorXd::Ones(6);
  std::vector<Eigen::VectorXd> out;
  EXPECT_FALSE(interpolateJointSegment(q0, q1, 1, out));
  EXPECT_FALSE(interpolateJointSegment(q0, q1, 0, out));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
