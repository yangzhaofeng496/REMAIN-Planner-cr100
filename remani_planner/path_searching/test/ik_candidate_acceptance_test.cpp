#include <gtest/gtest.h>
#include <path_searching/sample_mani_RRT.h>

using mani_sample::ManiCollisionFn;
using mani_sample::acceptIkCandidate;

namespace {
const Eigen::Vector3d kCar(1.0, 2.0, 0.3);
const Eigen::VectorXd kJoints = Eigen::VectorXd::Constant(6, 0.4);
}  // namespace

TEST(IkCandidateAcceptance, RejectsArmCloudCollisionEvenWhenIkSucceeds) {
  std::array<size_t, 4> counts{{0, 0, 0, 0}};
  size_t ik_failures = 0;
  bool collision_called = false;
  ManiCollisionFn collision = [&](const Eigen::Vector3d &, const Eigen::VectorXd &,
                                  int &type) {
    collision_called = true;
    type = 1;  // arm-cloud
    return true;
  };
  int collision_type = -1;
  const bool accepted = acceptIkCandidate(true, kCar, kJoints, collision,
                                          collision_type, &counts, &ik_failures);
  EXPECT_FALSE(accepted);
  EXPECT_TRUE(collision_called);
  EXPECT_EQ(collision_type, 1);
  EXPECT_EQ(counts[1], 1u);
  EXPECT_EQ(counts[0], 0u);
  EXPECT_EQ(ik_failures, 0u);
}

TEST(IkCandidateAcceptance, RejectsIkFailureWithoutRunningCollision) {
  std::array<size_t, 4> counts{{0, 0, 0, 0}};
  size_t ik_failures = 0;
  bool collision_called = false;
  ManiCollisionFn collision = [&](const Eigen::Vector3d &, const Eigen::VectorXd &,
                                  int &type) {
    collision_called = true;
    type = 0;
    return true;
  };
  int collision_type = -1;
  const bool accepted = acceptIkCandidate(false, kCar, kJoints, collision,
                                          collision_type, &counts, &ik_failures);
  EXPECT_FALSE(accepted);
  EXPECT_FALSE(collision_called);
  EXPECT_EQ(ik_failures, 1u);
  EXPECT_EQ(counts[0] + counts[1] + counts[2] + counts[3], 0u);
}

TEST(IkCandidateAcceptance, AcceptsOnlyWhenIkAndCollisionBothPass) {
  std::array<size_t, 4> counts{{0, 0, 0, 0}};
  size_t ik_failures = 0;
  ManiCollisionFn collision = [&](const Eigen::Vector3d &, const Eigen::VectorXd &,
                                  int &type) {
    type = -1;
    return false;
  };
  int collision_type = -1;
  const bool accepted = acceptIkCandidate(true, kCar, kJoints, collision,
                                          collision_type, &counts, &ik_failures);
  EXPECT_TRUE(accepted);
  EXPECT_EQ(ik_failures, 0u);
  EXPECT_EQ(counts[0] + counts[1] + counts[2] + counts[3], 0u);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
