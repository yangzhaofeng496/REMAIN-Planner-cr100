#include <gtest/gtest.h>
#include <path_searching/sample_mani_RRT.h>

using mani_sample::checkJointTransition;
using mani_sample::ManiBasePoseFn;
using mani_sample::ManiCollisionFn;

namespace {
constexpr int kDof = 6;
}  // namespace

TEST(LayerConnection, AcceptsSafeEdge) {
  const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(kDof);
  const Eigen::VectorXd q1 = Eigen::VectorXd::Constant(kDof, 0.2);
  ManiBasePoseFn base_pose = [](double f) { return Eigen::Vector3d(f, 0.0, 0.0); };
  bool called = false;
  ManiCollisionFn collision = [&](const Eigen::Vector3d &, const Eigen::VectorXd &,
                                  int &type) {
    called = true;
    type = -1;
    return false;
  };
  int collision_type = -1;
  EXPECT_TRUE(checkJointTransition(q0, q1, 1.0, 1.0, 11, base_pose, collision,
                                   collision_type));
  EXPECT_TRUE(called);
}

TEST(LayerConnection, RejectsMidpointCollisionWithSafeEndpoints) {
  const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(kDof);
  const Eigen::VectorXd q1 = Eigen::VectorXd::Constant(kDof, 0.5);
  ManiBasePoseFn base_pose = [](double f) { return Eigen::Vector3d(f, 0.0, 0.0); };
  // Only the middle of the transition collides; endpoints are safe.
  ManiCollisionFn collision = [&](const Eigen::Vector3d &car_state,
                                  const Eigen::VectorXd &, int &type) {
    if (car_state.x() > 0.4 && car_state.x() < 0.6) {
      type = 1;
      return true;
    }
    type = -1;
    return false;
  };
  int collision_type = -1;
  EXPECT_FALSE(checkJointTransition(q0, q1, 1.0, 1.0, 11, base_pose, collision,
                                    collision_type));
  EXPECT_EQ(collision_type, 1);
}

TEST(LayerConnection, RejectsVelocityViolationWithoutCollisionCheck) {
  const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(kDof);
  const Eigen::VectorXd q1 = Eigen::VectorXd::Constant(kDof, 5.0);
  ManiBasePoseFn base_pose = [](double) { return Eigen::Vector3d::Zero(); };
  bool called = false;
  ManiCollisionFn collision = [&](const Eigen::Vector3d &, const Eigen::VectorXd &,
                                  int &type) {
    called = true;
    type = -1;
    return false;
  };
  int collision_type = -1;
  EXPECT_FALSE(checkJointTransition(q0, q1, 1.0, 1.0, 11, base_pose, collision,
                                    collision_type));
  EXPECT_FALSE(called);
}

TEST(LayerConnection, RejectsDimensionMismatch) {
  const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(kDof);
  const Eigen::VectorXd q1 = Eigen::VectorXd::Zero(kDof - 1);
  ManiBasePoseFn base_pose = [](double) { return Eigen::Vector3d::Zero(); };
  ManiCollisionFn collision = [](const Eigen::Vector3d &, const Eigen::VectorXd &,
                                 int &type) {
    type = -1;
    return false;
  };
  int collision_type = -1;
  EXPECT_FALSE(checkJointTransition(q0, q1, 1.0, 1.0, 11, base_pose, collision,
                                    collision_type));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
