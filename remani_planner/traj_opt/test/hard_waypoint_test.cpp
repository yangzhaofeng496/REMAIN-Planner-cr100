#include <gtest/gtest.h>
#include <optimizer/poly_traj_optimizer.hpp>

using remani_planner::hardWaypointGradCost;
using remani_planner::hardWaypointsSatisfied;

namespace {
constexpr int kTrajDim = 8;
constexpr int kBase = 2;
constexpr int kArm = 6;

Eigen::MatrixXd makeHard() {
  Eigen::MatrixXd hard = Eigen::MatrixXd::Zero(kTrajDim, 3);
  hard.block(kBase, 0, kArm, 3).setConstant(1.0);
  return hard;
}
}  // namespace

TEST(HardWaypointPenalty, CostAndGradientTargetArmRows) {
  const Eigen::MatrixXd hard = makeHard();
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(kTrajDim, 3);
  Eigen::MatrixXd grad = Eigen::MatrixXd::Zero(kTrajDim, 3);
  const double weight = 1000.0;
  const double cost = hardWaypointGradCost(points, hard, kBase, kArm, weight, grad);
  // 6 arm rows * 3 columns, each diff 1.0.
  EXPECT_NEAR(cost, weight * kArm * 3, 1e-9);
  // Gradient points toward the target: 2 * weight * (0 - 1).
  EXPECT_NEAR(grad(kBase, 0), -2.0 * weight, 1e-9);
  // Base rows are untouched.
  for (int r = 0; r < kBase; ++r)
    for (int c = 0; c < 3; ++c) EXPECT_DOUBLE_EQ(grad(r, c), 0.0);
}

TEST(HardWaypointPenalty, GradientStepRecoversWaypoints) {
  const Eigen::MatrixXd hard = makeHard();
  Eigen::MatrixXd points = hard;
  // Pretend an unconstrained optimizer moved the arm waypoints by 1 rad.
  points.block(kBase, 0, kArm, 3).array() += 1.0;

  Eigen::MatrixXd grad = Eigen::MatrixXd::Zero(kTrajDim, 3);
  const double weight = 1.0e6;
  hardWaypointGradCost(points, hard, kBase, kArm, weight, grad);
  // A competing smoothness term pulls the waypoints further by a constant
  // force; the hard weight must dominate.
  const double competing_force = 1.0e3;
  grad.block(kBase, 0, kArm, 3).array() += competing_force;

  const double lr = 1.0 / (2.0 * weight);
  points -= lr * grad;

  double max_error = 0.0;
  for (int r = kBase; r < kTrajDim; ++r)
    for (int c = 0; c < 3; ++c)
      max_error = std::max(max_error, std::fabs(points(r, c) - hard(r, c)));
  EXPECT_LT(max_error, 1e-3);
}

TEST(HardWaypointStatus, DetectsArmDriftAndIgnoresBase) {
  const Eigen::MatrixXd hard = makeHard();
  Eigen::MatrixXd junctions = Eigen::MatrixXd::Zero(kTrajDim, hard.cols() + 2);
  junctions.block(kBase, 1, kArm, hard.cols()) = hard.block(kBase, 0, kArm, hard.cols());
  // Base rows intentionally differ; they are not hard waypoints.
  junctions.block(0, 1, kBase, hard.cols()).setConstant(5.0);
  EXPECT_TRUE(hardWaypointsSatisfied(junctions, hard, kBase, kArm, 1e-3));

  junctions(kBase, 1) += 2e-3;
  EXPECT_FALSE(hardWaypointsSatisfied(junctions, hard, kBase, kArm, 1e-3));
}

TEST(HardWaypointStatus, RejectsShapeMismatch) {
  const Eigen::MatrixXd hard = makeHard();
  Eigen::MatrixXd junctions = Eigen::MatrixXd::Zero(kTrajDim, hard.cols() + 1);
  EXPECT_FALSE(hardWaypointsSatisfied(junctions, hard, kBase, kArm, 1e-3));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
