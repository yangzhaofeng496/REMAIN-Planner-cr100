#include <gtest/gtest.h>
#include <path_searching/sample_mani_RRT.h>

using mani_sample::LayerIkCandidate;
using mani_sample::LayerIkStats;
using mani_sample::ManiCollisionFn;
using mani_sample::ManiIkFn;
using mani_sample::acceptIkCandidate;
using mani_sample::sampleLayerIkCandidates;

namespace {
constexpr int kDof = 6;

ManiIkFn constantIk(double value) {
  return [value](const Eigen::Vector3d &, const Eigen::Matrix3d &,
                 const Eigen::VectorXd &, Eigen::VectorXd &out) {
    out = Eigen::VectorXd::Constant(kDof, value);
    return true;
  };
}
}  // namespace

TEST(LayerCandidateGeneration, BoundsAndCounts) {
  const Eigen::Vector3d center(1.0, 2.0, 0.5);
  const Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  const Eigen::VectorXd seed = Eigen::VectorXd::Constant(kDof, 0.0);
  const Eigen::VectorXd lo = Eigen::VectorXd::Constant(kDof, -2.0);
  const Eigen::VectorXd hi = Eigen::VectorXd::Constant(kDof, 2.0);
  std::vector<LayerIkCandidate> out;
  LayerIkStats stats;
  ASSERT_TRUE(sampleLayerIkCandidates(center, rot, seed, kDof, lo, hi, 40,
                                      0.3, 1.0, 1.8, 1234u, constantIk(0.1),
                                      out, stats));
  EXPECT_EQ(out.size(), 40u);
  EXPECT_EQ(stats.accepted, 40);
  EXPECT_EQ(stats.ik_failure, 0);
  for (const auto &c : out) {
    EXPECT_LE(std::abs(c.ee_position.x() - center.x()), 0.3 + 1e-9);
    EXPECT_LE(std::abs(c.ee_position.y() - center.y()), 0.3 + 1e-9);
    EXPECT_GE(c.ee_position.z(), 1.0 - 1e-9);
    EXPECT_LE(c.ee_position.z(), 1.8 + 1e-9);
  }
}

TEST(LayerCandidateGeneration, DeterministicForFixedSeed) {
  const Eigen::Vector3d center(0.2, -0.1, 0.6);
  const Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  const Eigen::VectorXd seed = Eigen::VectorXd::Zero(kDof);
  const Eigen::VectorXd lo = Eigen::VectorXd::Constant(kDof, -3.0);
  const Eigen::VectorXd hi = Eigen::VectorXd::Constant(kDof, 3.0);
  std::vector<LayerIkCandidate> a, b;
  LayerIkStats sa, sb;
  ASSERT_TRUE(sampleLayerIkCandidates(center, rot, seed, kDof, lo, hi, 25, 0.25,
                                      1.1, 1.7, 99u, constantIk(0.2), a, sa));
  ASSERT_TRUE(sampleLayerIkCandidates(center, rot, seed, kDof, lo, hi, 25, 0.25,
                                      1.1, 1.7, 99u, constantIk(0.2), b, sb));
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_TRUE(a[i].ee_position.isApprox(b[i].ee_position));
    EXPECT_TRUE(a[i].joint_state.isApprox(b[i].joint_state));
  }
}

TEST(LayerCandidateGeneration, RejectsOutOfLimitIkSolutions) {
  const Eigen::Vector3d center(0.0, 0.0, 0.5);
  const Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  const Eigen::VectorXd seed = Eigen::VectorXd::Zero(kDof);
  const Eigen::VectorXd lo = Eigen::VectorXd::Constant(kDof, -0.5);
  const Eigen::VectorXd hi = Eigen::VectorXd::Constant(kDof, 0.5);
  std::vector<LayerIkCandidate> out;
  LayerIkStats stats;
  EXPECT_FALSE(sampleLayerIkCandidates(center, rot, seed, kDof, lo, hi, 10, 0.1,
                                       1.0, 1.2, 7u, constantIk(5.0), out, stats));
  EXPECT_TRUE(out.empty());
  EXPECT_EQ(stats.out_of_limits, 10);
  EXPECT_EQ(stats.accepted, 0);
}

TEST(LayerCandidateGeneration, CountsIkFailures) {
  const Eigen::Vector3d center(0.0, 0.0, 0.5);
  const Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  const Eigen::VectorXd seed = Eigen::VectorXd::Zero(kDof);
  const Eigen::VectorXd lo = Eigen::VectorXd::Constant(kDof, -2.0);
  const Eigen::VectorXd hi = Eigen::VectorXd::Constant(kDof, 2.0);
  ManiIkFn failing = [](const Eigen::Vector3d &, const Eigen::Matrix3d &,
                        const Eigen::VectorXd &, Eigen::VectorXd &) {
    return false;
  };
  std::vector<LayerIkCandidate> out;
  LayerIkStats stats;
  EXPECT_FALSE(sampleLayerIkCandidates(center, rot, seed, kDof, lo, hi, 8, 0.1,
                                       1.0, 1.2, 3u, failing, out, stats));
  EXPECT_EQ(stats.ik_failure, 8);
  EXPECT_EQ(stats.accepted, 0);
}

// The layer generator plus the single acceptance gate must yield only
// full-collision-free candidates.
TEST(LayerCandidateGeneration, AcceptedCandidatesAreCollisionFree) {
  const Eigen::Vector3d center(0.0, 0.0, 0.5);
  const Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  const Eigen::VectorXd seed = Eigen::VectorXd::Zero(kDof);
  const Eigen::VectorXd lo = Eigen::VectorXd::Constant(kDof, -2.0);
  const Eigen::VectorXd hi = Eigen::VectorXd::Constant(kDof, 2.0);
  std::vector<LayerIkCandidate> generated;
  LayerIkStats stats;
  ASSERT_TRUE(sampleLayerIkCandidates(center, rot, seed, kDof, lo, hi, 30, 0.2,
                                      1.0, 1.4, 42u, constantIk(0.6), generated,
                                      stats));
  ASSERT_FALSE(generated.empty());

  std::array<size_t, 4> counts{{0, 0, 0, 0}};
  size_t ik_failures = 0;
  // Pretend the first joint over 0.5 is in arm-cloud collision.
  ManiCollisionFn collision = [&](const Eigen::Vector3d &, const Eigen::VectorXd &q,
                                  int &type) {
    if (q(0) > 0.5) {
      type = 1;
      return true;
    }
    type = -1;
    return false;
  };
  int accepted = 0;
  for (const auto &c : generated) {
    int collision_type = -1;
    if (acceptIkCandidate(true, Eigen::Vector3d::Zero(), c.joint_state, collision,
                          collision_type, &counts, &ik_failures)) {
      ++accepted;
    }
  }
  // Every generated joint has q(0) == 0.6 > 0.5, so all must be rejected.
  EXPECT_EQ(accepted, 0);
  EXPECT_EQ(counts[1], generated.size());
  EXPECT_EQ(counts[0] + counts[2] + counts[3], 0u);
  EXPECT_EQ(ik_failures, 0u);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
