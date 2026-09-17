#include <gtest/gtest.h>
#include <ir100_goal_adapter/target_validator.hpp>
#include <limits>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(TargetValidator, AcceptsFiniteWorldTarget) {
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "world";
  msg.pose.position.x = 0.8;
  msg.pose.position.y = -0.2;
  msg.pose.position.z = 1.1;
  std::string reason;
  EXPECT_TRUE(ir100_goal_adapter::validateTarget(msg, "world", 10.0, &reason));
  EXPECT_TRUE(reason.empty());
}

TEST(TargetValidator, RejectsWrongFrameAndNonFinitePosition) {
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "base_link";
  msg.pose.position.x = 0.2;
  std::string reason;
  EXPECT_FALSE(ir100_goal_adapter::validateTarget(msg, "world", 10.0, &reason));
  EXPECT_EQ(reason, "unexpected frame_id");
  msg.header.frame_id = "world";
  msg.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ir100_goal_adapter::validateTarget(msg, "world", 10.0, &reason));
  EXPECT_EQ(reason, "non-finite position");
}

TEST(TargetValidator, RejectsOutOfBoundsPosition) {
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "world";
  msg.pose.position.x = 2.0;
  std::string reason;
  EXPECT_FALSE(ir100_goal_adapter::validateTarget(msg, "world", 1.0, &reason));
  EXPECT_EQ(reason, "position outside configured bounds");
}
