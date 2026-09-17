#pragma once

#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <string>

namespace ir100_goal_adapter {

inline bool validateTarget(const geometry_msgs::PoseStamped& msg,
                           const std::string& expected_frame,
                           double max_abs_xyz,
                           std::string* reason) {
  const auto fail = [reason](const std::string& text) {
    if (reason) *reason = text;
    return false;
  };
  if (msg.header.frame_id.empty()) return fail("empty frame_id");
  if (!expected_frame.empty() && msg.header.frame_id != expected_frame)
    return fail("unexpected frame_id");
  const double x = msg.pose.position.x;
  const double y = msg.pose.position.y;
  const double z = msg.pose.position.z;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    return fail("non-finite position");
  if (max_abs_xyz > 0.0 &&
      (std::abs(x) > max_abs_xyz || std::abs(y) > max_abs_xyz ||
       std::abs(z) > max_abs_xyz))
    return fail("position outside configured bounds");
  return true;
}

}  // namespace ir100_goal_adapter
