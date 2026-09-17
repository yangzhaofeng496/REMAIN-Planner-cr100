#include <ir100_goal_adapter/target_validator.hpp>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

class Ir100GoalAdapter {
 public:
  Ir100GoalAdapter() : nh_(), pnh_("~"), have_target_(false) {
    pnh_.param("target_topic", target_topic_, std::string("/ir100/end_effector_target"));
    pnh_.param("planner_topic", planner_topic_, std::string("/clicked_point"));
    pnh_.param("accepted_frame", accepted_frame_, std::string("world"));
    pnh_.param("max_abs_xyz", max_abs_xyz_, 10.0);
    target_sub_ = nh_.subscribe(target_topic_, 1, &Ir100GoalAdapter::targetCallback, this);
    planner_pub_ = nh_.advertise<geometry_msgs::PointStamped>(planner_topic_, 1, true);
    status_pub_ = pnh_.advertise<diagnostic_msgs::DiagnosticArray>("status", 1, true);
    state_pub_ = pnh_.advertise<std_msgs::String>("state", 1, true);
    clear_srv_ = pnh_.advertiseService("clear_target", &Ir100GoalAdapter::clearCallback, this);
    publishState("IDLE", diagnostic_msgs::DiagnosticStatus::OK, "waiting for end-effector target");
  }

 private:
  void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::string reason;
    if (!ir100_goal_adapter::validateTarget(*msg, accepted_frame_, max_abs_xyz_, &reason)) {
      publishState("INVALID_TARGET", diagnostic_msgs::DiagnosticStatus::ERROR, reason);
      ROS_ERROR_STREAM("[ir100_goal_adapter] rejected target: " << reason);
      return;
    }
    geometry_msgs::PointStamped point;
    point.header = msg->header;
    point.point = msg->pose.position;
    planner_pub_.publish(point);
    last_target_ = *msg;
    have_target_ = true;
    const bool connected = planner_pub_.getNumSubscribers() > 0;
    publishState(connected ? "FORWARDED" : "PLANNER_UNAVAILABLE",
                 connected ? diagnostic_msgs::DiagnosticStatus::OK
                           : diagnostic_msgs::DiagnosticStatus::WARN,
                 connected ? "target forwarded to REMAIN planner"
                           : "target published but no planner subscriber is connected");
    ROS_INFO("[ir100_goal_adapter] forwarded target frame=%s xyz=(%.3f %.3f %.3f) subscribers=%u",
             point.header.frame_id.c_str(), point.point.x, point.point.y, point.point.z,
             planner_pub_.getNumSubscribers());
  }

  bool clearCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    have_target_ = false;
    planner_pub_.publish(geometry_msgs::PointStamped());
    publishState("IDLE", diagnostic_msgs::DiagnosticStatus::OK, "target cleared");
    response.success = true;
    response.message = "target cleared";
    return true;
  }

  void publishState(const std::string& state, int level, const std::string& message) {
    std_msgs::String state_msg;
    state_msg.data = state;
    state_pub_.publish(state_msg);
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "ir100_goal_adapter";
    status.level = level;
    status.message = message;
    status.values.resize(1);
    status.values[0].key = "state";
    status.values[0].value = state;
    array.status.push_back(status);
    status_pub_.publish(array);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber target_sub_;
  ros::Publisher planner_pub_, status_pub_, state_pub_;
  ros::ServiceServer clear_srv_;
  std::string target_topic_, planner_topic_, accepted_frame_;
  double max_abs_xyz_;
  bool have_target_;
  geometry_msgs::PoseStamped last_target_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ir100_goal_adapter");
  Ir100GoalAdapter adapter;
  ros::spin();
  return 0;
}
