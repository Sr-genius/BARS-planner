// command_bridge.cpp
//
// Adaptation point between the planner and the flight controller. The planner
// publishes a quadrotor_msgs/PositionCommand carrying position, velocity,
// acceleration and yaw. This node shows how to translate that command into
// standard ROS messages that any controller can consume:
//
//   - geometry_msgs/PoseStamped: position and yaw (as a quaternion)
//   - geometry_msgs/Twist:       linear velocity and yaw rate
//
// If your autopilot already accepts quadrotor_msgs/PositionCommand, this node
// is not needed. Otherwise replace the publish section below with the message
// or service call used by your platform, keeping the same values.
#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class CommandBridge {
 public:
  CommandBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("command_in", in_topic_, "position_command");
    pnh.param<std::string>("pose_out", pose_topic_, "command_pose");
    pnh.param<std::string>("twist_out", twist_topic_, "command_twist");
    pnh.param<std::string>("frame_id", frame_id_, "world");

    sub_ = nh.subscribe(in_topic_, 1, &CommandBridge::commandCb, this);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(pose_topic_, 1);
    twist_pub_ = nh.advertise<geometry_msgs::Twist>(twist_topic_, 1);

    ROS_INFO_STREAM("[command_bridge] " << in_topic_ << " -> "
                    << pose_topic_ << ", " << twist_topic_);
  }

 private:
  void commandCb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    // Discard non-finite setpoints so an invalid value never reaches the
    // controller; the last valid command is retained.
    const double values[10] = {
        msg->position.x, msg->position.y, msg->position.z,
        msg->velocity.x, msg->velocity.y, msg->velocity.z,
        msg->acceleration.x, msg->acceleration.y, msg->acceleration.z,
        msg->yaw};
    for (double v : values) {
      if (!std::isfinite(v)) {
        ROS_WARN_THROTTLE(1.0, "[command_bridge] non-finite command dropped");
        return;
      }
    }

    // ---- platform-independent reference output ----
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;
    pose.pose.position = msg->position;

    double half_yaw = 0.5 * msg->yaw;
    pose.pose.orientation.z = std::sin(half_yaw);
    pose.pose.orientation.w = std::cos(half_yaw);
    pose_pub_.publish(pose);

    geometry_msgs::Twist twist;
    twist.linear = msg->velocity;
    twist.angular.z = msg->yaw_dot;
    twist_pub_.publish(twist);

    // ---- adapt to your platform here ----
    // Forward msg->position, msg->velocity, msg->acceleration and msg->yaw to
    // the controller's message or API. When the commanded velocity is zero,
    // send a position-only setpoint; otherwise send position and velocity so
    // the controller tracks the moving reference.
  }

  std::string in_topic_, pose_topic_, twist_topic_, frame_id_;
  ros::Subscriber sub_;
  ros::Publisher pose_pub_, twist_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "command_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  CommandBridge bridge(nh, pnh);
  ros::spin();
  return 0;
}
