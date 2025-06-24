#include "clock_follow_launch/vel_relay_node.hpp"

namespace clock_follow_launch
{

VelRelayNode::VelRelayNode(const rclcpp::NodeOptions & options)
: Node("vel_relay", options)
{
  // Declare and get the vel_topic parameter
  this->declare_parameter("vel_topic", "/turtle1/cmd_vel");
  vel_topic_ = this->get_parameter("vel_topic").as_string();
  
  // Declare and get the pose_topic parameter
  this->declare_parameter("pose_topic", "/turtle1/pose");
  pose_topic_ = this->get_parameter("pose_topic").as_string();
  
  RCLCPP_INFO(this->get_logger(), "Relaying /cmd_vel to %s", vel_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Transforming poses from %s to /robot_unit_pose", pose_topic_.c_str());
  
  // Create subscriber for /cmd_vel
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    10,
    std::bind(&VelRelayNode::cmd_vel_callback, this, std::placeholders::_1)
  );
  
  // Create publisher for the target topic
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(vel_topic_, 10);
  
  // Create subscriber for pose topic
  pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
    pose_topic_,
    10,
    std::bind(&VelRelayNode::pose_callback, this, std::placeholders::_1)
  );
  
  // Create publisher for transformed pose
  pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/robot_unit_pose", 10);
}

void VelRelayNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Simply relay the message to the target topic
  vel_pub_->publish(*msg);
}

void VelRelayNode::pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
{
  // Create transformed pose message
  auto transformed_pose = geometry_msgs::msg::Pose();
  
  // Apply transformation: x' = 3/11 * (x - 5.5), y' = 3/11 * (y - 5.5)
  transformed_pose.position.x = (3.0 / 11.0) * (msg->x - 5.5);
  transformed_pose.position.y = (3.0 / 11.0) * (msg->y - 5.5);
  transformed_pose.position.z = 0.0;
  
  // Convert theta to quaternion orientation
  transformed_pose.orientation.x = 0.0;
  transformed_pose.orientation.y = 0.0;
  transformed_pose.orientation.z = std::sin(msg->theta / 2.0);
  transformed_pose.orientation.w = std::cos(msg->theta / 2.0);
  
  // Publish the transformed pose
  pose_pub_->publish(transformed_pose);
}

}  // namespace clock_follow_launch