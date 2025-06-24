#ifndef MOTION_CONTROLLER__MOTION_CONTROLLER_NODE_HPP_
#define MOTION_CONTROLLER__MOTION_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <turtlesim/msg/pose.hpp>
#include <chrono>

namespace motion_controller
{

class MotionControllerNode : public rclcpp::Node
{
public:
  explicit MotionControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callback functions
  void current_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void user_pose_command_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void clock_pose_command_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void user_pose_cancel_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void control_timer_callback();

  // Control functions
  geometry_msgs::msg::Twist compute_control_command();

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr user_pose_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr clock_pose_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr user_pose_cancel_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State variables
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose target_pose_;
  geometry_msgs::msg::Pose user_pose_command_;
  geometry_msgs::msg::Pose clock_pose_command_;
  
  bool current_pose_received_;
  bool user_pose_active_;
  bool clock_pose_received_;
  
  rclcpp::Time last_user_command_time_;
  rclcpp::Time last_control_time_;
  
  // Parameters
  std::string pose_topic_;
  double controller_freq_;
  double user_timeout_seconds_;
};

}  // namespace motion_controller

#endif  // MOTION_CONTROLLER__MOTION_CONTROLLER_NODE_HPP_