#ifndef CLOCK_FOLLOW_LAUNCH__VEL_RELAY_NODE_HPP_
#define CLOCK_FOLLOW_LAUNCH__VEL_RELAY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <string>

namespace clock_follow_launch
{

class VelRelayNode : public rclcpp::Node
{
public:
  explicit VelRelayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg);
  
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr pose_pub_;
  
  std::string vel_topic_;
  std::string pose_topic_;
};

}  // namespace clock_follow_launch

#endif  // CLOCK_FOLLOW_LAUNCH__VEL_RELAY_NODE_HPP_