#include "clock_follow_launch/vel_relay_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<clock_follow_launch::VelRelayNode>();
  
  RCLCPP_INFO(node->get_logger(), "Velocity relay node started");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}