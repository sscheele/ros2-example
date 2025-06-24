#include "motion_controller/motion_controller.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<motion_controller::MotionControllerNode>();
  
  RCLCPP_INFO(node->get_logger(), "Motion Controller node started");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}