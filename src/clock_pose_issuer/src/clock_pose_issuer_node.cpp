#include "clock_pose_issuer/clock_pose_issuer_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClockPoseIssuer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}