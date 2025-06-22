#ifndef CLOCK_POSE_ISSUER_NODE_HPP
#define CLOCK_POSE_ISSUER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <cmath>

class ClockPoseIssuer : public rclcpp::Node
{
public:
    ClockPoseIssuer();

private:
    void publish_clock_pose();
    
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // CLOCK_POSE_ISSUER_NODE_HPP