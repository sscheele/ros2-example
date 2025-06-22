#include "clock_pose_issuer/clock_pose_issuer_node.hpp"

ClockPoseIssuer::ClockPoseIssuer() : Node("clock_pose_issuer")
{
    // Create publisher for clock pose commands
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
        "/clock/pose_command", 10);
    
    // Create timer that fires every minute (60 seconds)
    timer_ = this->create_wall_timer(
        std::chrono::seconds(60),
        std::bind(&ClockPoseIssuer::publish_clock_pose, this));
    
    RCLCPP_INFO(this->get_logger(), "Clock pose issuer node started");
    
    // Publish initial pose immediately
    publish_clock_pose();
}

void ClockPoseIssuer::publish_clock_pose()
{
    // Get current time
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    
    // Extract minutes (0-59)
    int minutes = tm.tm_min;
    
    // Convert minutes to angle (0 minutes = 0 radians, 30 minutes = π radians)
    // Clock goes clockwise, but standard math coordinates go counter-clockwise
    // So we need to adjust: angle = -2π * (minutes/60) + π/2 (to start at 12 o'clock)
    double angle = -2.0 * M_PI * (minutes / 60.0) + M_PI / 2.0;
    
    // Calculate position on unit circle
    double x = std::cos(angle);
    double y = std::sin(angle);
    
    // Create pose message
    geometry_msgs::msg::Pose pose_msg;
    
    // Set position (z = 0 for 2D movement on unit circle)
    pose_msg.position.x = x;
    pose_msg.position.y = y;
    pose_msg.position.z = 0.0;
    
    // Set orientation (facing forward, no rotation)
    pose_msg.orientation.x = 0.0;
    pose_msg.orientation.y = 0.0;
    pose_msg.orientation.z = 0.0;
    pose_msg.orientation.w = 1.0;
    
    // Publish the pose
    pose_publisher_->publish(pose_msg);
    
    RCLCPP_INFO(this->get_logger(), 
        "Published clock pose for minute %d: x=%.3f, y=%.3f", 
        minutes, x, y);
}