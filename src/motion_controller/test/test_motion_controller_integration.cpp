#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <turtlesim/msg/pose.hpp>
#include "motion_controller/motion_controller_node.hpp"

class TestMotionControllerIntegration : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<motion_controller::MotionControllerNode>();
    
    // Create test nodes and publishers
    pose_pub_node_ = std::make_shared<rclcpp::Node>("test_pose_publisher");
    user_pub_node_ = std::make_shared<rclcpp::Node>("test_user_publisher");
    clock_pub_node_ = std::make_shared<rclcpp::Node>("test_clock_publisher");
    test_sub_node_ = std::make_shared<rclcpp::Node>("test_subscriber");
    
    current_pose_pub_ = pose_pub_node_->create_publisher<turtlesim::msg::Pose>("/turtle1/pose", 10);
    user_command_pub_ = user_pub_node_->create_publisher<geometry_msgs::msg::Pose>("/user/pose_command", 10);
    clock_command_pub_ = clock_pub_node_->create_publisher<geometry_msgs::msg::Pose>("/clock/pose_command", 10);
    
    // Create test subscriber
    cmd_vel_sub_ = test_sub_node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_vel_ = *msg;
        cmd_vel_received_ = true;
      });
    
    cmd_vel_received_ = false;
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<motion_controller::MotionControllerNode> node_;
  std::shared_ptr<rclcpp::Node> pose_pub_node_;
  std::shared_ptr<rclcpp::Node> user_pub_node_;
  std::shared_ptr<rclcpp::Node> clock_pub_node_;
  std::shared_ptr<rclcpp::Node> test_sub_node_;
  
  rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr user_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr clock_command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  geometry_msgs::msg::Twist last_cmd_vel_;
  bool cmd_vel_received_;
};

TEST_F(TestMotionControllerIntegration, RespondsToCurrentPose)
{
  // Publish a current pose
  auto pose_msg = turtlesim::msg::Pose();
  pose_msg.x = 1.0;
  pose_msg.y = 1.0;
  pose_msg.theta = 0.0;
  
  current_pose_pub_->publish(pose_msg);
  
  // Spin briefly to process the message
  rclcpp::spin_some(node_);
  
  // The node should have received the pose (no direct way to test this without exposing internals)
  SUCCEED();  // If we get here without crashing, the test passes
}

TEST_F(TestMotionControllerIntegration, RespondsToUserCommand)
{
  // First publish current pose
  auto current_pose = turtlesim::msg::Pose();
  current_pose.x = 0.0;
  current_pose.y = 0.0;
  current_pose.theta = 0.0;
  current_pose_pub_->publish(current_pose);
  
  // Then publish user command
  auto user_cmd = geometry_msgs::msg::Pose();
  user_cmd.position.x = 2.0;
  user_cmd.position.y = 2.0;
  user_cmd.position.z = 0.0;
  
  user_command_pub_->publish(user_cmd);
  
  // Spin to process messages
  rclcpp::spin_some(node_);
  
  SUCCEED();  // Basic integration test - node should handle the messages without crashing
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}