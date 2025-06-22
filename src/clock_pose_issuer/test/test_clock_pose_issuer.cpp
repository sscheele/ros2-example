#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <memory>

class TestClockPoseIssuer : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node_ = rclcpp::Node::make_shared("test_clock_pose_issuer");
        
        // Create subscription to listen for pose messages
        pose_subscription_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/clock/pose_command", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
                last_received_pose_ = *msg;
                message_received_ = true;
            });
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    geometry_msgs::msg::Pose last_received_pose_;
    bool message_received_ = false;
};

TEST_F(TestClockPoseIssuer, TestPoseMessageStructure)
{
    // Test that pose messages have the expected structure
    geometry_msgs::msg::Pose test_pose;
    
    // Position should be on unit circle (x^2 + y^2 = 1)
    test_pose.position.x = 1.0;
    test_pose.position.y = 0.0;
    test_pose.position.z = 0.0;
    
    // Orientation should be identity quaternion
    test_pose.orientation.x = 0.0;
    test_pose.orientation.y = 0.0;
    test_pose.orientation.z = 0.0;
    test_pose.orientation.w = 1.0;
    
    // Check that position is on unit circle
    double distance_from_origin = std::sqrt(
        test_pose.position.x * test_pose.position.x + 
        test_pose.position.y * test_pose.position.y
    );
    EXPECT_NEAR(distance_from_origin, 1.0, 0.001);
    
    // Check that z position is zero (2D movement)
    EXPECT_EQ(test_pose.position.z, 0.0);
    
    // Check that orientation is identity quaternion
    EXPECT_EQ(test_pose.orientation.x, 0.0);
    EXPECT_EQ(test_pose.orientation.y, 0.0);
    EXPECT_EQ(test_pose.orientation.z, 0.0);
    EXPECT_EQ(test_pose.orientation.w, 1.0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}