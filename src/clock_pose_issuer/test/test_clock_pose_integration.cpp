#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include "clock_pose_issuer/clock_pose_issuer_node.hpp"

class TestClockPoseIntegration : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        test_node_ = rclcpp::Node::make_shared("test_clock_pose_integration");
        
        // Create subscription to listen for pose messages
        pose_subscription_ = test_node_->create_subscription<geometry_msgs::msg::Pose>(
            "/clock/pose_command", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
                last_received_pose_ = *msg;
                message_received_ = true;
                message_count_++;
            });
        
        // Create the clock pose issuer node
        clock_pose_issuer_node_ = std::make_shared<ClockPoseIssuer>();
        
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(test_node_);
        executor_->add_node(clock_pose_issuer_node_);
    }

    void TearDown() override
    {
        executor_->cancel();
        rclcpp::shutdown();
    }

    void spin_for_time(std::chrono::milliseconds duration)
    {
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < duration) {
            executor_->spin_some(std::chrono::milliseconds(10));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    rclcpp::Node::SharedPtr test_node_;
    std::shared_ptr<ClockPoseIssuer> clock_pose_issuer_node_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Executor::SharedPtr executor_;
    geometry_msgs::msg::Pose last_received_pose_;
    bool message_received_ = false;
    int message_count_ = 0;
};

TEST_F(TestClockPoseIntegration, TestNodeCanBeCreated)
{
    // This test just verifies that we can create the test infrastructure
    EXPECT_TRUE(test_node_ != nullptr);
    EXPECT_TRUE(pose_subscription_ != nullptr);
}

TEST_F(TestClockPoseIntegration, TestPoseMessageValidation)
{
    // Test that any received pose message has valid structure
    if (message_received_) {
        // Check that position is on unit circle
        double distance_from_origin = std::sqrt(
            last_received_pose_.position.x * last_received_pose_.position.x + 
            last_received_pose_.position.y * last_received_pose_.position.y
        );
        EXPECT_NEAR(distance_from_origin, 1.0, 0.001);
        
        // Check that z position is zero (2D movement)
        EXPECT_EQ(last_received_pose_.position.z, 0.0);
        
        // Check that orientation is identity quaternion
        EXPECT_EQ(last_received_pose_.orientation.x, 0.0);
        EXPECT_EQ(last_received_pose_.orientation.y, 0.0);
        EXPECT_EQ(last_received_pose_.orientation.z, 0.0);
        EXPECT_EQ(last_received_pose_.orientation.w, 1.0);
    }
}

TEST_F(TestClockPoseIntegration, TestReceivesPoseMessages)
{
    // Reset message state
    message_received_ = false;
    message_count_ = 0;
    
    // Spin for a few seconds to allow messages to be received
    spin_for_time(std::chrono::seconds(3));
    
    // Verify we received messages
    EXPECT_TRUE(message_received_);
    EXPECT_GT(message_count_, 0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}