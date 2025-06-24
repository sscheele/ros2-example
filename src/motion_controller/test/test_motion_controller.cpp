#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "motion_controller/motion_controller.hpp"

class TestMotionController : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestMotionController, NodeCreation)
{
  auto node = std::make_shared<motion_controller::MotionControllerNode>();
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->get_name(), std::string("motion_controller"));
}

TEST_F(TestMotionController, ParameterDefaults)
{
  auto node = std::make_shared<motion_controller::MotionControllerNode>();
  
  // Check default parameters
  auto pose_topic = node->get_parameter("pose_topic").as_string();
  auto controller_freq = node->get_parameter("controller_freq").as_double();
  
  EXPECT_EQ(pose_topic, "/turtle1/pose");
  EXPECT_DOUBLE_EQ(controller_freq, 30.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}