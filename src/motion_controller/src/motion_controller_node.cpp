#include "motion_controller/motion_controller_node.hpp"
#include <cmath>

namespace motion_controller
{

MotionControllerNode::MotionControllerNode(const rclcpp::NodeOptions & options)
: Node("motion_controller", options),
  current_pose_received_(false),
  user_pose_active_(false),
  clock_pose_received_(false),
  user_timeout_seconds_(30.0)
{
  // Declare parameters
  this->declare_parameter("pose_topic", "/turtle1/pose");
  this->declare_parameter("controller_freq", 30.0);

  // Get parameters
  pose_topic_ = this->get_parameter("pose_topic").as_string();
  controller_freq_ = this->get_parameter("controller_freq").as_double();

  RCLCPP_INFO(this->get_logger(), "Motion Controller starting with pose_topic: %s, freq: %.1f Hz", 
              pose_topic_.c_str(), controller_freq_);

  // Initialize subscribers
  current_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
    pose_topic_, 10,
    std::bind(&MotionControllerNode::current_pose_callback, this, std::placeholders::_1));

  user_pose_command_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "/user/pose_command", 10,
    std::bind(&MotionControllerNode::user_pose_command_callback, this, std::placeholders::_1));

  clock_pose_command_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "/clock/pose_command", 10,
    std::bind(&MotionControllerNode::clock_pose_command_callback, this, std::placeholders::_1));

  user_pose_cancel_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/user/pose_cancel", 10,
    std::bind(&MotionControllerNode::user_pose_cancel_callback, this, std::placeholders::_1));

  // Initialize publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Initialize control timer
  auto timer_period = std::chrono::duration<double>(1.0 / controller_freq_);
  control_timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&MotionControllerNode::control_timer_callback, this));

  // Initialize time tracking
  last_control_time_ = this->now();
  last_user_command_time_ = rclcpp::Time(0);

  RCLCPP_INFO(this->get_logger(), "Motion Controller initialized successfully");
}

void MotionControllerNode::current_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
{
  current_pose_ = *msg;
  current_pose_received_ = true;
}

void MotionControllerNode::user_pose_command_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  user_pose_command_ = *msg;
  user_pose_active_ = true;
  last_user_command_time_ = this->now();
  
  // Reset PID states when switching to user command
  reset_pid_state(pid_x_state_);
  reset_pid_state(pid_y_state_);
  
  RCLCPP_INFO(this->get_logger(), "Received user pose command: (%.2f, %.2f)", 
              msg->position.x, msg->position.y);
}

void MotionControllerNode::clock_pose_command_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  clock_pose_command_ = *msg;
  clock_pose_received_ = true;
}

void MotionControllerNode::user_pose_cancel_callback(const std_msgs::msg::Empty::SharedPtr)
{
  user_pose_active_ = false;
  last_user_command_time_ = rclcpp::Time(0);
  
  // Reset PID states when canceling user command
  reset_pid_state(pid_x_state_);
  reset_pid_state(pid_y_state_);
  
  RCLCPP_INFO(this->get_logger(), "User pose command canceled, returning to clock following");
}

void MotionControllerNode::control_timer_callback()
{
  if (!current_pose_received_) {
    return;  // Wait for current pose
  }

  // Check if user command has timed out
  auto current_time = this->now();
  if (user_pose_active_ && 
      (current_time - last_user_command_time_).seconds() > user_timeout_seconds_) {
    user_pose_active_ = false;
    reset_pid_state(pid_x_state_);
    reset_pid_state(pid_y_state_);
    RCLCPP_INFO(this->get_logger(), "User command timed out, returning to clock following");
  }

  // Determine target pose
  if (user_pose_active_) {
    target_pose_ = user_pose_command_;
  } else if (clock_pose_received_) {
    target_pose_ = clock_pose_command_;
  } else {
    // No target available, stop the robot
    auto twist_msg = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(twist_msg);
    return;
  }

  // Compute and publish control command
  auto control_cmd = compute_control_command();
  cmd_vel_pub_->publish(control_cmd);
}

geometry_msgs::msg::Twist MotionControllerNode::compute_control_command()
{
  auto twist_msg = geometry_msgs::msg::Twist();
  // RCLCPP_INFO(this->get_logger(), "Current pose: %f %f", current_pose_.x, current_pose_.y);
  
  // Calculate position errors
  double error_x = target_pose_.position.x - current_pose_.x;
  double error_y = target_pose_.position.y - current_pose_.y;
  
  // Calculate time delta
  auto current_time = this->now();
  double dt = (current_time - last_control_time_).seconds();
  last_control_time_ = current_time;
  
  // Avoid division by zero or very small dt
  if (dt <= 0.0 || dt > 1.0) {
    dt = 1.0 / controller_freq_;
  }
  
  // Convert to robot frame (linear and angular velocities)
  // For simplicity, we'll use a basic approach where we compute linear velocity
  // in the direction of the target and angular velocity to orient towards target
  
  double distance_to_target = std::sqrt(error_x * error_x + error_y * error_y);
  
  if (distance_to_target > 0.01) {  // Only move if not at target
    // Desired angle to target
    double desired_theta = std::atan2(error_y, error_x);
    double angle_error = desired_theta - current_pose_.theta;
    
    // Normalize angle error to [-pi, pi]
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
    
    // Linear velocity proportional to distance
    twist_msg.linear.x = std::min(2.0, distance_to_target * 1.0);
    
    // Angular velocity proportional to angle error
    twist_msg.angular.z = angle_error * 2.0;
    
    // Limit velocities
    twist_msg.linear.x = std::max(-2.0, std::min(2.0, twist_msg.linear.x));
    twist_msg.angular.z = std::max(-2.0, std::min(2.0, twist_msg.angular.z));
  }
  
  return twist_msg;
}

double MotionControllerNode::compute_pid_output(double error, PIDState & state, 
                                               const PIDGains & gains, double dt)
{
  // Initialize if first time
  if (!state.initialized) {
    state.previous_error = error;
    state.integral = 0.0;
    state.initialized = true;
  }
  
  // Proportional term
  double proportional = gains.kp * error;
  
  // Integral term
  state.integral += error * dt;
  // Anti-windup: limit integral
  state.integral = std::max(-10.0, std::min(10.0, state.integral));
  double integral = gains.ki * state.integral;
  
  // Derivative term
  double derivative = gains.kd * (error - state.previous_error) / dt;
  state.previous_error = error;
  
  return proportional + integral + derivative;
}

void MotionControllerNode::reset_pid_state(PIDState & state)
{
  state.integral = 0.0;
  state.previous_error = 0.0;
  state.initialized = false;
}

}  // namespace motion_controller