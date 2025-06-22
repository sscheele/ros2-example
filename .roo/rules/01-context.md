This repository contains a prototype ROS2 stack for a robot that follows pose inputs from two different sources. The robot should take a desired pose and then use a motion controller to get to the desired pose.

Imagine you're building infrastructure for someone else to plug into later -- it should be simple, easy to interpret and build on top of, and have things like a readme, launch system and test scaffolding.

# Nodes

## clock_pose_issuer
publishes 6D poses according to clock time

This node is implemented in C++ to keep it lightweight.

If no user input is coming from the `gui_pose_issuer`, the robot's position should follow the minute hand of the clock. `clock_pose_issuer` publishes the appropriate 6D pose for the robot every minute, converting the current time to a position on the unit circle.

## gui_pose_issuer
publishes 6D poses according to user desire

This node is implemented in Python to take advantage of more convenient GUI libraries

A graphical user interface that allows a human driver to click a position on a clock, which commands the robot to go to that pose, or to click the space bar to forget the last gui command and return to clock following. 

## motion_controller
Implements a simple PID controller and muxing between the user commanded position from `gui_pose_issuer` and the clock position from `clock_pose_issuer`

This node is implemented in C++ because that's what you're supposed to write controllers in.

If the human has not issued a pose in 30 seconds, the robot should return to clock following.

# Topics
- `/user/pose_command` - geometry_msgs/msg/Pose
- `/user/pose_cancel` - std_msgs/msg/Empty
- `/clock/pose_command` - geometry_msgs/msg/Pose
- `/cmd_vel` - geometry_msgs/msg/Twist