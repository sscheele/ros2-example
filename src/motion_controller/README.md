# Motion Controller

A ROS2 C++ node that implements a simple motion controller with pose command muxing for robot motion control.

## Overview

The motion controller subscribes to pose commands from two sources:
- User commands from `gui_pose_issuer` via `/user/pose_command`
- Clock-based commands from `clock_pose_issuer` via `/clock/pose_command`

It implements intelligent muxing between these sources:
- User commands take priority when active
- If no user command for 30 seconds, switches to clock following
- User can explicitly cancel commands via `/user/pose_cancel`

## Features

- **Proportional Control**: Simple proportional controller for smooth motion to target poses
- **Command Muxing**: Intelligent switching between user and clock commands
- **Timeout Handling**: Automatic fallback to clock following after 30 seconds
- **Configurable Parameters**: Customizable pose topic and controller frequency
- **Orientation Agnostic**: Focuses on position control only (as specified)

## Topics

### Subscribed Topics
- `/user/pose_command` (geometry_msgs/msg/Pose) - User-commanded target poses
- `/user/pose_cancel` (std_msgs/msg/Empty) - Cancel user commands
- `/clock/pose_command` (geometry_msgs/msg/Pose) - Clock-based target poses
- `pose_topic` parameter (geometry_msgs/msg/Pose) - Current robot pose (default: `/robot_pose`)

### Published Topics
- `/cmd_vel` (geometry_msgs/msg/Twist) - Velocity commands for robot motion

## Parameters

- `pose_topic` (string, default: "/robot_pose"): Topic name for current robot pose
- `controller_freq` (double, default: 30.0): Controller frequency in Hz

## Usage

### Launch the node
```bash
ros2 launch motion_controller motion_controller.launch.py
```

### Launch with custom parameters
```bash
ros2 launch motion_controller motion_controller.launch.py pose_topic:=/my_robot/pose controller_freq:=50.0
```

### Run the node directly
```bash
ros2 run motion_controller motion_controller
```

## Implementation Details

### Motion Controller
- Uses proportional control for both linear and angular velocities
- Linear velocity proportional to distance to target
- Angular velocity proportional to heading error
- Velocity limits applied for safety

### Control Strategy
1. Calculate position error (target - current)
2. Compute desired heading angle to target
3. Generate linear velocity proportional to distance
4. Generate angular velocity proportional to heading error
5. Apply velocity limits for safety

### State Management
- Tracks user command timeout (30 seconds)
- Handles switching between command sources smoothly
- Handles missing pose data gracefully

## Testing

Run unit tests:
```bash
colcon test --packages-select motion_controller
```

## Dependencies

- rclcpp
- geometry_msgs
- std_msgs
- turtlesim