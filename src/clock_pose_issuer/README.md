# Clock Pose Issuer

A ROS2 node that publishes 6D poses according to clock time, following the minute hand position on a unit circle.

## Overview

The `clock_pose_issuer` node converts the current time (specifically the minute component) to a position on the unit circle and publishes it as a geometry_msgs/Pose message. This provides a time-based reference pose that other nodes can use for robot positioning.

## Features

- Publishes poses every minute based on system time
- Maps minutes (0-59) to positions on a unit circle
- Lightweight C++ implementation for minimal resource usage
- Comprehensive test suite with unit and integration tests
- Launch file for easy deployment

## Topics

### Published Topics

- `/clock/pose_command` (geometry_msgs/msg/Pose): The current clock-based pose position

## Time to Position Mapping

The node maps the current minute to a position on the unit circle as follows:

- 0 minutes → 12 o'clock position (0, 1)
- 15 minutes → 3 o'clock position (1, 0)
- 30 minutes → 6 o'clock position (0, -1)
- 45 minutes → 9 o'clock position (-1, 0)

The position follows the equation:
```
angle = -2π * (minutes/60) + π/2
x = cos(angle)
y = sin(angle)
z = 0
```

## Usage

### Building

```bash
colcon build --packages-select clock_pose_issuer
```

### Running

#### Using the launch file (recommended):
```bash
ros2 launch clock_pose_issuer clock_pose_issuer.launch.py
```

#### Running directly:
```bash
ros2 run clock_pose_issuer clock_pose_issuer_node
```

### Testing

Run the test suite:
```bash
colcon test --packages-select clock_pose_issuer
```

View test results:
```bash
colcon test-result --verbose
```

## Node Details

- **Node Name**: `clock_pose_issuer`
- **Executable**: `clock_pose_issuer_node`
- **Language**: C++
- **Update Rate**: Every 60 seconds (1 minute)

## Dependencies

- rclcpp
- geometry_msgs
- ament_cmake (build)
- ament_cmake_gtest (testing)

## Architecture

The node uses a simple timer-based approach:
1. Timer fires every 60 seconds
2. Current system time is retrieved
3. Minute component is extracted (0-59)
4. Position on unit circle is calculated
5. Pose message is published

This design ensures consistent, predictable behavior that other nodes can rely on for time-based positioning.