# Clock Follow Launch

This package provides a single launch file that starts all components of the clock following robot system.

## Overview

The `clock_follow.launch.py` file launches all the necessary nodes and components:

1. **motion_controller** - PID controller that manages robot movement and muxes between clock and user commands
2. **turtlesim_node** - The turtle simulation environment
3. **vel_relay_node** - Relays velocity commands from `/cmd_vel` to a configurable target topic
4. **clock_pose_issuer** - Publishes poses based on clock time (minute hand position)
5. **gui_pose_issuer** - Provides GUI interface for user to command robot poses

## Usage

To launch the entire system:

```bash
ros2 launch clock_follow_launch clock_follow.launch.py
```

To launch with a custom velocity topic:

```bash
ros2 launch clock_follow_launch clock_follow.launch.py vel_topic:=/my_robot/cmd_vel
```

This single command replaces the need to run these individual commands:

```bash
ros2 launch motion_controller motion_controller.launch.py
ros2 run turtlesim turtlesim_node
ros2 launch clock_pose_issuer clock_pose_issuer.launch.py
ros2 launch gui_pose_issuer gui_pose_issuer.launch.py
```

## System Behavior

- The robot will follow the clock by default, moving to positions corresponding to the minute hand
- Users can click on the GUI to command the robot to specific positions
- If no user input is received for 30 seconds, the robot returns to clock following mode
- Press spacebar in the GUI to cancel user commands and return to clock following immediately
- The velocity relay node forwards all commands from `/cmd_vel` to the specified target topic (default: `/turtle1/cmd_vel`)

## Launch Parameters

- `vel_topic` (string, default: `/turtle1/cmd_vel`): Target topic for relaying velocity commands

## Dependencies

- motion_controller
- clock_pose_issuer  
- gui_pose_issuer
- turtlesim