# GUI Pose Issuer

A ROS2 Python package that provides a graphical user interface for commanding robot poses by clicking on a clock interface.

## Overview

The `gui_pose_issuer` node creates a simple Tkinter-based GUI with a clock face that allows users to:
- Click anywhere on the clock to command the robot to move to that position
- Press spacebar to cancel the current command and return to clock-following mode

## Features

- **Simple Clock Interface**: Circular clock with 12 hour markers and minimal styling
- **Click-to-Command**: Click anywhere on the clock circle to set robot target position
- **Spacebar Cancel**: Press spacebar to cancel user commands
- **ROS2 Integration**: Publishes standard ROS2 geometry messages
- **Visual Feedback**: Shows a red dot where you clicked

## Topics

### Published Topics

- `/user/pose_command` (`geometry_msgs/msg/Pose`): Published when user clicks on clock
- `/user/pose_cancel` (`std_msgs/msg/Empty`): Published when user presses spacebar

## Coordinate System

The clock uses a standard unit circle coordinate system:
- 12 o'clock = (0, 1)
- 3 o'clock = (1, 0) 
- 6 o'clock = (0, -1)
- 9 o'clock = (-1, 0)

This matches the coordinate system used by the `clock_pose_issuer` node.

## Usage

### Running the Node

```bash
# Run directly
ros2 run gui_pose_issuer gui_pose_issuer_node

# Or use the launch file
ros2 launch gui_pose_issuer gui_pose_issuer.launch.py
```

### GUI Controls

1. **Mouse Click**: Click anywhere on the clock circle to command the robot to that position
2. **Spacebar**: Press spacebar to cancel the current user command and return to automatic clock following

### Requirements

- Python 3.8+
- ROS2 (tested with Humble)
- Tkinter (usually included with Python)
- X11 forwarding (if running in Docker)

## Architecture

The package consists of three main components:

### `GuiPoseIssuer` (ROS2 Node)
- Main ROS2 node that handles message publishing
- Integrates Tkinter GUI with ROS2 spinning
- Publishes pose commands and cancel messages

### `ClockWidget` (UI Component)
- Tkinter-based clock interface
- Handles mouse clicks and keyboard events
- Provides visual feedback for user interactions

### `PoseCalculator` (Utility)
- Converts pixel coordinates to unit circle positions
- Handles coordinate system transformations
- Provides geometry utility functions

## Testing

Run the test suite:

```bash
# Run all tests
colcon test --packages-select gui_pose_issuer

# Run specific test files
python3 -m pytest src/gui_pose_issuer/test/test_pose_calculator.py
python3 -m pytest src/gui_pose_issuer/test/test_gui_pose_issuer.py
```

## Development

### Building

```bash
colcon build --packages-select gui_pose_issuer
```

### Code Style

The code follows PEP 8 style guidelines and includes comprehensive docstrings.

## Integration

This package is designed to work with:
- `clock_pose_issuer`: Provides automatic clock-following behavior
- `motion_controller`: Receives pose commands and controls robot movement

The motion controller should prioritize user commands from this node over automatic clock commands, and return to clock following after 30 seconds of inactivity or when a cancel command is received.