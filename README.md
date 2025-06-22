# ros2-example

A prototype ROS2 stack for a robot that follows pose inputs from two different sources. The robot should take a desired pose and then use a motion controller to get to the desired pose.

Imagine you're building infrastructure for someone else to plug into later -- it should be simple, easy to interpret and build on top of, and have things like a readme, launch system and test scaffolding.

# Development
This repo exposes a vscode dev container. If you want to be able to run graphical programs by sharing your host's X display with the container, the simplest way is to run the following on the host:

```
xhost +local:docker
```

Note that this will allow any user on your local workstation to interact with your X server; it is not secure for untrusted shared workstations. There are more involved options for authenticating to an X server in this case.

## VS Code
To get the VS Code C++ extension to properly detect your C++ libraries, generate a unified compile_commands.json or use the one in the root of this workspace. To generate, you first have to remove the `install`, `build`, and `log` directories, and then run:

```
colcon build --merge-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Then the `compile_commands.json` file will be in your `build` directory. It looks like as long as this JSON is generated, VS Code's C++ extension will automatically detect and add it. I like to copy it into my workspace root so that I can run isolated compilation during development. I am `gitignoring` compile_commands.json to force you to do this manually, as this file may change during your own development.

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