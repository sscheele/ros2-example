This code is implemented using ROS2. Do NOT manually create files that are typically created through ROS2 commands. For instance, if you want to make a new ROS package, do not create `src/my_pkg` and populate it manually with CMakeLists, package.xml, etc. Instead, run the terminal command `ros2 pkg create my_pkg`.

ROS2 C++ nodes should generally be implemented as follows:
- A .hpp header file in an `include` directory outlines the class extending `rclcpp::Node`
- A .cpp library file implements the class
- A second .cpp file defines the actual ROS node (with a `main` function). We'll use the convention that this file, which will usually consist primarily of the `main` function, should end with `_node.cpp`.
- Tests link against the same .cpp library as the main node