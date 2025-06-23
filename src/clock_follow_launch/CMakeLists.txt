cmake_minimum_required(VERSION 3.8)
project(clock_follow_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(vel_relay_lib src/vel_relay.cpp)
ament_target_dependencies(vel_relay_lib rclcpp geometry_msgs turtlesim)

# Create executable
add_executable(vel_relay_node src/vel_relay_node.cpp)
target_link_libraries(vel_relay_node vel_relay_lib)
ament_target_dependencies(vel_relay_node rclcpp geometry_msgs turtlesim)

# Install targets
install(TARGETS
  vel_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
