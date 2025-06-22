cmake_minimum_required(VERSION 3.8)
project(clock_pose_issuer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Add library for ClockPoseIssuer class
add_library(clock_pose_issuer_lib src/clock_pose_issuer.cpp)
ament_target_dependencies(clock_pose_issuer_lib rclcpp geometry_msgs)
target_include_directories(clock_pose_issuer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Add executable
add_executable(clock_pose_issuer_node src/clock_pose_issuer_node.cpp)
target_link_libraries(clock_pose_issuer_node clock_pose_issuer_lib)
ament_target_dependencies(clock_pose_issuer_node rclcpp geometry_msgs)

# Install executable
install(TARGETS
  clock_pose_issuer_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Add unit tests
  ament_add_gtest(test_clock_pose_issuer test/test_clock_pose_issuer.cpp)
  target_link_libraries(test_clock_pose_issuer clock_pose_issuer_lib)
  ament_target_dependencies(test_clock_pose_issuer rclcpp geometry_msgs)
  
  # Add integration tests
  ament_add_gtest(test_clock_pose_integration test/test_clock_pose_integration.cpp)
  target_link_libraries(test_clock_pose_integration clock_pose_issuer_lib)
  ament_target_dependencies(test_clock_pose_integration rclcpp geometry_msgs)
  
  # Skip linting checks for this prototype
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
