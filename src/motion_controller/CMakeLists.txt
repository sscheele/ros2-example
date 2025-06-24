cmake_minimum_required(VERSION 3.8)
project(motion_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# Create include directory
include_directories(include)

# Create library
add_library(motion_controller_lib
  src/motion_controller.cpp
)

ament_target_dependencies(motion_controller_lib
  rclcpp
  geometry_msgs
  std_msgs
  turtlesim
)

# Create executable
add_executable(motion_controller
  src/motion_controller_node.cpp
)

target_link_libraries(motion_controller motion_controller_lib)

ament_target_dependencies(motion_controller
  rclcpp
  geometry_msgs
  std_msgs
  turtlesim
)

# Install targets
install(TARGETS
  motion_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  motion_controller_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Add unit tests
  ament_add_gtest(test_motion_controller
    test/test_motion_controller.cpp
  )
  
  target_link_libraries(test_motion_controller motion_controller_lib)
  
  ament_target_dependencies(test_motion_controller
    rclcpp
    geometry_msgs
    std_msgs
    turtlesim
  )

  # Add integration tests
  ament_add_gtest(test_motion_controller_integration
    test/test_motion_controller_integration.cpp
  )
  
  target_link_libraries(test_motion_controller_integration motion_controller_lib)
  
  ament_target_dependencies(test_motion_controller_integration
    rclcpp
    geometry_msgs
    std_msgs
    turtlesim
  )
endif()

ament_package()
