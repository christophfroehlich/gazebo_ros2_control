cmake_minimum_required(VERSION 3.5.0)
project(gazebo_ros2_control_demos)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(example_position examples/example_position.cpp)
ament_target_dependencies(example_position
  rclcpp
  rclcpp_action
  control_msgs
)
add_executable(example_position_with_velocity examples/example_position_with_velocity.cpp)
ament_target_dependencies(example_position_with_velocity
  rclcpp
  rclcpp_action
  control_msgs
)
add_executable(example_position_with_velocity_nonzero examples/example_position_with_velocity_nonzero.cpp)
ament_target_dependencies(example_position_with_velocity_nonzero
  rclcpp
  rclcpp_action
  control_msgs
)
add_executable(example_position_cancel examples/example_position_cancel.cpp)
ament_target_dependencies(example_position_cancel
  rclcpp
  rclcpp_action
  control_msgs
)

add_executable(example_velocity examples/example_velocity.cpp)
ament_target_dependencies(example_velocity
  rclcpp
  std_msgs
)
add_executable(example_position_topic examples/example_position_topic.cpp)
ament_target_dependencies(example_position_topic
  rclcpp
  trajectory_msgs
)
add_executable(example_position_topic_empty examples/example_position_topic_empty.cpp)
ament_target_dependencies(example_position_topic_empty
  rclcpp
  trajectory_msgs
)

add_executable(example_effort examples/example_effort.cpp)
ament_target_dependencies(example_effort
  rclcpp
  std_msgs
)

add_executable(example_gripper examples/example_gripper.cpp)
ament_target_dependencies(example_gripper
  rclcpp
  std_msgs
)

add_executable(example_diff_drive examples/example_diff_drive.cpp)
ament_target_dependencies(example_diff_drive
  rclcpp
  geometry_msgs
)

add_executable(example_tricycle_drive examples/example_tricycle_drive.cpp)
ament_target_dependencies(example_tricycle_drive
  rclcpp
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

## Install
install(
  TARGETS
    example_position
    example_position_cancel
    example_position_with_velocity
    example_position_with_velocity_nonzero
    example_position_topic
    example_position_topic_empty
    example_velocity
    example_effort
    example_diff_drive
    example_tricycle_drive
    example_gripper
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
