cmake_minimum_required(VERSION 3.8)
project(simple_arm_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(tl_expected REQUIRED)

# Compatibility for Humble MoveIt binaries that still export tl::expected.
if(NOT TARGET tl::expected)
  add_library(tl::expected ALIAS tl_expected::tl_expected)
endif()
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(move_arm src/move_arm.cpp)
target_compile_features(move_arm PUBLIC cxx_std_17)
ament_target_dependencies(move_arm moveit_ros_planning_interface rclcpp)

install(TARGETS move_arm DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
