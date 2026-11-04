cmake_minimum_required(VERSION 3.8)
project(simple_arm_gazebo)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(simple_arm_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tl_expected REQUIRED)

if(NOT TARGET tl::expected)
  add_library(tl::expected ALIAS tl_expected::tl_expected)
endif()

add_library(simple_arm_gripper_plugin SHARED src/simple_arm_gripper_plugin.cpp)
target_compile_features(simple_arm_gripper_plugin PUBLIC cxx_std_17)
target_include_directories(simple_arm_gripper_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(simple_arm_gripper_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(simple_arm_gripper_plugin gazebo_ros rclcpp std_msgs)

add_executable(pick_place_demo src/pick_place_demo.cpp)
target_compile_features(pick_place_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  pick_place_demo
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  rclcpp
  shape_msgs
  std_msgs
)

add_executable(arm_state_feedback src/arm_state_feedback.cpp)
target_compile_features(arm_state_feedback PUBLIC cxx_std_17)
ament_target_dependencies(
  arm_state_feedback
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

add_executable(object_recognition_server src/object_recognition_server.cpp)
target_compile_features(object_recognition_server PUBLIC cxx_std_17)
ament_target_dependencies(
  object_recognition_server
  rclcpp
  sensor_msgs
  simple_arm_interfaces
  tf2
  tf2_geometry_msgs
  tf2_ros
  vision_msgs
  visualization_msgs
)

add_executable(pick_place_server src/pick_place_server.cpp)
target_compile_features(pick_place_server PUBLIC cxx_std_17)
ament_target_dependencies(
  pick_place_server
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  rclcpp
  rclcpp_action
  shape_msgs
  simple_arm_interfaces
  std_msgs
)

install(TARGETS
  simple_arm_gripper_plugin
  pick_place_demo
  arm_state_feedback
  object_recognition_server
  pick_place_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
