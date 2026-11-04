cmake_minimum_required(VERSION 3.8)
project(simple_arm_gui)

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(tl_expected REQUIRED)
if(NOT TARGET tl::expected)
  add_library(tl::expected ALIAS tl_expected::tl_expected)
endif()
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simple_arm_interfaces REQUIRED)

add_executable(simple_arm_gui_node src/main.cpp)
target_compile_features(simple_arm_gui_node PUBLIC cxx_std_17)
target_link_libraries(simple_arm_gui_node Qt5::Widgets)
ament_target_dependencies(simple_arm_gui_node
  moveit_ros_planning_interface
  moveit_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  simple_arm_interfaces
)

install(TARGETS simple_arm_gui_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
