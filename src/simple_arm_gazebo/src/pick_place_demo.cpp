#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

namespace
{
moveit_msgs::msg::CollisionObject make_box(
  const std::string & id, const std::vector<double> & dimensions,
  double x, double y, double z)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.assign(dimensions.begin(), dimensions.end());
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  object.primitives.push_back(box);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

geometry_msgs::msg::Pose downward_pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  constexpr double half_sqrt_two = 0.7071067811865476;
  pose.orientation.y = half_sqrt_two;
  pose.orientation.w = half_sqrt_two;
  return pose;
}

bool plan_and_execute(MoveGroupInterface & move_group, const std::string & stage)
{
  move_group.setStartStateToCurrentState();
  MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(move_group.plan(plan))) {
    RCLCPP_ERROR(rclcpp::get_logger("pick_place_demo"), "Planning failed at stage: %s", stage.c_str());
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("pick_place_demo"), "Executing stage: %s", stage.c_str());
  if (!static_cast<bool>(move_group.execute(plan))) {
    RCLCPP_ERROR(rclcpp::get_logger("pick_place_demo"), "Execution failed at stage: %s", stage.c_str());
    return false;
  }
  return true;
}

bool move_to_pose(
  MoveGroupInterface & move_group, const geometry_msgs::msg::Pose & pose,
  const std::string & stage)
{
  move_group.clearPoseTargets();
  if (!move_group.setPoseTarget(pose, "tool0")) {
    RCLCPP_ERROR(rclcpp::get_logger("pick_place_demo"), "Invalid pose target at stage: %s", stage.c_str());
    return false;
  }
  return plan_and_execute(move_group, stage);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "pick_place_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const auto get_or_declare_double = [&node](const std::string & name, double default_value) {
      if (node->has_parameter(name)) {
        return node->get_parameter(name).as_double();
      }
      return node->declare_parameter<double>(name, default_value);
    };
  const double pick_x = get_or_declare_double("pick_x", 0.50);
  const double pick_y = get_or_declare_double("pick_y", 0.20);
  const double place_x = get_or_declare_double("place_x", 0.50);
  const double place_y = get_or_declare_double("place_y", -0.20);
  const double object_z = get_or_declare_double("object_z", 0.395);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  int result = 1;
  {
    MoveGroupInterface move_group(node, "arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene;
    auto gripper_publisher = node->create_publisher<std_msgs::msg::Bool>(
      "/simple_arm/gripper_command", rclcpp::QoS(10));

    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(8);
    move_group.setMaxVelocityScalingFactor(0.25);
    move_group.setMaxAccelerationScalingFactor(0.20);
    move_group.setGoalPositionTolerance(0.008);
    move_group.setGoalOrientationTolerance(0.04);

    planning_scene.removeCollisionObjects({"work_table", "pick_cube"});
    std::this_thread::sleep_for(300ms);
    const auto table = make_box("work_table", {0.58, 0.78, 0.36}, 0.62, 0.0, 0.18);
    const auto cube = make_box("pick_cube", {0.06, 0.06, 0.06}, pick_x, pick_y, object_z);
    if (!planning_scene.applyCollisionObjects({table, cube})) {
      RCLCPP_ERROR(node->get_logger(), "Failed to add table and cube to the MoveIt planning scene");
    } else {
      std_msgs::msg::Bool gripper_command;
      gripper_command.data = false;
      gripper_publisher->publish(gripper_command);

      RCLCPP_INFO(node->get_logger(), "Starting automatic pick-place sequence");
      move_group.setNamedTarget("ready");
      bool success = plan_and_execute(move_group, "ready pose");
      success = success && move_to_pose(
        move_group, downward_pose(pick_x, pick_y, 0.62), "pre-grasp");
      success = success && move_to_pose(
        move_group, downward_pose(pick_x, pick_y, 0.56), "lower to object");

      if (success) {
        gripper_command.data = true;
        gripper_publisher->publish(gripper_command);
        std::this_thread::sleep_for(500ms);
        success = move_group.attachObject("pick_cube", "tool0", {"tool0"});
        if (!success) {
          RCLCPP_ERROR(node->get_logger(), "Failed to attach cube in the MoveIt planning scene");
        }
      }

      success = success && move_to_pose(
        move_group, downward_pose(pick_x, pick_y, 0.65), "lift object");
      success = success && move_to_pose(
        move_group, downward_pose(place_x, place_y, 0.62), "move above place pose");
      success = success && move_to_pose(
        move_group, downward_pose(place_x, place_y, 0.56), "lower object");

      if (success) {
        move_group.detachObject("pick_cube");
        gripper_command.data = false;
        gripper_publisher->publish(gripper_command);
        std::this_thread::sleep_for(500ms);
        planning_scene.applyCollisionObject(
          make_box("pick_cube", {0.06, 0.06, 0.06}, place_x, place_y, object_z));
        success = move_to_pose(
          move_group, downward_pose(place_x, place_y, 0.65), "retreat after place");
      }

      if (success) {
        move_group.setNamedTarget("ready");
        success = plan_and_execute(move_group, "return to ready pose");
      }

      if (success) {
        RCLCPP_INFO(node->get_logger(), "Pick-place sequence completed successfully");
        result = 0;
      } else {
        gripper_command.data = false;
        gripper_publisher->publish(gripper_command);
        RCLCPP_ERROR(node->get_logger(), "Pick-place sequence stopped after a failed stage");
      }
    }
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return result;
}
