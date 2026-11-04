#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "move_arm", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const auto target_name = node->has_parameter("target_name") ?
    node->get_parameter("target_name").as_string() :
    node->declare_parameter<std::string>("target_name", "ready");
  const auto joint_positions = node->has_parameter("joint_positions") ?
    node->get_parameter("joint_positions").as_double_array() :
    node->declare_parameter<std::vector<double>>("joint_positions", std::vector<double>{});
  const auto velocity_scaling = node->has_parameter("velocity_scaling") ?
    node->get_parameter("velocity_scaling").as_double() :
    node->declare_parameter<double>("velocity_scaling", 0.35);
  const auto acceleration_scaling = node->has_parameter("acceleration_scaling") ?
    node->get_parameter("acceleration_scaling").as_double() :
    node->declare_parameter<double>("acceleration_scaling", 0.35);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  int result = 1;
  {
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "arm");
    move_group.setPlanningTime(8.0);
    move_group.setMaxVelocityScalingFactor(velocity_scaling);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling);

    bool target_is_valid = false;
    if (!joint_positions.empty()) {
      if (joint_positions.size() != 6) {
        RCLCPP_ERROR(node->get_logger(),
          "Parameter joint_positions must contain exactly 6 values (received %zu)",
          joint_positions.size());
      } else {
        target_is_valid = move_group.setJointValueTarget(joint_positions);
        RCLCPP_INFO(node->get_logger(), "Using explicit six-joint target");
      }
    } else {
      target_is_valid = move_group.setNamedTarget(target_name);
      RCLCPP_INFO(node->get_logger(), "Using SRDF named target '%s'", target_name.c_str());
    }

    if (target_is_valid) {
      MoveGroupInterface::Plan plan;
      const bool planned = static_cast<bool>(move_group.plan(plan));
      if (!planned) {
        RCLCPP_ERROR(node->get_logger(), "Planning failed; the target may be in collision or unreachable");
      } else {
        RCLCPP_INFO(node->get_logger(), "Plan succeeded; executing trajectory");
        const bool executed = static_cast<bool>(move_group.execute(plan));
        if (executed) {
          RCLCPP_INFO(node->get_logger(), "Trajectory execution completed");
          result = 0;
        } else {
          RCLCPP_ERROR(node->get_logger(), "Trajectory execution failed");
        }
      }
    } else {
      RCLCPP_ERROR(node->get_logger(), "Unknown or invalid target");
    }
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return result;
}
