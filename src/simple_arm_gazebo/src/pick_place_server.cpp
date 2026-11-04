#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <simple_arm_interfaces/action/pick_place.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using PickPlace = simple_arm_interfaces::action::PickPlace;
using GoalHandlePickPlace = rclcpp_action::ServerGoalHandle<PickPlace>;

namespace
{
moveit_msgs::msg::CollisionObject make_box(
  const std::string & id, const std::array<double, 3> & dimensions,
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
}  // namespace

class PickPlaceServer : public rclcpp::Node
{
public:
  PickPlaceServer()
  : Node("pick_place_server")
  {
    gripper_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/simple_arm/gripper_command", 10);
  }

  void initialize()
  {
    move_group_ = std::make_unique<MoveGroupInterface>(shared_from_this(), "arm");
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(8);
    move_group_->setMaxVelocityScalingFactor(0.25);
    move_group_->setMaxAccelerationScalingFactor(0.20);
    move_group_->setGoalPositionTolerance(0.008);
    move_group_->setGoalOrientationTolerance(0.04);

    action_server_ = rclcpp_action::create_server<PickPlace>(
      shared_from_this(), "/simple_arm/tasks/pick_place",
      std::bind(&PickPlaceServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PickPlaceServer::handle_accepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Pick-place action ready: /simple_arm/tasks/pick_place");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const PickPlace::Goal> goal)
  {
    if (busy_.load()) {
      RCLCPP_WARN(get_logger(), "Rejecting pick-place goal because another task is running");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->target.id.empty() || goal->target.header.frame_id != "world" ||
      goal->place_pose.header.frame_id != "world")
    {
      RCLCPP_WARN(get_logger(), "Rejecting invalid target or non-world pose");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePickPlace>)
  {
    if (move_group_) {
      move_group_->stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
  {
    busy_.store(true);
    std::thread([this, goal_handle]() {execute(goal_handle);}).detach();
  }

  void feedback(
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle,
    const std::string & stage, float progress)
  {
    auto message = std::make_shared<PickPlace::Feedback>();
    message->stage = stage;
    message->progress = progress;
    goal_handle->publish_feedback(message);
    RCLCPP_INFO(get_logger(), "Task stage: %s", stage.c_str());
  }

  bool move_to_pose(
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle,
    const geometry_msgs::msg::Pose & pose, const std::string & stage, float progress)
  {
    if (goal_handle->is_canceling()) {
      return false;
    }
    feedback(goal_handle, stage + "：规划中", progress);
    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();
    if (!move_group_->setPoseTarget(pose, "tool0")) {
      return false;
    }
    MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(move_group_->plan(plan))) {
      return false;
    }
    if (goal_handle->is_canceling()) {
      return false;
    }
    feedback(goal_handle, stage + "：执行中", progress + 0.03F);
    return static_cast<bool>(move_group_->execute(plan));
  }

  bool move_to_ready(
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle, float progress)
  {
    feedback(goal_handle, "移动到准备位", progress);
    move_group_->setStartStateToCurrentState();
    move_group_->setNamedTarget("ready");
    MoveGroupInterface::Plan plan;
    return static_cast<bool>(move_group_->plan(plan)) &&
           !goal_handle->is_canceling() &&
           static_cast<bool>(move_group_->execute(plan));
  }

  void execute(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const double pick_x = goal->target.pose.position.x;
    const double pick_y = goal->target.pose.position.y;
    const double object_z = goal->target.pose.position.z;
    const double place_x = goal->place_pose.pose.position.x;
    const double place_y = goal->place_pose.pose.position.y;
    const double place_z = goal->place_pose.pose.position.z;
    const double approach = goal->approach_height > 0.02 ? goal->approach_height : 0.06;
    const double grasp_tool_z = object_z + 0.165;
    const double approach_tool_z = grasp_tool_z + approach;
    const double lift_tool_z = grasp_tool_z + std::max(approach + 0.03, 0.09);

    auto result = std::make_shared<PickPlace::Result>();
    bool success = false;
    bool attached = false;
    try {
      planning_scene_.removeCollisionObjects({"work_table", goal->target.id});
      std::this_thread::sleep_for(250ms);
      const auto table = make_box("work_table", {0.58, 0.78, 0.36}, 0.62, 0.0, 0.18);
      const auto object = make_box(
        goal->target.id,
        {goal->target.size.x, goal->target.size.y, goal->target.size.z},
        pick_x, pick_y, object_z);
      if (!planning_scene_.applyCollisionObjects({table, object})) {
        throw std::runtime_error("无法更新 MoveIt 规划场景");
      }

      publish_gripper(false);
      success = move_to_ready(goal_handle, 0.05F);
      success = success && move_to_pose(
        goal_handle, downward_pose(pick_x, pick_y, approach_tool_z), "接近抓取目标", 0.16F);
      success = success && move_to_pose(
        goal_handle, downward_pose(pick_x, pick_y, grasp_tool_z), "下降到抓取位", 0.30F);

      if (success && !goal_handle->is_canceling()) {
        feedback(goal_handle, "闭合夹具并附着目标", 0.43F);
        publish_gripper(true);
        std::this_thread::sleep_for(500ms);
        attached = move_group_->attachObject(goal->target.id, "tool0", {"tool0"});
        success = attached;
      }
      success = success && move_to_pose(
        goal_handle, downward_pose(pick_x, pick_y, lift_tool_z), "抬升目标", 0.52F);
      success = success && move_to_pose(
        goal_handle, downward_pose(place_x, place_y, place_z + 0.225), "搬运到放置区", 0.66F);
      success = success && move_to_pose(
        goal_handle, downward_pose(place_x, place_y, place_z + 0.165), "下降到放置位", 0.78F);

      if (success && !goal_handle->is_canceling()) {
        feedback(goal_handle, "释放目标", 0.86F);
        move_group_->detachObject(goal->target.id);
        attached = false;
        publish_gripper(false);
        std::this_thread::sleep_for(500ms);
        planning_scene_.applyCollisionObject(make_box(
          goal->target.id,
          {goal->target.size.x, goal->target.size.y, goal->target.size.z},
          place_x, place_y, place_z));
      }
      success = success && move_to_pose(
        goal_handle, downward_pose(place_x, place_y, place_z + 0.255), "撤离放置区", 0.90F);
      success = success && move_to_ready(goal_handle, 0.96F);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Pick-place task exception: %s", error.what());
      result->message = error.what();
      success = false;
    }

    if (attached) {
      move_group_->detachObject(goal->target.id);
    }
    if (!success) {
      publish_gripper(false);
    }

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "抓取任务已取消";
      goal_handle->canceled(result);
    } else if (success) {
      feedback(goal_handle, "抓取放置任务完成", 1.0F);
      result->success = true;
      result->message = "目标已成功抓取并放置";
      goal_handle->succeed(result);
    } else {
      result->success = false;
      if (result->message.empty()) {
        result->message = "规划或执行失败，请检查目标位置与 OctoMap 障碍物";
      }
      goal_handle->abort(result);
    }
    busy_.store(false);
  }

  void publish_gripper(bool close)
  {
    std_msgs::msg::Bool command;
    command.data = close;
    gripper_publisher_->publish(command);
  }

  std::unique_ptr<MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  rclcpp_action::Server<PickPlace>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_publisher_;
  std::atomic_bool busy_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceServer>();
  node->initialize();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
