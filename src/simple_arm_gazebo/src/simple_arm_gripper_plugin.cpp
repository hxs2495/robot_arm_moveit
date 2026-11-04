#include <atomic>
#include <memory>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace simple_arm_gazebo
{
class SimpleArmGripperPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    world_ = model_->GetWorld();
    node_ = gazebo_ros::Node::Get(sdf);

    robot_link_name_ = sdf->Get<std::string>("robot_link", "link_6").first;
    object_model_name_ = sdf->Get<std::string>("object_model", "pick_cube").first;
    object_link_name_ = sdf->Get<std::string>("object_link", "link").first;
    const auto command_topic =
      sdf->Get<std::string>("command_topic", "/simple_arm/gripper_command").first;

    command_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
      command_topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        requested_attached_.store(message->data);
      });

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&SimpleArmGripperPlugin::OnUpdate, this));
    RCLCPP_INFO(
      node_->get_logger(), "Gazebo gripper ready: topic=%s, target=%s",
      command_topic.c_str(), object_model_name_.c_str());
  }

private:
  void OnUpdate()
  {
    const bool requested = requested_attached_.load();
    if (requested && !attached_) {
      Attach();
    } else if (!requested && attached_) {
      Detach();
    }
  }

  void Attach()
  {
    const auto robot_link = model_->GetLink(robot_link_name_);
    const auto object_model = world_->ModelByName(object_model_name_);
    const auto object_link = object_model ? object_model->GetLink(object_link_name_) : nullptr;
    if (!robot_link || !object_link) {
      if (++lookup_warning_counter_ % 1000 == 1) {
        RCLCPP_WARN(
          node_->get_logger(), "Cannot attach object: link %s or model %s::%s is unavailable",
          robot_link_name_.c_str(), object_model_name_.c_str(), object_link_name_.c_str());
      }
      return;
    }

    fixed_joint_ = world_->Physics()->CreateJoint("fixed", model_);
    fixed_joint_->Load(robot_link, object_link, ignition::math::Pose3d::Zero);
    fixed_joint_->Init();
    attached_ = true;
    RCLCPP_INFO(node_->get_logger(), "Gripper attached Gazebo object %s", object_model_name_.c_str());
  }

  void Detach()
  {
    if (fixed_joint_) {
      fixed_joint_->Detach();
      fixed_joint_.reset();
    }
    attached_ = false;
    RCLCPP_INFO(node_->get_logger(), "Gripper released Gazebo object %s", object_model_name_.c_str());
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::JointPtr fixed_joint_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr command_subscription_;
  std::atomic_bool requested_attached_{false};
  bool attached_{false};
  std::size_t lookup_warning_counter_{0};
  std::string robot_link_name_;
  std::string object_model_name_;
  std::string object_link_name_;
};
}  // namespace simple_arm_gazebo

GZ_REGISTER_MODEL_PLUGIN(simple_arm_gazebo::SimpleArmGripperPlugin)
