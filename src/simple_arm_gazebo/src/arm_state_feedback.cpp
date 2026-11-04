#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class ArmStateFeedback : public rclcpp::Node
{
public:
  ArmStateFeedback()
  : Node("arm_state_feedback"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "/simple_arm/feedback/joint_states", rclcpp::QoS(10));
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/simple_arm/feedback/tool_pose", rclcpp::QoS(10));
    joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        joint_publisher_->publish(*message);
        received_joint_state_ = true;
      });
    timer_ = create_wall_timer(100ms, std::bind(&ArmStateFeedback::PublishToolPose, this));
    RCLCPP_INFO(
      get_logger(),
      "Feedback ready: /simple_arm/feedback/joint_states and /simple_arm/feedback/tool_pose");
  }

private:
  void PublishToolPose()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform("world", "tool0", tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      pose_publisher_->publish(pose);
    } catch (const tf2::TransformException & exception) {
      if (received_joint_state_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Waiting for world -> tool0 transform: %s",
          exception.what());
      }
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool received_joint_state_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmStateFeedback>());
  rclcpp::shutdown();
  return 0;
}
