#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <simple_arm_interfaces/srv/detect_objects.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ObjectRecognitionServer : public rclcpp::Node
{
public:
  ObjectRecognitionServer()
  : Node("object_recognition_server"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    cloud_topic_ = declare_parameter<std::string>(
      "cloud_topic", "/simple_arm/sensors/depth_camera/points");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    object_z_ = declare_parameter<double>("object_z", 0.395);
    min_points_ = declare_parameter<int>("min_points", 20);

    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_cloud_ = std::move(cloud);
      });
    detection_publisher_ = create_publisher<vision_msgs::msg::Detection3DArray>(
      "/simple_arm/perception/detections", 10);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/simple_arm/perception/markers", 10);
    service_ = create_service<simple_arm_interfaces::srv::DetectObjects>(
      "/simple_arm/perception/detect_objects",
      std::bind(
        &ObjectRecognitionServer::detect, this, std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(), "Object recognition ready: %s -> /simple_arm/perception/detect_objects",
      cloud_topic_.c_str());
  }

private:
  static int field_offset(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
  {
    for (const auto & field : cloud.fields) {
      if (field.name == name) {
        return static_cast<int>(field.offset);
      }
    }
    return -1;
  }

  static float read_float(const std::uint8_t * point, int offset)
  {
    float value = 0.0F;
    std::memcpy(&value, point + offset, sizeof(float));
    return value;
  }

  void detect(
    const std::shared_ptr<simple_arm_interfaces::srv::DetectObjects::Request>,
    std::shared_ptr<simple_arm_interfaces::srv::DetectObjects::Response> response)
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud = latest_cloud_;
    }
    if (!cloud) {
      response->success = false;
      response->message = "尚未收到深度相机点云";
      return;
    }

    geometry_msgs::msg::TransformStamped transform_message;
    try {
      transform_message = tf_buffer_.lookupTransform(
        world_frame_, cloud->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      response->success = false;
      response->message = std::string("点云 TF 不可用：") + error.what();
      return;
    }

    const int x_offset = field_offset(*cloud, "x");
    const int y_offset = field_offset(*cloud, "y");
    const int z_offset = field_offset(*cloud, "z");
    int rgb_offset = field_offset(*cloud, "rgb");
    if (rgb_offset < 0) {
      rgb_offset = field_offset(*cloud, "rgba");
    }
    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
      response->success = false;
      response->message = "点云缺少 x/y/z 字段";
      return;
    }

    tf2::Transform world_from_cloud;
    tf2::fromMsg(transform_message.transform, world_from_cloud);
    tf2::Vector3 tool_position;
    bool have_tool_position = false;
    try {
      const auto tool_transform = tf_buffer_.lookupTransform(
        world_frame_, "tool0", tf2::TimePointZero);
      tool_position = tf2::Vector3(
        tool_transform.transform.translation.x, tool_transform.transform.translation.y,
        tool_transform.transform.translation.z);
      have_tool_position = true;
    } catch (const tf2::TransformException &) {
    }

    std::vector<tf2::Vector3> orange_points;
    std::vector<tf2::Vector3> geometry_points;
    const std::size_t point_count =
      static_cast<std::size_t>(cloud->width) * static_cast<std::size_t>(cloud->height);
    orange_points.reserve(point_count / 20);
    geometry_points.reserve(point_count / 20);

    for (std::size_t index = 0; index < point_count; ++index) {
      const std::uint8_t * point = cloud->data.data() + index * cloud->point_step;
      const float x = read_float(point, x_offset);
      const float y = read_float(point, y_offset);
      const float z = read_float(point, z_offset);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const tf2::Vector3 world_point = world_from_cloud * tf2::Vector3(x, y, z);
      if (world_point.x() < 0.32 || world_point.x() > 0.90 ||
        world_point.y() < -0.38 || world_point.y() > 0.38 ||
        world_point.z() < 0.375 || world_point.z() > 0.46)
      {
        continue;
      }
      if (have_tool_position && (world_point - tool_position).length2() < 0.0225) {
        continue;
      }
      geometry_points.push_back(world_point);

      if (rgb_offset >= 0) {
        const auto * color = point + rgb_offset;
        const std::uint8_t red = cloud->is_bigendian ? color[1] : color[2];
        const std::uint8_t green = color[1];
        const std::uint8_t blue = cloud->is_bigendian ? color[2] : color[0];
        if (red > 150 && green > 20 && green < 150 && blue < 100 && red > green * 3 / 2) {
          orange_points.push_back(world_point);
        }
      }
    }

    const auto & candidates =
      orange_points.size() >= static_cast<std::size_t>(min_points_) ?
      orange_points : geometry_points;
    if (candidates.size() < static_cast<std::size_t>(min_points_)) {
      response->success = false;
      response->message = "工作台区域内未识别到可抓取方块";
      publish_detections({});
      return;
    }

    std::vector<bool> assigned(candidates.size(), false);
    std::vector<std::size_t> best_cluster;
    double best_score = std::numeric_limits<double>::infinity();
    constexpr double neighbor_distance_squared = 0.000625;
    for (std::size_t seed = 0; seed < candidates.size(); ++seed) {
      if (assigned[seed]) {
        continue;
      }
      std::vector<std::size_t> cluster{seed};
      assigned[seed] = true;
      for (std::size_t cursor = 0; cursor < cluster.size(); ++cursor) {
        const auto & source = candidates[cluster[cursor]];
        for (std::size_t neighbor = 0; neighbor < candidates.size(); ++neighbor) {
          if (assigned[neighbor]) {
            continue;
          }
          const auto & target = candidates[neighbor];
          const double dx = source.x() - target.x();
          const double dy = source.y() - target.y();
          const double dz = source.z() - target.z();
          if (dx * dx + dy * dy < neighbor_distance_squared && std::abs(dz) < 0.035) {
            assigned[neighbor] = true;
            cluster.push_back(neighbor);
          }
        }
      }
      if (cluster.size() < static_cast<std::size_t>(min_points_)) {
        continue;
      }

      double min_x = candidates[cluster.front()].x();
      double max_x = min_x;
      double min_y = candidates[cluster.front()].y();
      double max_y = min_y;
      double center_x = 0.0;
      double center_y = 0.0;
      for (const auto index : cluster) {
        const auto & point = candidates[index];
        min_x = std::min(min_x, point.x());
        max_x = std::max(max_x, point.x());
        min_y = std::min(min_y, point.y());
        max_y = std::max(max_y, point.y());
        center_x += point.x();
        center_y += point.y();
      }
      center_x /= static_cast<double>(cluster.size());
      center_y /= static_cast<double>(cluster.size());
      const double span_x = max_x - min_x;
      const double span_y = max_y - min_y;
      if (span_x > 0.13 || span_y > 0.13) {
        continue;
      }

      double score = std::abs(span_x - 0.06) + std::abs(span_y - 0.06);
      if (have_tool_position) {
        const double tool_dx = center_x - tool_position.x();
        const double tool_dy = center_y - tool_position.y();
        if (tool_dx * tool_dx + tool_dy * tool_dy < 0.04) {
          score += 0.25;
        }
      }
      if (score < best_score) {
        best_score = score;
        best_cluster = std::move(cluster);
      }
    }

    std::vector<double> xs;
    std::vector<double> ys;
    if (best_cluster.empty()) {
      xs.reserve(candidates.size());
      ys.reserve(candidates.size());
      for (const auto & point : candidates) {
        xs.push_back(point.x());
        ys.push_back(point.y());
      }
    } else {
      xs.reserve(best_cluster.size());
      ys.reserve(best_cluster.size());
      for (const auto index : best_cluster) {
        xs.push_back(candidates[index].x());
        ys.push_back(candidates[index].y());
      }
    }
    const auto middle_x = xs.begin() + static_cast<std::ptrdiff_t>(xs.size() / 2);
    const auto middle_y = ys.begin() + static_cast<std::ptrdiff_t>(ys.size() / 2);
    std::nth_element(xs.begin(), middle_x, xs.end());
    std::nth_element(ys.begin(), middle_y, ys.end());

    simple_arm_interfaces::msg::DetectedObject object;
    object.header.stamp = now();
    object.header.frame_id = world_frame_;
    object.id = "pick_cube";
    object.class_name = "橙色方块";
    object.confidence = std::min(
      0.99F, 0.70F + static_cast<float>(xs.size()) / 3000.0F);
    object.pose.position.x = *middle_x;
    object.pose.position.y = *middle_y;
    object.pose.position.z = object_z_;
    object.pose.orientation.w = 1.0;
    object.size.x = 0.06;
    object.size.y = 0.06;
    object.size.z = 0.06;

    response->objects.push_back(object);
    response->success = true;
    response->message = "识别到 1 个可抓取目标";
    publish_detections({object});
    publish_marker(object);
  }

  void publish_detections(
    const std::vector<simple_arm_interfaces::msg::DetectedObject> & objects)
  {
    vision_msgs::msg::Detection3DArray array;
    array.header.stamp = now();
    array.header.frame_id = world_frame_;
    for (const auto & object : objects) {
      vision_msgs::msg::Detection3D detection;
      detection.header = object.header;
      detection.id = object.id;
      detection.bbox.center = object.pose;
      detection.bbox.size = object.size;
      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id = object.class_name;
      hypothesis.hypothesis.score = object.confidence;
      hypothesis.pose.pose = object.pose;
      detection.results.push_back(hypothesis);
      array.detections.push_back(detection);
    }
    detection_publisher_->publish(array);
  }

  void publish_marker(const simple_arm_interfaces::msg::DetectedObject & object)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.header = object.header;
    marker.ns = "recognized_objects";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = object.pose;
    marker.scale = object.size;
    marker.color.r = 1.0F;
    marker.color.g = 0.32F;
    marker.color.b = 0.04F;
    marker.color.a = 0.85F;
    marker.lifetime = rclcpp::Duration::from_seconds(5.0);
    markers.markers.push_back(marker);
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.pose.position.z += 0.09;
    marker.scale.z = 0.035;
    marker.color.r = marker.color.g = marker.color.b = 1.0F;
    marker.text = object.class_name + " " + std::to_string(
      static_cast<int>(object.confidence * 100.0F)) + "%";
    markers.markers.push_back(marker);
    marker_publisher_->publish(markers);
  }

  std::string cloud_topic_;
  std::string world_frame_;
  double object_z_{0.395};
  int min_points_{20};
  std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Service<simple_arm_interfaces::srv::DetectObjects>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectRecognitionServer>());
  rclcpp::shutdown();
  return 0;
}
