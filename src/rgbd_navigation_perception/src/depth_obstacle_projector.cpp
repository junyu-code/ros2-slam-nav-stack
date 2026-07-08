#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rgbd_navigation_perception
{
namespace
{
struct Intrinsics
{
  double fx{};
  double fy{};
  double cx{};
  double cy{};
  bool valid{false};
};

struct PointXYZ
{
  float x{};
  float y{};
  float z{};
};

bool isDepthEncodingSupported(const std::string & encoding)
{
  return encoding == "16UC1" || encoding == "mono16" || encoding == "32FC1";
}

uint16_t readUint16(const uint8_t * data, const bool big_endian)
{
  if (big_endian) {
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
  }
  return static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
}

float readFloat32(const uint8_t * data, const bool big_endian)
{
  const uint32_t bits = big_endian ?
    ((static_cast<uint32_t>(data[0]) << 24) |
    (static_cast<uint32_t>(data[1]) << 16) |
    (static_cast<uint32_t>(data[2]) << 8) |
    static_cast<uint32_t>(data[3])) :
    (static_cast<uint32_t>(data[0]) |
    (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[3]) << 24));

  float value = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
}  // namespace

class DepthObstacleProjector : public rclcpp::Node
{
public:
  DepthObstacleProjector()
  : Node("depth_obstacle_projector")
  {
    enabled_ = declare_parameter<bool>("enabled", false);
    depth_image_topic_ = declare_parameter<std::string>(
      "depth_image_topic", "/nav_camera/d435i/depth/image_rect_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/nav_camera/d435i/depth/camera_info");
    output_cloud_topic_ = declare_parameter<std::string>("output_cloud_topic", "/visual_obstacles");
    frame_id_override_ = declare_parameter<std::string>("frame_id_override", "");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    set_enabled_service_ = declare_parameter<std::string>(
      "set_enabled_service", "/rgbd_nav/set_enabled");

    pixel_step_ = declare_parameter<int>("pixel_step", 4);
    min_depth_m_ = declare_parameter<double>("min_depth_m", 0.25);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 5.0);
    obstacle_min_height_m_ = declare_parameter<double>("obstacle_min_height_m", 0.08);
    obstacle_max_height_m_ = declare_parameter<double>("obstacle_max_height_m", 1.20);
    min_forward_m_ = declare_parameter<double>("min_forward_m", 0.05);
    max_forward_m_ = declare_parameter<double>("max_forward_m", 4.0);
    max_lateral_m_ = declare_parameter<double>("max_lateral_m", 2.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud_topic_, rclcpp::SensorDataQoS());

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DepthObstacleProjector::cameraInfoCallback, this, std::placeholders::_1));

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DepthObstacleProjector::depthCallback, this, std::placeholders::_1));

    enable_service_ = create_service<std_srvs::srv::SetBool>(
      set_enabled_service_,
      std::bind(
        &DepthObstacleProjector::setEnabledCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "depth_obstacle_projector ready: enabled=%s depth=%s info=%s output=%s target=%s",
      enabled_ ? "true" : "false",
      depth_image_topic_.c_str(),
      camera_info_topic_.c_str(),
      output_cloud_topic_.c_str(),
      target_frame_.c_str());
  }

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    const double fx = msg->k[0];
    const double fy = msg->k[4];
    const double cx = msg->k[2];
    const double cy = msg->k[5];
    if (fx <= 0.0 || fy <= 0.0 || !std::isfinite(fx) || !std::isfinite(fy)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "camera_info 内参无效，等待有效的 fx/fy");
      intrinsics_.valid = false;
      return;
    }

    intrinsics_.fx = fx;
    intrinsics_.fy = fy;
    intrinsics_.cx = cx;
    intrinsics_.cy = cy;
    intrinsics_.valid = true;
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!enabled_) {
      return;
    }

    if (!intrinsics_.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "还没有收到有效 CameraInfo，暂不投影深度图");
      return;
    }

    if (!isDepthEncodingSupported(msg->encoding)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "不支持的深度图编码: %s，仅支持 16UC1/mono16/32FC1",
        msg->encoding.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped sensor_to_target;
    if (!lookupSensorToTarget(*msg, sensor_to_target)) {
      return;
    }

    const auto points = projectDepthImage(*msg, sensor_to_target);
    sensor_msgs::msg::PointCloud2 cloud;
    fillPointCloud(*msg, points, cloud);
    cloud_pub_->publish(cloud);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "RGB-D 导航感知发布点云: input=%ux%u output=%zu enabled=%s",
      msg->width, msg->height, points.size(), enabled_ ? "true" : "false");
  }

  bool lookupSensorToTarget(
    const sensor_msgs::msg::Image & msg,
    geometry_msgs::msg::TransformStamped & sensor_to_target)
  {
    const std::string sensor_frame = !frame_id_override_.empty() ?
      frame_id_override_ : msg.header.frame_id;
    if (sensor_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "深度图没有 frame_id，无法投影到机器人坐标系");
      return false;
    }

    try {
      sensor_to_target = tf_buffer_->lookupTransform(
        target_frame_, sensor_frame, msg.header.stamp, rclcpp::Duration::from_seconds(0.05));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "等待 RGB-D TF %s -> %s: %s",
        sensor_frame.c_str(), target_frame_.c_str(), ex.what());
      return false;
    }
  }

  std::vector<PointXYZ> projectDepthImage(
    const sensor_msgs::msg::Image & msg,
    const geometry_msgs::msg::TransformStamped & sensor_to_target)
  {
    std::vector<PointXYZ> points;
    const int step = std::max(pixel_step_, 1);
    const std::size_t bytes_per_pixel = msg.encoding == "32FC1" ? 4U : 2U;

    if (msg.step < msg.width * bytes_per_pixel) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "深度图 step 与图像宽度不匹配，跳过本帧");
      return points;
    }
    if (msg.data.size() < static_cast<std::size_t>(msg.step) * msg.height) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "深度图数据长度不足，跳过本帧");
      return points;
    }

    const std::size_t reserve_count =
      (static_cast<std::size_t>(msg.width) / static_cast<std::size_t>(step) + 1U) *
      (static_cast<std::size_t>(msg.height) / static_cast<std::size_t>(step) + 1U);
    points.reserve(reserve_count);

    for (uint32_t v = 0; v < msg.height; v += static_cast<uint32_t>(step)) {
      const auto * row = msg.data.data() + static_cast<std::size_t>(v) * msg.step;
      for (uint32_t u = 0; u < msg.width; u += static_cast<uint32_t>(step)) {
        const auto * pixel = row + static_cast<std::size_t>(u) * bytes_per_pixel;
        const double depth_m = depthAtPixel(pixel, msg.encoding, msg.is_bigendian != 0);
        if (!isDepthUsable(depth_m)) {
          continue;
        }

        // 深度图投影采用 ROS 光学坐标系：x 向右，y 向下，z 向前。
        geometry_msgs::msg::PointStamped optical_point;
        optical_point.header = msg.header;
        optical_point.point.x =
          (static_cast<double>(u) - intrinsics_.cx) * depth_m / intrinsics_.fx;
        optical_point.point.y =
          (static_cast<double>(v) - intrinsics_.cy) * depth_m / intrinsics_.fy;
        optical_point.point.z = depth_m;

        geometry_msgs::msg::PointStamped target_point;
        tf2::doTransform(optical_point, target_point, sensor_to_target);
        if (!isObstaclePointUsable(target_point)) {
          continue;
        }

        points.push_back(PointXYZ{
          static_cast<float>(target_point.point.x),
          static_cast<float>(target_point.point.y),
          static_cast<float>(target_point.point.z)});
      }
    }

    return points;
  }

  double depthAtPixel(
    const uint8_t * pixel,
    const std::string & encoding,
    const bool big_endian) const
  {
    if (encoding == "32FC1") {
      return static_cast<double>(readFloat32(pixel, big_endian));
    }

    const uint16_t raw_mm = readUint16(pixel, big_endian);
    if (raw_mm == 0U) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return static_cast<double>(raw_mm) * 0.001;
  }

  bool isDepthUsable(const double depth_m) const
  {
    return std::isfinite(depth_m) && depth_m >= min_depth_m_ && depth_m <= max_depth_m_;
  }

  bool isObstaclePointUsable(const geometry_msgs::msg::PointStamped & point) const
  {
    const auto & p = point.point;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      return false;
    }
    if (p.x < min_forward_m_ || p.x > max_forward_m_) {
      return false;
    }
    if (std::abs(p.y) > max_lateral_m_) {
      return false;
    }
    return p.z >= obstacle_min_height_m_ && p.z <= obstacle_max_height_m_;
  }

  void fillPointCloud(
    const sensor_msgs::msg::Image & depth_msg,
    const std::vector<PointXYZ> & points,
    sensor_msgs::msg::PointCloud2 & cloud) const
  {
    cloud.header = depth_msg.header;
    cloud.header.frame_id = target_frame_;
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (const auto & point : points) {
      *iter_x = point.x;
      *iter_y = point.y;
      *iter_z = point.z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  }

  void setEnabledCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    enabled_ = request->data;
    response->success = true;
    response->message = enabled_ ?
      "RGB-D navigation perception enabled" :
      "RGB-D navigation perception disabled";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  bool enabled_{false};
  std::string depth_image_topic_;
  std::string camera_info_topic_;
  std::string output_cloud_topic_;
  std::string frame_id_override_;
  std::string target_frame_;
  std::string set_enabled_service_;
  int pixel_step_{4};
  double min_depth_m_{0.25};
  double max_depth_m_{5.0};
  double obstacle_min_height_m_{0.08};
  double obstacle_max_height_m_{1.20};
  double min_forward_m_{0.05};
  double max_forward_m_{4.0};
  double max_lateral_m_{2.0};
  Intrinsics intrinsics_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
}  // namespace rgbd_navigation_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rgbd_navigation_perception::DepthObstacleProjector>());
  rclcpp::shutdown();
  return 0;
}
