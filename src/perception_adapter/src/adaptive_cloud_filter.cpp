#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"

namespace perception_adapter
{
namespace
{
struct VoxelKey
{
  int64_t x{};
  int64_t y{};
  int64_t z{};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto hx = std::hash<int64_t>{}(key.x);
    const auto hy = std::hash<int64_t>{}(key.y);
    const auto hz = std::hash<int64_t>{}(key.z);
    return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6) + (hx >> 2)) ^
           (hz + 0x9e3779b97f4a7c15ULL + (hy << 6) + (hy >> 2));
  }
};

struct VoxelAccumulator
{
  double x{};
  double y{};
  double z{};
  uint32_t count{};
};

struct FilteredClouds
{
  std::vector<std::array<float, 3>> navigation_points;
  std::vector<std::array<float, 3>> obstacle_points;
  std::vector<std::array<float, 3>> ground_points;
};

enum class PerceptionMode
{
  Detail,
  Normal,
  Fast,
  Safe
};

std::string toString(const PerceptionMode mode)
{
  switch (mode) {
    case PerceptionMode::Detail:
      return "DETAIL";
    case PerceptionMode::Fast:
      return "FAST";
    case PerceptionMode::Safe:
      return "SAFE";
    case PerceptionMode::Normal:
    default:
      return "NORMAL";
  }
}
}  // namespace

class AdaptiveCloudFilter : public rclcpp::Node
{
public:
  AdaptiveCloudFilter()
  : Node("adaptive_cloud_filter")
  {
    input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/cloud_registered");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    output_cloud_topic_ = declare_parameter<std::string>("output_cloud_topic", "/cloud_nav_filtered");
    obstacle_cloud_topic_ =
      declare_parameter<std::string>("obstacle_cloud_topic", "/nav_obstacle_cloud");
    ground_cloud_topic_ = declare_parameter<std::string>("ground_cloud_topic", "/nav_ground_cloud");
    mode_topic_ = declare_parameter<std::string>("mode_topic", "/perception_mode");

    low_speed_threshold_ = declare_parameter<double>("low_speed_threshold", 0.15);
    high_speed_threshold_ = declare_parameter<double>("high_speed_threshold", 0.55);
    detail_voxel_size_ = declare_parameter<double>("detail_voxel_size", 0.03);
    normal_voxel_size_ = declare_parameter<double>("normal_voxel_size", 0.08);
    fast_voxel_size_ = declare_parameter<double>("fast_voxel_size", 0.18);
    safe_voxel_size_ = declare_parameter<double>("safe_voxel_size", 0.05);

    min_height_ = declare_parameter<double>("min_height", -0.30);
    max_height_ = declare_parameter<double>("max_height", 0.60);
    min_range_ = declare_parameter<double>("min_range", 0.35);
    max_range_ = declare_parameter<double>("max_range", 15.0);
    roi_filter_enabled_ = declare_parameter<bool>("roi_filter_enabled", false);
    min_valid_points_ = declare_parameter<int>("min_valid_points", 30);

    publish_split_clouds_ = declare_parameter<bool>("publish_split_clouds", true);
    ground_min_height_ = declare_parameter<double>("ground_min_height", -0.15);
    ground_max_height_ = declare_parameter<double>("ground_max_height", 0.06);
    obstacle_min_height_ = declare_parameter<double>("obstacle_min_height", 0.08);
    obstacle_max_height_ = declare_parameter<double>("obstacle_max_height", 1.20);

    obstacle_check_enabled_ = declare_parameter<bool>("obstacle_check_enabled", false);
    front_x_min_ = declare_parameter<double>("front_x_min", 0.20);
    front_x_max_ = declare_parameter<double>("front_x_max", 1.20);
    front_y_abs_ = declare_parameter<double>("front_y_abs", 0.55);
    obstacle_dense_count_ = declare_parameter<int>("obstacle_dense_count", 80);

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud_topic_, rclcpp::SensorDataQoS());
    if (publish_split_clouds_) {
      obstacle_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        obstacle_cloud_topic_, rclcpp::SensorDataQoS());
      ground_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        ground_cloud_topic_, rclcpp::SensorDataQoS());
    }
    mode_pub_ = create_publisher<std_msgs::msg::String>(mode_topic_, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&AdaptiveCloudFilter::odomCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AdaptiveCloudFilter::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "adaptive_cloud_filter: %s -> %s, odom=%s",
      input_cloud_topic_.c_str(), output_cloud_topic_.c_str(), odom_topic_.c_str());
    if (publish_split_clouds_) {
      RCLCPP_INFO(
        get_logger(),
        "adaptive_cloud_filter split output: obstacle=%s ground=%s",
        obstacle_cloud_topic_.c_str(), ground_cloud_topic_.c_str());
    }
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & linear = msg->twist.twist.linear;
    speed_ = std::sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!hasRequiredFields(*msg)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "input cloud has no x/y/z fields, skip filtering");
      return;
    }

    const auto mode = chooseMode(*msg);
    const double voxel_size = voxelSizeForMode(mode);
    const auto filtered_clouds = filterCloud(*msg, voxel_size);

    sensor_msgs::msg::PointCloud2 output;
    fillCloudMessage(*msg, filtered_clouds.navigation_points, output);
    cloud_pub_->publish(output);

    if (publish_split_clouds_) {
      sensor_msgs::msg::PointCloud2 obstacle_output;
      fillCloudMessage(*msg, filtered_clouds.obstacle_points, obstacle_output);
      obstacle_cloud_pub_->publish(obstacle_output);

      sensor_msgs::msg::PointCloud2 ground_output;
      fillCloudMessage(*msg, filtered_clouds.ground_points, ground_output);
      ground_cloud_pub_->publish(ground_output);
    }

    std_msgs::msg::String mode_msg;
    mode_msg.data = toString(mode);
    mode_pub_->publish(mode_msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "mode=%s speed=%.3f m/s input=%u nav=%zu obstacle=%zu ground=%zu voxel=%.3f",
      mode_msg.data.c_str(), speed_, msg->width * msg->height,
      filtered_clouds.navigation_points.size(), filtered_clouds.obstacle_points.size(),
      filtered_clouds.ground_points.size(), voxel_size);
  }

  bool hasRequiredFields(const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    for (const auto & field : cloud.fields) {
      has_x = has_x || field.name == "x";
      has_y = has_y || field.name == "y";
      has_z = has_z || field.name == "z";
    }
    return has_x && has_y && has_z;
  }

  PerceptionMode chooseMode(const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    const auto [valid_count, front_count] = countValidAndFrontPoints(cloud);
    if (valid_count < static_cast<std::size_t>(std::max(0, min_valid_points_))) {
      return PerceptionMode::Safe;
    }
    if (obstacle_check_enabled_ &&
      front_count > static_cast<std::size_t>(std::max(0, obstacle_dense_count_)))
    {
      return PerceptionMode::Safe;
    }
    if (speed_ >= high_speed_threshold_) {
      return PerceptionMode::Fast;
    }
    if (speed_ <= low_speed_threshold_) {
      return PerceptionMode::Detail;
    }
    return PerceptionMode::Normal;
  }

  std::pair<std::size_t, std::size_t> countValidAndFrontPoints(
    const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    std::size_t valid_count = 0;
    std::size_t front_count = 0;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (!isPointInRoi(x, y, z)) {
        continue;
      }
      ++valid_count;
      // 默认采用 ROS 常见雷达坐标：x 向前、y 向左、z 向上。
      if (x >= front_x_min_ && x <= front_x_max_ && std::fabs(y) <= front_y_abs_) {
        ++front_count;
      }
    }
    return {valid_count, front_count};
  }

  FilteredClouds filterCloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const double voxel_size) const
  {
    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> navigation_voxels;
    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> obstacle_voxels;
    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> ground_voxels;
    navigation_voxels.reserve(std::max<std::size_t>(1, cloud.width * cloud.height / 3));
    if (publish_split_clouds_) {
      obstacle_voxels.reserve(std::max<std::size_t>(1, cloud.width * cloud.height / 6));
      ground_voxels.reserve(std::max<std::size_t>(1, cloud.width * cloud.height / 6));
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (!isPointInRoi(x, y, z)) {
        continue;
      }

      addPointToVoxelMap(navigation_voxels, x, y, z, voxel_size);
      if (publish_split_clouds_) {
        if (isObstaclePoint(z)) {
          addPointToVoxelMap(obstacle_voxels, x, y, z, voxel_size);
        } else if (isGroundPoint(z)) {
          addPointToVoxelMap(ground_voxels, x, y, z, voxel_size);
        }
      }
    }

    FilteredClouds result;
    result.navigation_points = materializeVoxelMap(navigation_voxels);
    if (publish_split_clouds_) {
      result.obstacle_points = materializeVoxelMap(obstacle_voxels);
      result.ground_points = materializeVoxelMap(ground_voxels);
    }
    return result;
  }

  void addPointToVoxelMap(
    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> & voxels,
    const float x,
    const float y,
    const float z,
    const double voxel_size) const
  {
    const VoxelKey key{
      static_cast<int64_t>(std::floor(x / voxel_size)),
      static_cast<int64_t>(std::floor(y / voxel_size)),
      static_cast<int64_t>(std::floor(z / voxel_size))};
    auto & acc = voxels[key];
    acc.x += x;
    acc.y += y;
    acc.z += z;
    ++acc.count;
  }

  std::vector<std::array<float, 3>> materializeVoxelMap(
    const std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> & voxels) const
  {
    std::vector<std::array<float, 3>> points;
    points.reserve(voxels.size());
    for (const auto & item : voxels) {
      const auto & acc = item.second;
      if (acc.count == 0) {
        continue;
      }
      points.push_back({
        static_cast<float>(acc.x / acc.count),
        static_cast<float>(acc.y / acc.count),
        static_cast<float>(acc.z / acc.count)});
    }
    return points;
  }

  bool isGroundPoint(const float z) const
  {
    // 当前先采用简单高度带分割，适合平地仿真和导航可视化验证；
    // 后续可替换为地面拟合、法向量或时空体素层中的 marking/clearing 分流。
    return z >= ground_min_height_ && z <= ground_max_height_;
  }

  bool isObstaclePoint(const float z) const
  {
    return z >= obstacle_min_height_ && z <= obstacle_max_height_;
  }

  bool isPointInRoi(const float x, const float y, const float z) const
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return false;
    }
    // /cloud_registered 通常在 FAST-LIO2 的全局/里程计坐标系下。
    // 默认不在这里做高度和距离裁剪，避免把远离地图原点但位于机器人附近的墙体误删。
    // 导航高度和距离 ROI 交给 pointcloud_to_laserscan 在 target_frame 下处理。
    if (!roi_filter_enabled_) {
      return true;
    }
    if (z < min_height_ || z > max_height_) {
      return false;
    }
    const double range = std::sqrt(static_cast<double>(x) * x + static_cast<double>(y) * y);
    return range >= min_range_ && range <= max_range_;
  }

  double voxelSizeForMode(const PerceptionMode mode) const
  {
    switch (mode) {
      case PerceptionMode::Detail:
        return sanitizeVoxel(detail_voxel_size_);
      case PerceptionMode::Fast:
        return sanitizeVoxel(fast_voxel_size_);
      case PerceptionMode::Safe:
        return sanitizeVoxel(safe_voxel_size_);
      case PerceptionMode::Normal:
      default:
        return sanitizeVoxel(normal_voxel_size_);
    }
  }

  double sanitizeVoxel(const double value) const
  {
    return std::max(0.01, value);
  }

  void fillCloudMessage(
    const sensor_msgs::msg::PointCloud2 & input,
    const std::vector<std::array<float, 3>> & points,
    sensor_msgs::msg::PointCloud2 & output) const
  {
    output.header = input.header;
    output.is_bigendian = false;
    output.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");

    for (const auto & point : points) {
      *out_x = point[0];
      *out_y = point[1];
      *out_z = point[2];
      ++out_x;
      ++out_y;
      ++out_z;
    }
  }

  std::string input_cloud_topic_;
  std::string odom_topic_;
  std::string output_cloud_topic_;
  std::string obstacle_cloud_topic_;
  std::string ground_cloud_topic_;
  std::string mode_topic_;

  double low_speed_threshold_{};
  double high_speed_threshold_{};
  double detail_voxel_size_{};
  double normal_voxel_size_{};
  double fast_voxel_size_{};
  double safe_voxel_size_{};
  double min_height_{};
  double max_height_{};
  double min_range_{};
  double max_range_{};
  bool roi_filter_enabled_{};
  int min_valid_points_{};

  bool publish_split_clouds_{};
  double ground_min_height_{};
  double ground_max_height_{};
  double obstacle_min_height_{};
  double obstacle_max_height_{};

  bool obstacle_check_enabled_{};
  double front_x_min_{};
  double front_x_max_{};
  double front_y_abs_{};
  int obstacle_dense_count_{};

  double speed_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
};
}  // namespace perception_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_adapter::AdaptiveCloudFilter>());
  rclcpp::shutdown();
  return 0;
}
