#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace cloud_relocalization
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

struct GridPoint
{
  int x{};
  int y{};
};

struct SearchNode
{
  int x{};
  int y{};
  int level{};
  int angle_index{};
  float upper_bound{};
};

struct SearchNodeLess
{
  bool operator()(const SearchNode & lhs, const SearchNode & rhs) const
  {
    return lhs.upper_bound < rhs.upper_bound;
  }
};

struct Candidate
{
  int x{};
  int y{};
  int angle_index{};
  float score{-std::numeric_limits<float>::infinity()};
};

struct DistanceCell
{
  int index{};
  float distance{};
};

struct DistanceCellGreater
{
  bool operator()(const DistanceCell & lhs, const DistanceCell & rhs) const
  {
    return lhs.distance > rhs.distance;
  }
};
}  // namespace

class BnbLocalizationNode : public rclcpp::Node
{
public:
  BnbLocalizationNode()
  : Node("bnb_localization_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    pose_topic_ = declare_parameter<std::string>(
      "pose_topic", "/relocalization/coarse_pose");
    quality_topic_ = declare_parameter<std::string>(
      "quality_topic", "/relocalization/coarse_quality");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/relocalization/coarse_status");
    trigger_service_ = declare_parameter<std::string>(
      "trigger_service", "/relocalization/coarse_trigger");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    map_pcd_path_ = declare_parameter<std::string>("map_pcd_path", "");
    pcd_grid_resolution_ = std::max(
      0.02, declare_parameter<double>("pcd_grid_resolution", 0.05));
    pcd_min_z_ = declare_parameter<double>("pcd_min_z", 0.35);
    pcd_max_z_ = declare_parameter<double>("pcd_max_z", 1.80);
    pcd_map_padding_ = std::max(
      0.0, declare_parameter<double>("pcd_map_padding", 0.50));
    pcd_occupied_radius_ = std::max(
      0.0, declare_parameter<double>("pcd_occupied_radius", 0.08));

    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 65);
    likelihood_sigma_ = declare_parameter<double>("likelihood_sigma", 0.20);
    max_obstacle_distance_ = declare_parameter<double>("max_obstacle_distance", 1.0);
    scan_sample_step_ = std::max(
      1, static_cast<int>(declare_parameter<int>("scan_sample_step", 3)));
    min_scan_points_ = std::max(
      1, static_cast<int>(declare_parameter<int>("min_scan_points", 60)));
    min_range_ = declare_parameter<double>("min_range", 0.35);
    max_range_ = declare_parameter<double>("max_range", 20.0);
    angular_search_window_ = std::clamp(
      declare_parameter<double>("angular_search_window", kPi), 0.0, kPi);
    angular_step_ = std::max(
      declare_parameter<double>("angular_step", 0.0523598776), 0.001);
    max_depth_ = std::clamp(
      static_cast<int>(declare_parameter<int>("max_depth", 6)), 1, 10);
    max_expansions_ = std::max(
      1000, static_cast<int>(declare_parameter<int>("max_expansions", 300000)));
    max_search_time_sec_ = std::max(
      0.1, declare_parameter<double>("max_search_time_sec", 5.0));
    min_score_ = std::clamp(declare_parameter<double>("min_score", 0.55), 0.0, 1.0);
    min_score_gap_ = std::clamp(
      declare_parameter<double>("min_score_gap", 0.05), 0.0, 1.0);
    candidate_score_window_ = std::clamp(
      declare_parameter<double>("candidate_score_window", 0.20), min_score_gap_, 1.0);
    max_candidates_ = std::clamp(
      static_cast<int>(declare_parameter<int>("max_candidates", 8)), 2, 32);
    ambiguity_translation_ = std::max(
      0.0, declare_parameter<double>("ambiguity_translation", 0.75));
    ambiguity_yaw_ = std::max(
      0.0, declare_parameter<double>("ambiguity_yaw", 0.35));

    auto_match_ = declare_parameter<bool>("auto_match", false);
    min_interval_sec_ = std::max(
      0.0, declare_parameter<double>("min_interval_sec", 5.0));

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&BnbLocalizationNode::mapCallback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&BnbLocalizationNode::scanCallback, this, std::placeholders::_1));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    quality_pub_ = create_publisher<std_msgs::msg::String>(quality_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    trigger_srv_ = create_service<std_srvs::srv::Trigger>(
      trigger_service_,
      std::bind(
        &BnbLocalizationNode::triggerCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    if (!map_pcd_path_.empty()) {
      pcd_map_loaded_ = loadPcdOccupancyMap();
    }

    RCLCPP_INFO(
      get_logger(),
      "2D BnB localization ready: map=%s scan=%s trigger=%s yaw_step=%.2fdeg depth=%d",
      map_topic_.c_str(), scan_topic_.c_str(), trigger_service_.c_str(),
      angular_step_ * 180.0 / kPi, max_depth_);
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (pcd_map_loaded_) {
      return;
    }
    if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0F) {
      publishRejected("occupancy map metadata is invalid");
      return;
    }
    map_ = msg;
    buildLikelihoodPyramid();
    RCLCPP_INFO(
      get_logger(), "BnB map ready: %ux%u resolution=%.3f levels=%zu",
      map_->info.width, map_->info.height, map_->info.resolution, pyramid_.size());
  }

  bool loadPcdOccupancyMap()
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(map_pcd_path_, cloud) != 0 || cloud.empty()) {
      RCLCPP_ERROR(get_logger(), "failed to load BnB PCD map: %s", map_pcd_path_.c_str());
      return false;
    }
    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        continue;
      }
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
      max_x = std::max(max_x, point.x);
      max_y = std::max(max_y, point.y);
    }
    if (!std::isfinite(min_x) || !std::isfinite(min_y)) {
      RCLCPP_ERROR(get_logger(), "BnB PCD map contains no finite XY points");
      return false;
    }

    const double resolution = pcd_grid_resolution_;
    const double origin_x = std::floor((min_x - pcd_map_padding_) / resolution) * resolution;
    const double origin_y = std::floor((min_y - pcd_map_padding_) / resolution) * resolution;
    const auto width = static_cast<std::uint32_t>(
      std::ceil((max_x + pcd_map_padding_ - origin_x) / resolution) + 1.0);
    const auto height = static_cast<std::uint32_t>(
      std::ceil((max_y + pcd_map_padding_ - origin_y) / resolution) + 1.0);
    map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map_->header.frame_id = map_frame_;
    map_->info.resolution = static_cast<float>(resolution);
    map_->info.width = width;
    map_->info.height = height;
    map_->info.origin.position.x = origin_x;
    map_->info.origin.position.y = origin_y;
    map_->info.origin.orientation.w = 1.0;
    map_->data.assign(static_cast<std::size_t>(width) * height, 0);

    const int radius_cells = static_cast<int>(std::ceil(pcd_occupied_radius_ / resolution));
    std::size_t used_points = 0;
    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        point.z < pcd_min_z_ || point.z > pcd_max_z_)
      {
        continue;
      }
      const int gx = static_cast<int>(std::floor((point.x - origin_x) / resolution));
      const int gy = static_cast<int>(std::floor((point.y - origin_y) / resolution));
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          if (dx * dx + dy * dy > radius_cells * radius_cells) {
            continue;
          }
          const int x = gx + dx;
          const int y = gy + dy;
          if (x >= 0 && y >= 0 && x < static_cast<int>(width) && y < static_cast<int>(height)) {
            map_->data[static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)] = 100;
          }
        }
      }
      ++used_points;
    }
    if (used_points < static_cast<std::size_t>(min_scan_points_)) {
      RCLCPP_ERROR(
        get_logger(), "BnB PCD height filter retained too few points: %zu", used_points);
      map_.reset();
      return false;
    }
    buildLikelihoodPyramid();
    RCLCPP_INFO(
      get_logger(),
      "BnB PCD occupancy ready: points=%zu grid=%ux%u resolution=%.3f z=[%.2f,%.2f]",
      used_points, width, height, resolution, pcd_min_z_, pcd_max_z_);
    return true;
  }

  void triggerCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!map_ || pyramid_.empty()) {
      response->success = false;
      response->message = "occupancy map is not ready";
      return;
    }
    pending_trigger_ = true;
    response->success = true;
    response->message = "2D BnB localization will run on the next scan";
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto now = this->now();
    const bool interval_ok =
      last_match_time_.nanoseconds() == 0 ||
      (now - last_match_time_).seconds() >= min_interval_sec_;
    if (!pending_trigger_ && !(auto_match_ && interval_ok)) {
      return;
    }
    pending_trigger_ = false;
    last_match_time_ = now;
    runMatch(*msg);
  }

  void buildLikelihoodPyramid()
  {
    const int width = static_cast<int>(map_->info.width);
    const int height = static_cast<int>(map_->info.height);
    const int cell_count = width * height;
    const float resolution = map_->info.resolution;
    const float max_distance = static_cast<float>(max_obstacle_distance_);
    const float infinity = std::numeric_limits<float>::infinity();
    std::vector<float> distances(static_cast<std::size_t>(cell_count), infinity);
    std::priority_queue<DistanceCell, std::vector<DistanceCell>, DistanceCellGreater> queue;

    for (int index = 0; index < cell_count; ++index) {
      if (map_->data[static_cast<std::size_t>(index)] >= occupied_threshold_) {
        distances[static_cast<std::size_t>(index)] = 0.0F;
        queue.push({index, 0.0F});
      }
    }

    constexpr int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    constexpr int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    while (!queue.empty()) {
      const auto current = queue.top();
      queue.pop();
      if (current.distance != distances[static_cast<std::size_t>(current.index)] ||
        current.distance >= max_distance)
      {
        continue;
      }
      const int x = current.index % width;
      const int y = current.index / width;
      for (int direction = 0; direction < 8; ++direction) {
        const int nx = x + dx[direction];
        const int ny = y + dy[direction];
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
          continue;
        }
        const float step = resolution * ((direction < 4) ? 1.0F : std::sqrt(2.0F));
        const float candidate = current.distance + step;
        const int next_index = ny * width + nx;
        if (candidate < distances[static_cast<std::size_t>(next_index)] &&
          candidate <= max_distance)
        {
          distances[static_cast<std::size_t>(next_index)] = candidate;
          queue.push({next_index, candidate});
        }
      }
    }

    pyramid_.clear();
    pyramid_.resize(static_cast<std::size_t>(max_depth_ + 1));
    auto & base = pyramid_.front();
    base.resize(static_cast<std::size_t>(cell_count), 0.0F);
    const double sigma_squared = std::max(likelihood_sigma_ * likelihood_sigma_, 1e-6);
    for (int index = 0; index < cell_count; ++index) {
      if (map_->data[static_cast<std::size_t>(index)] < 0 ||
        !std::isfinite(distances[static_cast<std::size_t>(index)]))
      {
        continue;
      }
      const double distance = distances[static_cast<std::size_t>(index)];
      base[static_cast<std::size_t>(index)] = static_cast<float>(
        std::exp(-0.5 * distance * distance / sigma_squared));
    }

    for (int level = 1; level <= max_depth_; ++level) {
      const int offset = 1 << (level - 1);
      auto & grid = pyramid_[static_cast<std::size_t>(level)];
      const auto & previous = pyramid_[static_cast<std::size_t>(level - 1)];
      grid.resize(static_cast<std::size_t>(cell_count), 0.0F);
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          float value = 0.0F;
          for (int oy : {0, offset}) {
            for (int ox : {0, offset}) {
              const int sx = x + ox;
              const int sy = y + oy;
              if (sx < width && sy < height) {
                value = std::max(
                  value, previous[static_cast<std::size_t>(sy * width + sx)]);
              }
            }
          }
          grid[static_cast<std::size_t>(y * width + x)] = value;
        }
      }
    }
  }

  bool scanPointsInBase(
    const sensor_msgs::msg::LaserScan & scan,
    std::vector<std::pair<float, float>> & points)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        base_frame_, scan.header.frame_id, rclcpp::Time(scan.header.stamp),
        rclcpp::Duration::from_seconds(0.2));
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "BnB scan TF unavailable: %s", error.what());
      publishRejected(std::string("scan TF unavailable: ") + error.what());
      return false;
    }

    const double sensor_yaw = yawFromQuaternion(transform.transform.rotation);
    const double cos_sensor = std::cos(sensor_yaw);
    const double sin_sensor = std::sin(sensor_yaw);
    const double tx = transform.transform.translation.x;
    const double ty = transform.transform.translation.y;
    const double lower_range = std::max<double>(scan.range_min, min_range_);
    const double upper_range = std::min<double>(scan.range_max, max_range_);

    points.clear();
    points.reserve(scan.ranges.size() / static_cast<std::size_t>(scan_sample_step_) + 1U);
    for (std::size_t index = 0; index < scan.ranges.size();
      index += static_cast<std::size_t>(scan_sample_step_))
    {
      const double range = scan.ranges[index];
      if (!std::isfinite(range) || range < lower_range || range > upper_range) {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
      const double sx = range * std::cos(angle);
      const double sy = range * std::sin(angle);
      points.emplace_back(
        static_cast<float>(tx + cos_sensor * sx - sin_sensor * sy),
        static_cast<float>(ty + sin_sensor * sx + cos_sensor * sy));
    }
    return true;
  }

  std::vector<double> buildAngles() const
  {
    std::vector<double> angles;
    const int count = std::max(
      1, static_cast<int>(std::ceil(2.0 * angular_search_window_ / angular_step_)));
    angles.reserve(static_cast<std::size_t>(count + 1));
    for (int index = 0; index <= count; ++index) {
      const double ratio = static_cast<double>(index) / static_cast<double>(count);
      angles.push_back(-angular_search_window_ + 2.0 * angular_search_window_ * ratio);
    }
    return angles;
  }

  std::vector<std::vector<GridPoint>> discretizeRotatedScans(
    const std::vector<std::pair<float, float>> & points,
    const std::vector<double> & angles) const
  {
    const double map_yaw = yawFromQuaternion(map_->info.origin.orientation);
    const double inverse_resolution = 1.0 / map_->info.resolution;
    std::vector<std::vector<GridPoint>> rotated;
    rotated.reserve(angles.size());
    for (const double angle : angles) {
      const double grid_angle = angle - map_yaw;
      const double cosine = std::cos(grid_angle);
      const double sine = std::sin(grid_angle);
      auto & scan = rotated.emplace_back();
      scan.reserve(points.size());
      for (const auto & point : points) {
        scan.push_back({
          static_cast<int>(std::floor((cosine * point.first - sine * point.second) * inverse_resolution)),
          static_cast<int>(std::floor((sine * point.first + cosine * point.second) * inverse_resolution)),
        });
      }
    }
    return rotated;
  }

  float scoreNode(
    const SearchNode & node,
    const std::vector<std::vector<GridPoint>> & rotated_scans) const
  {
    const int width = static_cast<int>(map_->info.width);
    const int height = static_cast<int>(map_->info.height);
    const auto & grid = pyramid_[static_cast<std::size_t>(node.level)];
    const auto & scan = rotated_scans[static_cast<std::size_t>(node.angle_index)];
    double score = 0.0;
    for (const auto & point : scan) {
      const int x = node.x + point.x;
      const int y = node.y + point.y;
      if (x >= 0 && x < width && y >= 0 && y < height) {
        score += grid[static_cast<std::size_t>(y * width + x)];
      }
    }
    return static_cast<float>(score / std::max<std::size_t>(scan.size(), 1U));
  }

  bool candidateBaseCellIsFree(int x, int y) const
  {
    const int width = static_cast<int>(map_->info.width);
    const int height = static_cast<int>(map_->info.height);
    if (x < 0 || x >= width || y < 0 || y >= height) {
      return false;
    }
    const int8_t value = map_->data[static_cast<std::size_t>(y * width + x)];
    return value >= 0 && value < occupied_threshold_;
  }

  bool candidatesAreDistinct(
    const Candidate & lhs, const Candidate & rhs,
    const std::vector<double> & angles) const
  {
    const double resolution = map_->info.resolution;
    const double translation = std::hypot(lhs.x - rhs.x, lhs.y - rhs.y) * resolution;
    const double yaw = std::abs(normalizeAngle(
      angles[static_cast<std::size_t>(lhs.angle_index)] -
      angles[static_cast<std::size_t>(rhs.angle_index)]));
    return translation >= ambiguity_translation_ || yaw >= ambiguity_yaw_;
  }

  void runMatch(const sensor_msgs::msg::LaserScan & scan)
  {
    if (!map_ || pyramid_.empty()) {
      publishRejected("occupancy map is not ready");
      return;
    }

    std::vector<std::pair<float, float>> points;
    if (!scanPointsInBase(scan, points)) {
      return;
    }
    if (points.size() < static_cast<std::size_t>(min_scan_points_)) {
      publishRejected("scan has too few valid points");
      return;
    }

    const auto started = std::chrono::steady_clock::now();
    const auto angles = buildAngles();
    const auto rotated_scans = discretizeRotatedScans(points, angles);
    const int width = static_cast<int>(map_->info.width);
    const int height = static_cast<int>(map_->info.height);
    const int root_size = 1 << max_depth_;
    std::priority_queue<SearchNode, std::vector<SearchNode>, SearchNodeLess> queue;

    for (int angle_index = 0; angle_index < static_cast<int>(angles.size()); ++angle_index) {
      for (int y = 0; y < height; y += root_size) {
        for (int x = 0; x < width; x += root_size) {
          SearchNode node{x, y, max_depth_, angle_index, 0.0F};
          node.upper_bound = scoreNode(node, rotated_scans);
          if (node.upper_bound >= min_score_) {
            queue.push(node);
          }
        }
      }
    }

    Candidate best;
    Candidate second;
    std::vector<Candidate> candidates;
    int expansions = 0;
    bool search_complete = true;
    while (!queue.empty()) {
      if (std::isfinite(best.score) &&
        queue.top().upper_bound < best.score - static_cast<float>(candidate_score_window_))
      {
        break;
      }
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
      if (expansions >= max_expansions_ || elapsed >= max_search_time_sec_) {
        search_complete = false;
        break;
      }

      const SearchNode node = queue.top();
      queue.pop();
      ++expansions;
      if (node.level == 0) {
        if (!candidateBaseCellIsFree(node.x, node.y)) {
          continue;
        }
        const Candidate candidate{node.x, node.y, node.angle_index, node.upper_bound};
        bool merged = false;
        for (auto & existing : candidates) {
          if (!candidatesAreDistinct(candidate, existing, angles)) {
            if (candidate.score > existing.score) {
              existing = candidate;
            }
            merged = true;
            break;
          }
        }
        if (!merged) {
          candidates.push_back(candidate);
        }
        std::sort(candidates.begin(), candidates.end(), [](const auto & lhs, const auto & rhs) {
          return lhs.score > rhs.score;
        });
        if (candidates.size() > static_cast<std::size_t>(max_candidates_)) {
          candidates.resize(static_cast<std::size_t>(max_candidates_));
        }
        best = candidates.front();
        second = candidates.size() > 1U ? candidates[1] : Candidate{};
        continue;
      }

      const int child_level = node.level - 1;
      const int child_offset = 1 << child_level;
      for (int dy : {0, child_offset}) {
        for (int dx : {0, child_offset}) {
          const int child_x = node.x + dx;
          const int child_y = node.y + dy;
          if (child_x >= width || child_y >= height) {
            continue;
          }
          SearchNode child{
            child_x, child_y, child_level, node.angle_index, 0.0F};
          child.upper_bound = scoreNode(child, rotated_scans);
          if (child.upper_bound >= min_score_ &&
            (!std::isfinite(best.score) ||
            child.upper_bound >= best.score - static_cast<float>(candidate_score_window_)))
          {
            queue.push(child);
          }
        }
      }
    }

    // 未展开节点的上界也属于潜在竞争者。把它纳入第二名分数，避免质量状态
    // 把“已经证明低于分差门限”误报成完全不存在第二候选。
    if (!queue.empty() && queue.top().upper_bound > second.score) {
      const auto & remaining = queue.top();
      second = Candidate{
        remaining.x, remaining.y, remaining.angle_index, remaining.upper_bound};
    }

    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
    if (!std::isfinite(best.score)) {
      publishSearchResult(scan, best, second, candidates, angles, points.size(), expansions, elapsed, false,
        "no candidate reached the minimum score");
      return;
    }

    const double second_score = std::isfinite(second.score) ? second.score : 0.0;
    const double score_gap = best.score - second_score;
    const bool accepted = search_complete && best.score >= min_score_ && score_gap >= min_score_gap_;
    std::string reason = "accepted";
    if (!search_complete) {
      reason = "search budget exhausted before uniqueness was proven";
    } else if (best.score < min_score_) {
      reason = "best score is below threshold";
    } else if (score_gap < min_score_gap_) {
      reason = "best candidate is ambiguous";
    }
    publishSearchResult(
      scan, best, second, candidates, angles, points.size(), expansions, elapsed, accepted, reason);
  }

  void publishSearchResult(
    const sensor_msgs::msg::LaserScan & scan,
    const Candidate & best,
    const Candidate & second,
    const std::vector<Candidate> & candidates,
    const std::vector<double> & angles,
    std::size_t scan_points,
    int expansions,
    double elapsed,
    bool accepted,
    const std::string & reason)
  {
    const double best_score = std::isfinite(best.score) ? best.score : 0.0;
    const double second_score = std::isfinite(second.score) ? second.score : 0.0;
    const double score_gap = best_score - second_score;
    const auto candidatePose = [this, &angles](const Candidate & candidate) {
        const double resolution = map_->info.resolution;
        const double local_x = (static_cast<double>(candidate.x) + 0.5) * resolution;
        const double local_y = (static_cast<double>(candidate.y) + 0.5) * resolution;
        const double map_yaw = yawFromQuaternion(map_->info.origin.orientation);
        const double cosine = std::cos(map_yaw);
        const double sine = std::sin(map_yaw);
        return std::vector<double>{
          map_->info.origin.position.x + cosine * local_x - sine * local_y,
          map_->info.origin.position.y + sine * local_x + cosine * local_y,
          angles[static_cast<std::size_t>(candidate.angle_index)],
        };
      };
    const auto best_pose = std::isfinite(best.score) ?
      candidatePose(best) : std::vector<double>{0.0, 0.0, 0.0};
    const auto second_pose = std::isfinite(second.score) ?
      candidatePose(second) : std::vector<double>{0.0, 0.0, 0.0};
    std::ostringstream quality;
    quality << std::fixed << std::setprecision(6)
            << "{\"available\":true"
            << ",\"accepted\":" << (accepted ? "true" : "false")
            << ",\"method\":\"bnb_2d\""
            << ",\"score\":" << best_score
            << ",\"second_score\":" << second_score
            << ",\"score_gap\":" << score_gap
            << ",\"score_threshold\":" << min_score_
            << ",\"score_gap_threshold\":" << min_score_gap_
            << ",\"x\":" << best_pose[0]
            << ",\"y\":" << best_pose[1]
            << ",\"yaw\":" << best_pose[2]
            << ",\"second_x\":" << second_pose[0]
            << ",\"second_y\":" << second_pose[1]
            << ",\"second_yaw\":" << second_pose[2]
            << ",\"candidates\":[";
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      if (index > 0U) {
        quality << ",";
      }
      const auto pose = candidatePose(candidates[index]);
      quality << "{\"rank\":" << (index + 1U)
              << ",\"score\":" << candidates[index].score
              << ",\"x\":" << pose[0]
              << ",\"y\":" << pose[1]
              << ",\"yaw\":" << pose[2] << "}";
    }
    quality << "]"
            << ",\"scan_points\":" << scan_points
            << ",\"expansions\":" << expansions
            << ",\"elapsed_sec\":" << elapsed
            << ",\"reason\":\"" << reason << "\""
            << ",\"stamp_sec\":" << scan.header.stamp.sec
            << ",\"stamp_nanosec\":" << scan.header.stamp.nanosec
            << "}";
    std_msgs::msg::String quality_msg;
    quality_msg.data = quality.str();
    quality_pub_->publish(quality_msg);

    std_msgs::msg::String status_msg;
    std::ostringstream status;
    status << (accepted ? "OK: " : "WARN: ") << "2D BnB " << reason
           << " score=" << best_score << " second=" << second_score
           << " gap=" << score_gap << " expansions=" << expansions
           << " elapsed=" << elapsed << "s"
           << " best=(" << best_pose[0] << "," << best_pose[1] << "," << best_pose[2] << ")"
           << " second=(" << second_pose[0] << "," << second_pose[1] << "," << second_pose[2] << ")";
    status_msg.data = status.str();
    status_pub_->publish(status_msg);
    if (!accepted) {
      RCLCPP_WARN(get_logger(), "%s", status_msg.data.c_str());
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = scan.header;
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = best_pose[0];
    pose.pose.position.y = best_pose[1];
    pose.pose.position.z = 0.0;
    const double yaw = best_pose[2];
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
    pose_pub_->publish(pose);
    RCLCPP_INFO(
      get_logger(), "%s pose=(%.3f, %.3f, %.3f)", status_msg.data.c_str(),
      pose.pose.position.x, pose.pose.position.y, yaw);
  }

  void publishRejected(const std::string & reason)
  {
    std_msgs::msg::String quality;
    quality.data =
      "{\"available\":false,\"accepted\":false,\"method\":\"bnb_2d\","
      "\"score\":0.0,\"reason\":\"" + reason + "\"}";
    quality_pub_->publish(quality);
    std_msgs::msg::String status;
    status.data = "WARN: 2D BnB " + reason;
    status_pub_->publish(status);
  }

  std::string map_topic_;
  std::string scan_topic_;
  std::string pose_topic_;
  std::string quality_topic_;
  std::string status_topic_;
  std::string trigger_service_;
  std::string map_frame_;
  std::string base_frame_;
  std::string map_pcd_path_;
  double pcd_grid_resolution_{};
  double pcd_min_z_{};
  double pcd_max_z_{};
  double pcd_map_padding_{};
  double pcd_occupied_radius_{};
  bool pcd_map_loaded_{false};
  int occupied_threshold_{};
  double likelihood_sigma_{};
  double max_obstacle_distance_{};
  int scan_sample_step_{};
  int min_scan_points_{};
  double min_range_{};
  double max_range_{};
  double angular_search_window_{};
  double angular_step_{};
  int max_depth_{};
  int max_expansions_{};
  double max_search_time_sec_{};
  double min_score_{};
  double min_score_gap_{};
  double candidate_score_window_{};
  int max_candidates_{};
  double ambiguity_translation_{};
  double ambiguity_yaw_{};
  bool auto_match_{};
  double min_interval_sec_{};
  bool pending_trigger_{false};

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::vector<std::vector<float>> pyramid_;
  rclcpp::Time last_match_time_{0, 0, RCL_ROS_TIME};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr quality_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
};
}  // namespace cloud_relocalization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cloud_relocalization::BnbLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
