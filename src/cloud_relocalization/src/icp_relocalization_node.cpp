#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace cloud_relocalization
{
class IcpRelocalizationNode : public rclcpp::Node
{
public:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  IcpRelocalizationNode()
  : Node("icp_relocalization_node"),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    map_pcd_path_ = declare_parameter<std::string>("map_pcd_path", "");
    input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/cloud_registered");
    aligned_cloud_topic_ = declare_parameter<std::string>(
      "aligned_cloud_topic", "/relocalization/aligned_cloud");
    status_topic_ = declare_parameter<std::string>("status_topic", "/relocalization/status");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/relocalization/pose");
    trigger_service_ = declare_parameter<std::string>(
      "trigger_service", "/relocalization/trigger");

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    auto_align_ = declare_parameter<bool>("auto_align", false);
    use_last_transform_as_guess_ = declare_parameter<bool>("use_last_transform_as_guess", true);

    map_leaf_size_ = declare_parameter<double>("map_leaf_size", 0.12);
    scan_leaf_size_ = declare_parameter<double>("scan_leaf_size", 0.10);
    max_correspondence_distance_ =
      declare_parameter<double>("max_correspondence_distance", 1.0);
    transformation_epsilon_ = declare_parameter<double>("transformation_epsilon", 1e-5);
    euclidean_fitness_epsilon_ =
      declare_parameter<double>("euclidean_fitness_epsilon", 1e-4);
    fitness_score_threshold_ = declare_parameter<double>("fitness_score_threshold", 0.45);
    max_iterations_ = declare_parameter<int>("max_iterations", 45);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 120);
    min_map_points_ = declare_parameter<int>("min_map_points", 300);
    min_interval_sec_ = declare_parameter<double>("min_interval_sec", 2.0);
    crop_map_around_guess_ = declare_parameter<bool>("crop_map_around_guess", true);
    local_map_radius_ = declare_parameter<double>("local_map_radius", 8.0);
    max_result_translation_jump_ = declare_parameter<double>("max_result_translation_jump", 1.5);
    max_result_yaw_jump_ = declare_parameter<double>("max_result_yaw_jump", 0.8);

    initial_transform_ = initialTransformFromParameters();
    last_transform_ = initial_transform_;
    map_cloud_ = std::make_shared<CloudT>();

    map_loaded_ = loadMap();

    aligned_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      aligned_cloud_topic_, rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&IcpRelocalizationNode::cloudCallback, this, std::placeholders::_1));
    trigger_srv_ = create_service<std_srvs::srv::Trigger>(
      trigger_service_,
      std::bind(
        &IcpRelocalizationNode::triggerCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "ICP relocalization ready: map=%s cloud=%s service=%s loaded=%s",
      map_pcd_path_.c_str(), input_cloud_topic_.c_str(), trigger_service_.c_str(),
      map_loaded_ ? "true" : "false");
  }

private:
  void triggerCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!map_loaded_) {
      response->success = false;
      response->message = "PCD map is not loaded";
      return;
    }
    pending_trigger_.store(true);
    response->success = true;
    response->message = "ICP relocalization will run on the next cloud";
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!map_loaded_ || aligning_.load()) {
      return;
    }

    const auto now = this->now();
    const bool interval_ok = (now - last_alignment_time_).seconds() >= min_interval_sec_;
    const bool manual_trigger = pending_trigger_.load();
    const bool should_align = manual_trigger || (auto_align_ && interval_ok);
    if (!should_align) {
      return;
    }
    if (manual_trigger) {
      pending_trigger_.store(false);
    }

    aligning_.store(true);
    runIcp(*msg);
    last_alignment_time_ = now;
    aligning_.store(false);
  }

  bool loadMap()
  {
    if (map_pcd_path_.empty()) {
      RCLCPP_WARN(get_logger(), "map_pcd_path is empty; relocalization is disabled");
      return false;
    }

    auto raw_map = std::make_shared<CloudT>();
    if (pcl::io::loadPCDFile<PointT>(map_pcd_path_, *raw_map) != 0) {
      RCLCPP_ERROR(get_logger(), "failed to load PCD map: %s", map_pcd_path_.c_str());
      return false;
    }

    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*raw_map, *raw_map, valid_indices);
    map_cloud_ = downsample(raw_map, map_leaf_size_);
    RCLCPP_INFO(
      get_logger(), "loaded PCD map: raw=%zu filtered=%zu leaf=%.3f",
      raw_map->size(), map_cloud_->size(), map_leaf_size_);
    return !map_cloud_->empty();
  }

  void runIcp(const sensor_msgs::msg::PointCloud2 & msg)
  {
    auto scan = std::make_shared<CloudT>();
    pcl::fromROSMsg(msg, *scan);
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*scan, *scan, valid_indices);
    auto filtered_scan = downsample(scan, scan_leaf_size_);

    if (filtered_scan->size() < static_cast<std::size_t>(std::max(1, min_scan_points_))) {
      publishStatus(false, "scan has too few points after filtering");
      return;
    }

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    const Eigen::Matrix4f guess =
      use_last_transform_as_guess_ ? last_transform_ : initial_transform_;
    const auto target_map = targetMapForGuess(guess);
    if (target_map->size() < static_cast<std::size_t>(std::max(1, min_map_points_))) {
      std::ostringstream oss;
      oss << "local map has too few points: " << target_map->size()
          << " min=" << min_map_points_;
      publishStatus(false, oss.str());
      return;
    }

    icp.setInputSource(filtered_scan);
    icp.setInputTarget(target_map);
    icp.setMaximumIterations(std::max(1, max_iterations_));
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp.setTransformationEpsilon(transformation_epsilon_);
    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

    CloudT aligned;
    icp.align(aligned, guess);

    const double fitness = icp.getFitnessScore();
    if (!icp.hasConverged() || fitness > fitness_score_threshold_) {
      std::ostringstream oss;
      oss << "ICP rejected: converged=" << (icp.hasConverged() ? "true" : "false")
          << " fitness=" << fitness
          << " threshold=" << fitness_score_threshold_;
      publishStatus(false, oss.str());
      return;
    }

    const auto result_transform = icp.getFinalTransformation();
    if (!isResultAcceptable(guess, result_transform)) {
      publishStatus(false, "ICP rejected: result jump exceeds configured gate");
      return;
    }

    last_transform_ = result_transform;
    publishResult(msg.header.stamp, aligned, fitness);
  }

  std::shared_ptr<CloudT> targetMapForGuess(const Eigen::Matrix4f & guess) const
  {
    if (!crop_map_around_guess_) {
      return map_cloud_;
    }

    const float radius = static_cast<float>(std::max(local_map_radius_, 0.5));
    pcl::CropBox<PointT> crop_box;
    crop_box.setInputCloud(map_cloud_);
    crop_box.setMin(Eigen::Vector4f(
      guess(0, 3) - radius, guess(1, 3) - radius, guess(2, 3) - radius, 1.0F));
    crop_box.setMax(Eigen::Vector4f(
      guess(0, 3) + radius, guess(1, 3) + radius, guess(2, 3) + radius, 1.0F));

    auto cropped = std::make_shared<CloudT>();
    crop_box.filter(*cropped);
    return cropped;
  }

  bool isResultAcceptable(
    const Eigen::Matrix4f & guess,
    const Eigen::Matrix4f & result) const
  {
    const double dx = static_cast<double>(result(0, 3) - guess(0, 3));
    const double dy = static_cast<double>(result(1, 3) - guess(1, 3));
    const double dz = static_cast<double>(result(2, 3) - guess(2, 3));
    const double translation_jump = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (translation_jump > max_result_translation_jump_) {
      return false;
    }

    const double yaw_jump = std::fabs(normalizeAngle(yawFromMatrix(result) - yawFromMatrix(guess)));
    return yaw_jump <= max_result_yaw_jump_;
  }

  double yawFromMatrix(const Eigen::Matrix4f & matrix) const
  {
    return std::atan2(static_cast<double>(matrix(1, 0)), static_cast<double>(matrix(0, 0)));
  }

  double normalizeAngle(double angle) const
  {
    constexpr double pi = 3.14159265358979323846;
    while (angle > pi) {
      angle -= 2.0 * pi;
    }
    while (angle < -pi) {
      angle += 2.0 * pi;
    }
    return angle;
  }

  std::shared_ptr<CloudT> downsample(
    const std::shared_ptr<CloudT> & input,
    const double leaf_size) const
  {
    const float leaf = static_cast<float>(std::max(leaf_size, 0.01));
    auto output = std::make_shared<CloudT>();
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setLeafSize(leaf, leaf, leaf);
    voxel_grid.setInputCloud(input);
    voxel_grid.filter(*output);
    return output;
  }

  void publishResult(
    const builtin_interfaces::msg::Time & stamp,
    const CloudT & aligned,
    const double fitness)
  {
    if (publish_tf_) {
      tf_broadcaster_->sendTransform(matrixToTransform(stamp, last_transform_));
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = last_transform_(0, 3);
    pose.pose.position.y = last_transform_(1, 3);
    pose.pose.position.z = last_transform_(2, 3);
    const Eigen::Quaternionf q(last_transform_.block<3, 3>(0, 0));
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pose_pub_->publish(pose);

    sensor_msgs::msg::PointCloud2 aligned_msg;
    pcl::toROSMsg(aligned, aligned_msg);
    aligned_msg.header.stamp = stamp;
    aligned_msg.header.frame_id = map_frame_;
    aligned_cloud_pub_->publish(aligned_msg);

    std::ostringstream oss;
    oss << "ICP accepted: fitness=" << fitness
        << " x=" << pose.pose.position.x
        << " y=" << pose.pose.position.y;
    publishStatus(true, oss.str());
  }

  geometry_msgs::msg::TransformStamped matrixToTransform(
    const builtin_interfaces::msg::Time & stamp,
    const Eigen::Matrix4f & matrix) const
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = matrix(0, 3);
    transform.transform.translation.y = matrix(1, 3);
    transform.transform.translation.z = matrix(2, 3);
    const Eigen::Quaternionf q(matrix.block<3, 3>(0, 0));
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    return transform;
  }

  Eigen::Matrix4f initialTransformFromParameters()
  {
    const double x = declare_parameter<double>("initial_x", 0.0);
    const double y = declare_parameter<double>("initial_y", 0.0);
    const double z = declare_parameter<double>("initial_z", 0.0);
    const double roll = declare_parameter<double>("initial_roll", 0.0);
    const double pitch = declare_parameter<double>("initial_pitch", 0.0);
    const double yaw = declare_parameter<double>("initial_yaw", 0.0);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    const Eigen::AngleAxisf roll_angle(static_cast<float>(roll), Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf pitch_angle(static_cast<float>(pitch), Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf yaw_angle(static_cast<float>(yaw), Eigen::Vector3f::UnitZ());
    transform.block<3, 3>(0, 0) = (yaw_angle * pitch_angle * roll_angle).matrix();
    transform(0, 3) = static_cast<float>(x);
    transform(1, 3) = static_cast<float>(y);
    transform(2, 3) = static_cast<float>(z);
    return transform;
  }

  void publishStatus(const bool ok, const std::string & message)
  {
    std_msgs::msg::String msg;
    msg.data = std::string(ok ? "OK: " : "WARN: ") + message;
    status_pub_->publish(msg);
    if (ok) {
      RCLCPP_INFO(get_logger(), "%s", msg.data.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", msg.data.c_str());
    }
  }

  std::string map_pcd_path_;
  std::string input_cloud_topic_;
  std::string aligned_cloud_topic_;
  std::string status_topic_;
  std::string pose_topic_;
  std::string trigger_service_;
  std::string map_frame_;
  std::string odom_frame_;
  bool publish_tf_{};
  bool auto_align_{};
  bool use_last_transform_as_guess_{};
  double map_leaf_size_{};
  double scan_leaf_size_{};
  double max_correspondence_distance_{};
  double transformation_epsilon_{};
  double euclidean_fitness_epsilon_{};
  double fitness_score_threshold_{};
  int max_iterations_{};
  int min_scan_points_{};
  int min_map_points_{};
  double min_interval_sec_{};
  bool crop_map_around_guess_{};
  double local_map_radius_{};
  double max_result_translation_jump_{};
  double max_result_yaw_jump_{};

  bool map_loaded_{false};
  Eigen::Matrix4f initial_transform_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f last_transform_{Eigen::Matrix4f::Identity()};
  std::shared_ptr<CloudT> map_cloud_;
  rclcpp::Time last_alignment_time_{0, 0, RCL_ROS_TIME};
  std::atomic_bool pending_trigger_{false};
  std::atomic_bool aligning_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}  // namespace cloud_relocalization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cloud_relocalization::IcpRelocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
