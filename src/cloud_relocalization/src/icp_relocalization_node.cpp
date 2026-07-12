#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <small_gicp/pcl/pcl_registration.hpp>

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
    quality_topic_ = declare_parameter<std::string>("quality_topic", "/relocalization/quality");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/relocalization/pose");
    initial_guess_topic_ = declare_parameter<std::string>(
      "initial_guess_topic", "/relocalization/initial_guess");
    trigger_service_ = declare_parameter<std::string>(
      "trigger_service", "/relocalization/trigger");
    readiness_service_ = declare_parameter<std::string>(
      "readiness_service", "/relocalization/ready");

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    auto_align_ = declare_parameter<bool>("auto_align", false);
    use_last_transform_as_guess_ = declare_parameter<bool>("use_last_transform_as_guess", true);
    // 通过一个参数保留 ICP/GICP/NDT 三种重定位后端，便于同一套输入、门限和发布逻辑下
    // 对比不同点云配准策略：ICP 快，GICP 稳，NDT 更适合粗配准或体素化地图。
    registration_method_ = normalizeMethod(
      declare_parameter<std::string>("registration_method", "small_gicp"));

    map_leaf_size_ = declare_parameter<double>("map_leaf_size", 0.12);
    scan_leaf_size_ = declare_parameter<double>("scan_leaf_size", 0.10);
    max_correspondence_distance_ =
      declare_parameter<double>("max_correspondence_distance", 1.0);
    transformation_epsilon_ = declare_parameter<double>("transformation_epsilon", 1e-5);
    euclidean_fitness_epsilon_ =
      declare_parameter<double>("euclidean_fitness_epsilon", 1e-4);
    fitness_score_threshold_ = declare_parameter<double>("fitness_score_threshold", 0.45);
    max_iterations_ = declare_parameter<int>("max_iterations", 45);
    ndt_resolution_ = declare_parameter<double>("ndt_resolution", 1.0);
    ndt_step_size_ = declare_parameter<double>("ndt_step_size", 0.1);
    small_gicp_num_threads_ = declare_parameter<int>("small_gicp_num_threads", 4);
    small_gicp_correspondence_randomness_ =
      declare_parameter<int>("small_gicp_correspondence_randomness", 20);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 120);
    min_map_points_ = declare_parameter<int>("min_map_points", 300);
    min_interval_sec_ = declare_parameter<double>("min_interval_sec", 2.0);
    crop_map_around_guess_ = declare_parameter<bool>("crop_map_around_guess", true);
    local_map_radius_ = declare_parameter<double>("local_map_radius", 8.0);
    max_result_translation_jump_ = declare_parameter<double>("max_result_translation_jump", 1.5);
    max_result_yaw_jump_ = declare_parameter<double>("max_result_yaw_jump", 0.8);
    max_result_z_jump_ = declare_parameter<double>("max_result_z_jump", 0.40);
    max_result_roll_pitch_jump_ = declare_parameter<double>(
      "max_result_roll_pitch_jump", 0.30);
    min_overlap_ratio_ = declare_parameter<double>("min_overlap_ratio", 0.20);

    initial_transform_ = initialTransformFromParameters();
    last_transform_ = initial_transform_;
    map_cloud_ = std::make_shared<CloudT>();

    map_loaded_ = loadMap();

    aligned_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      aligned_cloud_topic_, rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    quality_pub_ = create_publisher<std_msgs::msg::String>(quality_topic_, 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    initial_guess_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      initial_guess_topic_, 10,
      std::bind(&IcpRelocalizationNode::initialGuessCallback, this, std::placeholders::_1));
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
    readiness_srv_ = create_service<std_srvs::srv::Trigger>(
      readiness_service_,
      std::bind(
        &IcpRelocalizationNode::readinessCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "cloud relocalization ready: method=%s map=%s cloud=%s service=%s loaded=%s",
      registration_method_.c_str(), map_pcd_path_.c_str(), input_cloud_topic_.c_str(),
      trigger_service_.c_str(), map_loaded_ ? "true" : "false");
  }

private:
  void initialGuessCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        get_logger(), "ignore initial guess in frame '%s'; expected '%s'",
        msg->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    const auto & q = msg->pose.orientation;
    Eigen::Quaternionf rotation(
      static_cast<float>(q.w), static_cast<float>(q.x),
      static_cast<float>(q.y), static_cast<float>(q.z));
    if (rotation.norm() < 1e-6F) {
      RCLCPP_WARN(get_logger(), "ignore initial guess with invalid quaternion");
      return;
    }
    rotation.normalize();
    last_transform_ = Eigen::Matrix4f::Identity();
    last_transform_.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    last_transform_(0, 3) = static_cast<float>(msg->pose.position.x);
    last_transform_(1, 3) = static_cast<float>(msg->pose.position.y);
    last_transform_(2, 3) = static_cast<float>(msg->pose.position.z);
    RCLCPP_INFO(
      get_logger(), "updated registration initial guess: x=%.3f y=%.3f z=%.3f",
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }

  void readinessCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) const
  {
    response->success = map_loaded_;
    std::ostringstream oss;
    oss << (map_loaded_ ? "PCD map is ready" : "PCD map is unavailable")
        << ": points=" << map_cloud_->size()
        << " min=" << min_map_points_;
    response->message = oss.str();
  }

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
    response->message = registrationLabel() + " relocalization will run on the next cloud";
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
    runRelocalization(*msg);
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
    if (map_cloud_->size() < static_cast<std::size_t>(std::max(1, min_map_points_))) {
      RCLCPP_ERROR(
        get_logger(), "PCD map has too few filtered points: %zu min=%d",
        map_cloud_->size(), min_map_points_);
      return false;
    }
    return true;
  }

  void runRelocalization(const sensor_msgs::msg::PointCloud2 & msg)
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

    CloudT aligned;
    Eigen::Matrix4f result_transform = Eigen::Matrix4f::Identity();
    double fitness = std::numeric_limits<double>::infinity();
    bool converged = false;

    // 三个分支只替换配准后端，前后的点云过滤、局部地图裁剪、fitness 门限和跳变门控保持一致。
    // 这样重定位行为差异主要来自算法本身，而不是流程或安全条件不一致。
    if (registration_method_ == "small_gicp") {
      // 使用 small_gicp 的 PCL 适配器，沿用现有 PCL 点云、初值和输出接口。
      small_gicp::RegistrationPCL<PointT, PointT> small_gicp;
      small_gicp.setRegistrationType("GICP");
      small_gicp.setNumThreads(std::max(1, small_gicp_num_threads_));
      small_gicp.setCorrespondenceRandomness(
        std::max(5, small_gicp_correspondence_randomness_));
      configureIcpLikeRegistration(small_gicp, filtered_scan, target_map);
      small_gicp.align(aligned, guess);
      converged = small_gicp.hasConverged();
      fitness = small_gicp.getFitnessScore();
      result_transform = small_gicp.getFinalTransformation();
    } else if (registration_method_ == "gicp") {
      // GICP 会估计局部几何协方差，比普通 ICP 更能利用 3D 面结构；代价是计算更重。
      pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
      configureIcpLikeRegistration(gicp, filtered_scan, target_map);
      gicp.align(aligned, guess);
      converged = gicp.hasConverged();
      fitness = gicp.getFitnessScore();
      result_transform = gicp.getFinalTransformation();
    } else if (registration_method_ == "ndt") {
      // NDT 把目标地图建成体素正态分布，适合点密度不均或初值略粗的场景；分辨率需要按地图尺度调。
      pcl::NormalDistributionsTransform<PointT, PointT> ndt;
      ndt.setInputSource(filtered_scan);
      ndt.setInputTarget(target_map);
      ndt.setMaximumIterations(std::max(1, max_iterations_));
      ndt.setTransformationEpsilon(transformation_epsilon_);
      ndt.setStepSize(std::max(0.01, ndt_step_size_));
      ndt.setResolution(std::max(0.1, ndt_resolution_));
      ndt.align(aligned, guess);
      converged = ndt.hasConverged();
      fitness = ndt.getFitnessScore();
      result_transform = ndt.getFinalTransformation();
    } else {
      // ICP 是默认后端，参数少、速度快，适合已有 AMCL/里程计给出较好初值后的局部精配准。
      pcl::IterativeClosestPoint<PointT, PointT> icp;
      configureIcpLikeRegistration(icp, filtered_scan, target_map);
      icp.align(aligned, guess);
      converged = icp.hasConverged();
      fitness = icp.getFitnessScore();
      result_transform = icp.getFinalTransformation();
    }

    if (!converged || fitness > fitness_score_threshold_) {
      std::ostringstream oss;
      oss << registrationLabel() << " rejected: converged=" << (converged ? "true" : "false")
          << " fitness=" << fitness
          << " threshold=" << fitness_score_threshold_;
      publishStatus(false, oss.str());
      return;
    }

    const std::string rejection_reason = resultRejectionReason(guess, result_transform);
    if (!rejection_reason.empty()) {
      publishStatus(false, registrationLabel() + " rejected: " + rejection_reason);
      return;
    }

    const double overlap_ratio = calculateOverlapRatio(aligned, target_map);
    if (overlap_ratio < min_overlap_ratio_) {
      std::ostringstream oss;
      oss << registrationLabel() << " rejected: overlap=" << overlap_ratio
          << " threshold=" << min_overlap_ratio_;
      publishStatus(false, oss.str());
      return;
    }

    last_transform_ = result_transform;
    publishResult(
      msg.header.stamp, aligned, fitness, overlap_ratio,
      filtered_scan->size(), target_map->size());
  }

  template<typename RegistrationT>
  void configureIcpLikeRegistration(
    RegistrationT & registration,
    const std::shared_ptr<CloudT> & source,
    const std::shared_ptr<CloudT> & target) const
  {
    registration.setInputSource(source);
    registration.setInputTarget(target);
    registration.setMaximumIterations(std::max(1, max_iterations_));
    registration.setMaxCorrespondenceDistance(max_correspondence_distance_);
    registration.setTransformationEpsilon(transformation_epsilon_);
    registration.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
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

  std::string resultRejectionReason(
    const Eigen::Matrix4f & guess,
    const Eigen::Matrix4f & result) const
  {
    const double dx = static_cast<double>(result(0, 3) - guess(0, 3));
    const double dy = static_cast<double>(result(1, 3) - guess(1, 3));
    const double dz = static_cast<double>(result(2, 3) - guess(2, 3));
    const double translation_jump = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (translation_jump > max_result_translation_jump_) {
      return "translation jump exceeds gate";
    }
    if (std::fabs(dz) > max_result_z_jump_) {
      return "z jump exceeds gate";
    }

    const double yaw_jump = std::fabs(normalizeAngle(yawFromMatrix(result) - yawFromMatrix(guess)));
    if (yaw_jump > max_result_yaw_jump_) {
      return "yaw jump exceeds gate";
    }

    const double roll_jump = std::fabs(normalizeAngle(
      rollFromMatrix(result) - rollFromMatrix(guess)));
    const double pitch_jump = std::fabs(normalizeAngle(
      pitchFromMatrix(result) - pitchFromMatrix(guess)));
    if (std::max(roll_jump, pitch_jump) > max_result_roll_pitch_jump_) {
      return "roll/pitch jump exceeds gate";
    }
    return "";
  }

  double yawFromMatrix(const Eigen::Matrix4f & matrix) const
  {
    return std::atan2(static_cast<double>(matrix(1, 0)), static_cast<double>(matrix(0, 0)));
  }

  double rollFromMatrix(const Eigen::Matrix4f & matrix) const
  {
    return std::atan2(static_cast<double>(matrix(2, 1)), static_cast<double>(matrix(2, 2)));
  }

  double pitchFromMatrix(const Eigen::Matrix4f & matrix) const
  {
    const double horizontal = std::hypot(
      static_cast<double>(matrix(2, 1)), static_cast<double>(matrix(2, 2)));
    return std::atan2(-static_cast<double>(matrix(2, 0)), horizontal);
  }

  double calculateOverlapRatio(
    const CloudT & aligned,
    const std::shared_ptr<CloudT> & target) const
  {
    if (aligned.empty() || target->empty()) {
      return 0.0;
    }

    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud(target);
    const float max_distance_squared = static_cast<float>(
      max_correspondence_distance_ * max_correspondence_distance_);
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    std::size_t overlap_count = 0;
    for (const auto & point : aligned.points) {
      if (tree.nearestKSearch(point, 1, indices, distances) > 0 &&
        distances[0] <= max_distance_squared)
      {
        ++overlap_count;
      }
    }
    return static_cast<double>(overlap_count) / static_cast<double>(aligned.size());
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
    const double fitness,
    const double overlap_ratio,
    const std::size_t scan_points,
    const std::size_t local_map_points)
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
    sensor_msgs::msg::PointCloud2 aligned_msg;
    pcl::toROSMsg(aligned, aligned_msg);
    aligned_msg.header.stamp = stamp;
    aligned_msg.header.frame_id = map_frame_;
    aligned_cloud_pub_->publish(aligned_msg);

    std::ostringstream oss;
    oss << registrationLabel() << " accepted: fitness=" << fitness
        << " overlap=" << overlap_ratio
        << " scan_points=" << scan_points
        << " local_map_points=" << local_map_points
        << " x=" << pose.pose.position.x
        << " y=" << pose.pose.position.y;
    publishStatus(true, oss.str());
    publishAcceptedQuality(
      stamp, fitness, overlap_ratio, scan_points, local_map_points);
    pose_pub_->publish(pose);
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
      publishRejectedQuality(message);
    }
  }

  void publishAcceptedQuality(
    const builtin_interfaces::msg::Time & stamp,
    const double fitness,
    const double overlap_ratio,
    const std::size_t scan_points,
    const std::size_t local_map_points)
  {
    const double fitness_component = std::clamp(
      1.0 - fitness / std::max(fitness_score_threshold_, 1e-9), 0.0, 1.0);
    const double overlap_component = std::clamp(
      (overlap_ratio - min_overlap_ratio_) / std::max(1.0 - min_overlap_ratio_, 1e-9),
      0.0, 1.0);
    const double score = 100.0 * (0.55 * fitness_component + 0.45 * overlap_component);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "{\"available\":true,\"accepted\":true"
        << ",\"method\":\"" << jsonEscape(registration_method_) << "\""
        << ",\"score\":" << score
        << ",\"fitness\":" << fitness
        << ",\"fitness_threshold\":" << fitness_score_threshold_
        << ",\"overlap_ratio\":" << overlap_ratio
        << ",\"overlap_threshold\":" << min_overlap_ratio_
        << ",\"scan_points\":" << scan_points
        << ",\"local_map_points\":" << local_map_points
        << ",\"stamp_sec\":" << stamp.sec
        << ",\"stamp_nanosec\":" << stamp.nanosec
        << "}";
    std_msgs::msg::String msg;
    msg.data = oss.str();
    quality_pub_->publish(msg);
  }

  void publishRejectedQuality(const std::string & reason)
  {
    std::ostringstream oss;
    oss << "{\"available\":" << (map_loaded_ ? "true" : "false")
        << ",\"accepted\":false"
        << ",\"method\":\"" << jsonEscape(registration_method_) << "\""
        << ",\"score\":0.0"
        << ",\"reason\":\"" << jsonEscape(reason) << "\"}";
    std_msgs::msg::String msg;
    msg.data = oss.str();
    quality_pub_->publish(msg);
  }

  std::string jsonEscape(const std::string & input) const
  {
    std::string output;
    output.reserve(input.size());
    for (const char c : input) {
      if (c == '\\' || c == '"') {
        output.push_back('\\');
      }
      output.push_back(c);
    }
    return output;
  }

  std::string normalizeMethod(std::string method) const
  {
    std::transform(method.begin(), method.end(), method.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    // 配置入口只允许这三种 PCL 配准方法；写错时回退到默认 ICP，避免节点直接退出。
    if (method != "small_gicp" && method != "icp" && method != "gicp" && method != "ndt") {
      RCLCPP_WARN(
        get_logger(),
        "unsupported registration_method '%s', fallback to small_gicp",
        method.c_str());
      return "small_gicp";
    }
    return method;
  }

  std::string registrationLabel() const
  {
    if (registration_method_ == "small_gicp") {
      return "small_gicp";
    }
    if (registration_method_ == "gicp") {
      return "GICP";
    }
    if (registration_method_ == "ndt") {
      return "NDT";
    }
    return "ICP";
  }

  std::string map_pcd_path_;
  std::string input_cloud_topic_;
  std::string aligned_cloud_topic_;
  std::string status_topic_;
  std::string quality_topic_;
  std::string pose_topic_;
  std::string initial_guess_topic_;
  std::string trigger_service_;
  std::string readiness_service_;
  std::string map_frame_;
  std::string odom_frame_;
  bool publish_tf_{};
  bool auto_align_{};
  bool use_last_transform_as_guess_{};
  std::string registration_method_;
  double map_leaf_size_{};
  double scan_leaf_size_{};
  double max_correspondence_distance_{};
  double transformation_epsilon_{};
  double euclidean_fitness_epsilon_{};
  double fitness_score_threshold_{};
  int max_iterations_{};
  double ndt_resolution_{};
  double ndt_step_size_{};
  int small_gicp_num_threads_{};
  int small_gicp_correspondence_randomness_{};
  int min_scan_points_{};
  int min_map_points_{};
  double min_interval_sec_{};
  bool crop_map_around_guess_{};
  double local_map_radius_{};
  double max_result_translation_jump_{};
  double max_result_yaw_jump_{};
  double max_result_z_jump_{};
  double max_result_roll_pitch_jump_{};
  double min_overlap_ratio_{};

  bool map_loaded_{false};
  Eigen::Matrix4f initial_transform_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f last_transform_{Eigen::Matrix4f::Identity()};
  std::shared_ptr<CloudT> map_cloud_;
  rclcpp::Time last_alignment_time_{0, 0, RCL_ROS_TIME};
  std::atomic_bool pending_trigger_{false};
  std::atomic_bool aligning_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initial_guess_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr readiness_srv_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr quality_pub_;
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
