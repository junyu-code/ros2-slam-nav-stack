// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{

template<typename T>
T clampValue(T value, T low, T high)
{
  return std::min(std::max(value, low), high);
}

double hypot2d(double x, double y)
{
  return std::hypot(x, y);
}

}  // namespace

namespace pb_omni_pid_pursuit_controller
{

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using rcl_interfaces::msg::ParameterType;

void OmniPidPursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock controller node");
  }

  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 0.1;
  double control_frequency = 20.0;

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kp", rclcpp::ParameterValue(translation_kp_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_ki", rclcpp::ParameterValue(translation_ki_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kd", rclcpp::ParameterValue(translation_kd_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kp", rclcpp::ParameterValue(rotation_kp_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_ki", rclcpp::ParameterValue(rotation_ki_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kd", rclcpp::ParameterValue(rotation_kd_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".limit_i_v", rclcpp::ParameterValue(limit_i_v_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".limit_i_w", rclcpp::ParameterValue(limit_i_w_));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(transform_tolerance));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(lookahead_dist_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    rclcpp::ParameterValue(use_velocity_scaled_lookahead_dist_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(min_lookahead_dist_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(max_lookahead_dist_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(lookahead_time_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_interpolation", rclcpp::ParameterValue(use_interpolation_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_rotation", rclcpp::ParameterValue(enable_rotation_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(use_rotate_to_heading_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_threshold",
    rclcpp::ParameterValue(rotate_to_heading_threshold_));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity",
    rclcpp::ParameterValue(min_approach_linear_velocity_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist",
    rclcpp::ParameterValue(approach_velocity_scaling_dist_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_min", rclcpp::ParameterValue(v_linear_min_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_max", rclcpp::ParameterValue(v_linear_max_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_min", rclcpp::ParameterValue(v_angular_min_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_max", rclcpp::ParameterValue(v_angular_max_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(getCostmapMaxExtent()));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_curvature_speed_limit",
    rclcpp::ParameterValue(enable_curvature_speed_limit_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_min", rclcpp::ParameterValue(curvature_min_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_max", rclcpp::ParameterValue(curvature_max_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".reduction_ratio_at_high_curvature",
    rclcpp::ParameterValue(reduction_ratio_at_high_curvature_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_forward_dist", rclcpp::ParameterValue(curvature_forward_dist_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_backward_dist",
    rclcpp::ParameterValue(curvature_backward_dist_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_velocity_scaling_factor_rate",
    rclcpp::ParameterValue(max_velocity_scaling_factor_rate_));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".odom_topic", rclcpp::ParameterValue(odom_topic_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_stuck_escape", rclcpp::ParameterValue(enable_stuck_escape_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_stuck_status", rclcpp::ParameterValue(publish_stuck_status_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".fail_after_escape_attempts",
    rclcpp::ParameterValue(fail_after_escape_attempts_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_escape_attempts", rclcpp::ParameterValue(max_escape_attempts_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".escape_duration", rclcpp::ParameterValue(escape_duration_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".escape_linear_speed", rclcpp::ParameterValue(escape_linear_speed_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".stuck_duration_threshold",
    rclcpp::ParameterValue(stuck_duration_threshold_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_actual_speed_threshold",
    rclcpp::ParameterValue(min_actual_speed_threshold_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_command_speed_threshold",
    rclcpp::ParameterValue(max_command_speed_threshold_));

  node->get_parameter(plugin_name_ + ".translation_kp", translation_kp_);
  node->get_parameter(plugin_name_ + ".translation_ki", translation_ki_);
  node->get_parameter(plugin_name_ + ".translation_kd", translation_kd_);
  node->get_parameter(plugin_name_ + ".rotation_kp", rotation_kp_);
  node->get_parameter(plugin_name_ + ".rotation_ki", rotation_ki_);
  node->get_parameter(plugin_name_ + ".rotation_kd", rotation_kd_);
  node->get_parameter(plugin_name_ + ".limit_i_v", limit_i_v_);
  node->get_parameter(plugin_name_ + ".limit_i_w", limit_i_w_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".use_interpolation", use_interpolation_);
  node->get_parameter(plugin_name_ + ".enable_rotation", enable_rotation_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_threshold", rotate_to_heading_threshold_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".v_linear_min", v_linear_min_);
  node->get_parameter(plugin_name_ + ".v_linear_max", v_linear_max_);
  node->get_parameter(plugin_name_ + ".v_angular_min", v_angular_min_);
  node->get_parameter(plugin_name_ + ".v_angular_max", v_angular_max_);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
  node->get_parameter(plugin_name_ + ".enable_curvature_speed_limit", enable_curvature_speed_limit_);
  node->get_parameter(plugin_name_ + ".curvature_min", curvature_min_);
  node->get_parameter(plugin_name_ + ".curvature_max", curvature_max_);
  node->get_parameter(
    plugin_name_ + ".reduction_ratio_at_high_curvature", reduction_ratio_at_high_curvature_);
  node->get_parameter(plugin_name_ + ".curvature_forward_dist", curvature_forward_dist_);
  node->get_parameter(plugin_name_ + ".curvature_backward_dist", curvature_backward_dist_);
  node->get_parameter(
    plugin_name_ + ".max_velocity_scaling_factor_rate", max_velocity_scaling_factor_rate_);
  node->get_parameter(plugin_name_ + ".odom_topic", odom_topic_);
  node->get_parameter(plugin_name_ + ".enable_stuck_escape", enable_stuck_escape_);
  node->get_parameter(plugin_name_ + ".publish_stuck_status", publish_stuck_status_);
  node->get_parameter(plugin_name_ + ".fail_after_escape_attempts", fail_after_escape_attempts_);
  node->get_parameter(plugin_name_ + ".max_escape_attempts", max_escape_attempts_);
  node->get_parameter(plugin_name_ + ".escape_duration", escape_duration_);
  node->get_parameter(plugin_name_ + ".escape_linear_speed", escape_linear_speed_);
  node->get_parameter(plugin_name_ + ".stuck_duration_threshold", stuck_duration_threshold_);
  node->get_parameter(plugin_name_ + ".min_actual_speed_threshold", min_actual_speed_threshold_);
  node->get_parameter(plugin_name_ + ".max_command_speed_threshold", max_command_speed_threshold_);
  node->get_parameter("controller_frequency", control_frequency);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / std::max(1.0, control_frequency);
  nominal_v_linear_min_ = v_linear_min_;
  nominal_v_linear_max_ = v_linear_max_;
  max_robot_pose_search_dist_ =
    max_robot_pose_search_dist_ > 0.0 ? max_robot_pose_search_dist_ : getCostmapMaxExtent();
  last_scaled_linear_vel_ = v_linear_max_;
  escape_start_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  stuck_start_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  resetPidControllers();

  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  curvature_points_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "curvature_points_marker_array", rclcpp::QoS(10));
  stuck_pub_ = node->create_publisher<std_msgs::msg::Bool>("stuck_status", 1);
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&OmniPidPursuitController::odomCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_, "Configured generic omni PID pursuit controller, odom_topic=%s",
    odom_topic_.c_str());
}

void OmniPidPursuitController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up controller plugin %s", plugin_name_.c_str());
  local_path_pub_.reset();
  carrot_pub_.reset();
  curvature_points_pub_.reset();
  stuck_pub_.reset();
  odom_sub_.reset();
}

void OmniPidPursuitController::activate()
{
  RCLCPP_INFO(logger_, "Activating controller plugin %s", plugin_name_.c_str());
  local_path_pub_->on_activate();
  carrot_pub_->on_activate();
  curvature_points_pub_->on_activate();
  stuck_pub_->on_activate();

  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&OmniPidPursuitController::dynamicParametersCallback, this, std::placeholders::_1));
}

void OmniPidPursuitController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating controller plugin %s", plugin_name_.c_str());
  local_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  curvature_points_pub_->on_deactivate();
  stuck_pub_->on_deactivate();
  dyn_params_handler_.reset();
}

geometry_msgs::msg::TwistStamped OmniPidPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  const auto now = clock_->now();

  const auto transformed_plan = transformGlobalPlan(pose);
  const double lookahead_dist = getLookAheadDistance(velocity);
  const auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  const double lin_dist =
    hypot2d(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  const double theta_dist =
    std::atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);

  double lin_vel = move_pid_->calculate(lin_dist, 0.0, limit_i_v_);
  applyApproachVelocityScaling(pose, lin_vel);
  if (enable_curvature_speed_limit_) {
    applyCurvatureLimitation(transformed_plan, carrot_pose, lin_vel);
  }

  double rotation_target = theta_dist;
  if (use_rotate_to_heading_ && std::abs(theta_dist) < rotate_to_heading_threshold_) {
    rotation_target = tf2::getYaw(carrot_pose.pose.orientation);
  }
  const double angular_vel =
    enable_rotation_ ? heading_pid_->calculate(rotation_target, 0.0, limit_i_w_) : 0.0;

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = now;
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd_vel.twist.linear.x = lin_vel * std::cos(theta_dist);
  cmd_vel.twist.linear.y = lin_vel * std::sin(theta_dist);
  cmd_vel.twist.linear.z = 0.0;
  cmd_vel.twist.angular.x = 0.0;
  cmd_vel.twist.angular.y = 0.0;
  cmd_vel.twist.angular.z = angular_vel;

  const double command_linear_speed =
    hypot2d(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y);
  double actual_linear_speed = 0.0;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    actual_linear_speed = current_linear_speed_;
  }

  if (!enable_stuck_escape_) {
    publishStuckStatus(false);
    return cmd_vel;
  }

  if (in_escape_phase_) {
    const double elapsed = (now - escape_start_time_).seconds();
    if (elapsed < escape_duration_) {
      geometry_msgs::msg::TwistStamped escape_cmd;
      escape_cmd.header = cmd_vel.header;
      escape_cmd.twist.linear.x = -std::abs(escape_linear_speed_);
      escape_cmd.twist.linear.y = 0.0;
      escape_cmd.twist.angular.z = 0.0;
      publishStuckStatus(true);
      return escape_cmd;
    }

    in_escape_phase_ = false;
    ++escape_count_;
    RCLCPP_WARN(
      logger_, "Stuck escape attempt %d/%d finished", escape_count_, max_escape_attempts_);
  }

  const bool stuck_signal =
    command_linear_speed > max_command_speed_threshold_ &&
    actual_linear_speed < min_actual_speed_threshold_;

  if (!stuck_signal) {
    is_stuck_ = false;
    escape_count_ = 0;
    publishStuckStatus(false);
    return cmd_vel;
  }

  publishStuckStatus(true);
  if (!is_stuck_) {
    stuck_start_time_ = now;
    is_stuck_ = true;
    return cmd_vel;
  }

  const double stuck_duration = (now - stuck_start_time_).seconds();
  if (stuck_duration < stuck_duration_threshold_) {
    return cmd_vel;
  }

  if (escape_count_ >= max_escape_attempts_) {
    RCLCPP_WARN(
      logger_,
      "Robot is still stuck after %d escape attempts. Nav2 recovery should take over if enabled.",
      max_escape_attempts_);
    if (fail_after_escape_attempts_) {
      throw nav2_core::PlannerException("Omni controller escape attempts exhausted");
    }
    return cmd_vel;
  }

  in_escape_phase_ = true;
  escape_start_time_ = now;
  is_stuck_ = false;

  geometry_msgs::msg::TwistStamped escape_cmd;
  escape_cmd.header = cmd_vel.header;
  escape_cmd.twist.linear.x = -std::abs(escape_linear_speed_);
  escape_cmd.twist.linear.y = 0.0;
  escape_cmd.twist.angular.z = 0.0;
  RCLCPP_WARN(logger_, "Robot appears stuck, backing up for %.2f s", escape_duration_);
  return escape_cmd;
}

void OmniPidPursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void OmniPidPursuitController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (percentage) {
    const double ratio = clampValue(speed_limit / 100.0, 0.0, 1.0);
    v_linear_max_ = nominal_v_linear_max_ * ratio;
    v_linear_min_ = nominal_v_linear_min_ * ratio;
  } else {
    const double limit = std::abs(speed_limit);
    v_linear_max_ = limit;
    v_linear_min_ = -limit;
  }

  resetPidControllers();
}

nav_msgs::msg::Path OmniPidPursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into plan frame");
  }

  const double max_costmap_extent = getCostmapMaxExtent();
  auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);
  auto transformation_begin = nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & plan_pose) {
      return euclidean_distance(plan_pose, robot_pose) > max_costmap_extent;
    });

  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  auto transform_global_pose_to_local = [&](const auto & global_plan_pose) {
    geometry_msgs::msg::PoseStamped stamped_pose;
    geometry_msgs::msg::PoseStamped transformed_pose;
    stamped_pose.header.frame_id = global_plan_.header.frame_id;
    stamped_pose.header.stamp = robot_pose.header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    if (!transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose)) {
      throw nav2_core::PlannerException("Unable to transform plan pose into local frame");
    }
    transformed_pose.pose.position.z = 0.0;
    return transformed_pose;
  };

  std::transform(
    transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
    transform_global_pose_to_local);

  if (transformed_plan.poses.empty()) {
    transformed_plan.poses.push_back(transform_global_pose_to_local(*transformation_begin));
  }

  global_plan_.poses.erase(global_plan_.poses.begin(), transformation_begin);
  local_path_pub_->publish(transformed_plan);

  return transformed_plan;
}

bool OmniPidPursuitController::transformPose(
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double OmniPidPursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters =
    std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

std::unique_ptr<geometry_msgs::msg::PointStamped> OmniPidPursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;
  return carrot_msg;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::getLookAheadPoint(
  double lookahead_dist, const nav_msgs::msg::Path & transformed_plan)
{
  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Transformed plan has zero poses");
  }

  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot2d(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  if (goal_pose_it == transformed_plan.poses.end()) {
    return transformed_plan.poses.back();
  }

  if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    pose.pose.orientation = goal_pose_it->pose.orientation;
    return pose;
  }

  return *goal_pose_it;
}

geometry_msgs::msg::Point OmniPidPursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r) const
{
  const double dx = p2.x - p1.x;
  const double dy = p2.y - p1.y;
  const double dr2 = dx * dx + dy * dy;
  if (dr2 <= std::numeric_limits<double>::epsilon()) {
    return p2;
  }

  const double d = p1.x * p2.y - p2.x * p1.y;
  const double discriminant = std::max(0.0, r * r * dr2 - d * d);
  const double dd = (p2.x * p2.x + p2.y * p2.y) - (p1.x * p1.x + p1.y * p1.y);
  const double sqrt_term = std::sqrt(discriminant);

  geometry_msgs::msg::Point p;
  p.x = (d * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-d * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  p.z = 0.0;
  return p;
}

double OmniPidPursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed) const
{
  double lookahead_dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = hypot2d(speed.linear.x, speed.linear.y) * lookahead_time_;
    lookahead_dist = clampValue(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }
  return lookahead_dist;
}

double OmniPidPursuitController::calculateDistanceToGoal(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  if (global_plan_.poses.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto & goal_pose = global_plan_.poses.back();
  return hypot2d(
    goal_pose.pose.position.x - robot_pose.pose.position.x,
    goal_pose.pose.position.y - robot_pose.pose.position.y);
}

double OmniPidPursuitController::approachVelocityScalingFactor(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  const double remaining_distance = calculateDistanceToGoal(robot_pose);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    const double min_scale =
      min_approach_linear_velocity_ / std::max(std::abs(v_linear_max_), 1e-6);
    return std::max(remaining_distance / approach_velocity_scaling_dist_, min_scale);
  }
  return 1.0;
}

void OmniPidPursuitController::applyApproachVelocityScaling(
  const geometry_msgs::msg::PoseStamped & robot_pose, double & linear_vel) const
{
  const double scaling = approachVelocityScalingFactor(robot_pose);
  const double scaled = linear_vel * scaling;
  if (std::abs(scaled) < min_approach_linear_velocity_) {
    linear_vel = std::copysign(min_approach_linear_velocity_, linear_vel);
    return;
  }
  linear_vel = std::abs(scaled) < std::abs(linear_vel) ? scaled : linear_vel;
}

void OmniPidPursuitController::applyCurvatureLimitation(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double & linear_vel)
{
  if (path.poses.size() < 3 || std::abs(linear_vel) < min_approach_linear_velocity_) {
    return;
  }

  const double curvature =
    calculateCurvature(path, lookahead_pose, curvature_forward_dist_, curvature_backward_dist_);
  if (!std::isfinite(curvature) || curvature <= curvature_min_) {
    last_scaled_linear_vel_ = linear_vel;
    return;
  }

  double reduction_ratio = reduction_ratio_at_high_curvature_;
  if (curvature < curvature_max_) {
    const double t = (curvature - curvature_min_) / (curvature_max_ - curvature_min_);
    reduction_ratio = 1.0 - t * (1.0 - reduction_ratio_at_high_curvature_);
  }

  const double target_scaled_vel = linear_vel * reduction_ratio;
  if (last_scaled_linear_vel_ == 0.0) {
    last_scaled_linear_vel_ = linear_vel;
  }
  const double max_delta = max_velocity_scaling_factor_rate_ * control_duration_;
  const double delta = clampValue(
    target_scaled_vel - last_scaled_linear_vel_, -max_delta, max_delta);
  const double scaled_linear_vel = last_scaled_linear_vel_ + delta;

  linear_vel =
    std::copysign(std::max(std::abs(scaled_linear_vel), min_approach_linear_velocity_), linear_vel);
  last_scaled_linear_vel_ = linear_vel;
}

double OmniPidPursuitController::calculateCurvature(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double forward_dist, double backward_dist) const
{
  const auto cumulative_distances = calculateCumulativeDistances(path);
  geometry_msgs::msg::PoseStamped robot_base_frame_pose;
  robot_base_frame_pose.pose = geometry_msgs::msg::Pose();
  const double lookahead_distance =
    nav2_util::geometry_utils::euclidean_distance(robot_base_frame_pose, lookahead_pose);

  const auto backward_pose =
    findPoseAtDistance(path, cumulative_distances, lookahead_distance - backward_dist);
  const auto forward_pose =
    findPoseAtDistance(path, cumulative_distances, lookahead_distance + forward_dist);
  const double radius = calculateCurvatureRadius(
    backward_pose.pose.position, lookahead_pose.pose.position, forward_pose.pose.position);

  visualizeCurvaturePoints(backward_pose, forward_pose);
  if (radius <= 1e-6) {
    return 0.0;
  }
  return 1.0 / radius;
}

double OmniPidPursuitController::calculateCurvatureRadius(
  const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
  const geometry_msgs::msg::Point & far_point) const
{
  const double x1 = near_point.x;
  const double y1 = near_point.y;
  const double x2 = current_point.x;
  const double y2 = current_point.y;
  const double x3 = far_point.x;
  const double y3 = far_point.y;
  const double denominator = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
  if (std::abs(denominator) < 1e-9) {
    return 1e9;
  }

  const double center_x =
    ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
    (x3 * x3 + y3 * y3) * (y1 - y2)) / denominator;
  const double center_y =
    ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
    (x3 * x3 + y3 * y3) * (x2 - x1)) / denominator;

  const double radius = hypot2d(x2 - center_x, y2 - center_y);
  if (!std::isfinite(radius) || radius < 1e-9) {
    return 1e9;
  }
  return radius;
}

void OmniPidPursuitController::visualizeCurvaturePoints(
  const geometry_msgs::msg::PoseStamped & backward_pose,
  const geometry_msgs::msg::PoseStamped & forward_pose) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker near_marker;
  near_marker.header = backward_pose.header;
  near_marker.ns = "curvature_points";
  near_marker.id = 0;
  near_marker.type = visualization_msgs::msg::Marker::SPHERE;
  near_marker.action = visualization_msgs::msg::Marker::ADD;
  near_marker.pose = backward_pose.pose;
  near_marker.scale.x = 0.1;
  near_marker.scale.y = 0.1;
  near_marker.scale.z = 0.1;
  near_marker.color.g = 1.0;
  near_marker.color.a = 1.0;

  visualization_msgs::msg::Marker far_marker;
  far_marker.header = forward_pose.header;
  far_marker.ns = "curvature_points";
  far_marker.id = 1;
  far_marker.type = visualization_msgs::msg::Marker::SPHERE;
  far_marker.action = visualization_msgs::msg::Marker::ADD;
  far_marker.pose = forward_pose.pose;
  far_marker.scale.x = 0.1;
  far_marker.scale.y = 0.1;
  far_marker.scale.z = 0.1;
  far_marker.color.r = 1.0;
  far_marker.color.a = 1.0;

  marker_array.markers.push_back(near_marker);
  marker_array.markers.push_back(far_marker);
  curvature_points_pub_->publish(marker_array);
}

std::vector<double> OmniPidPursuitController::calculateCumulativeDistances(
  const nav_msgs::msg::Path & path) const
{
  std::vector<double> cumulative_distances;
  cumulative_distances.reserve(path.poses.size());
  cumulative_distances.push_back(0.0);

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & prev = path.poses[i - 1].pose.position;
    const auto & curr = path.poses[i].pose.position;
    cumulative_distances.push_back(
      cumulative_distances.back() + hypot2d(curr.x - prev.x, curr.y - prev.y));
  }

  return cumulative_distances;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::findPoseAtDistance(
  const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
  double target_distance) const
{
  if (path.poses.empty() || cumulative_distances.empty()) {
    return geometry_msgs::msg::PoseStamped();
  }
  if (target_distance <= 0.0) {
    return path.poses.front();
  }
  if (target_distance >= cumulative_distances.back()) {
    return path.poses.back();
  }

  auto it =
    std::lower_bound(cumulative_distances.begin(), cumulative_distances.end(), target_distance);
  const size_t index = std::distance(cumulative_distances.begin(), it);
  if (index == 0) {
    return path.poses.front();
  }

  const double denom = cumulative_distances[index] - cumulative_distances[index - 1];
  const double ratio =
    denom > 1e-9 ? (target_distance - cumulative_distances[index - 1]) / denom : 0.0;
  const auto & pose1 = path.poses[index - 1];
  const auto & pose2 = path.poses[index];

  geometry_msgs::msg::PoseStamped interpolated_pose;
  interpolated_pose.header = pose2.header;
  interpolated_pose.pose.position.x =
    pose1.pose.position.x + ratio * (pose2.pose.position.x - pose1.pose.position.x);
  interpolated_pose.pose.position.y =
    pose1.pose.position.y + ratio * (pose2.pose.position.y - pose1.pose.position.y);
  interpolated_pose.pose.position.z =
    pose1.pose.position.z + ratio * (pose2.pose.position.z - pose1.pose.position.z);
  interpolated_pose.pose.orientation = pose2.pose.orientation;
  return interpolated_pose;
}

void OmniPidPursuitController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  current_linear_speed_ = hypot2d(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  current_angular_speed_ = msg->twist.twist.angular.z;
}

void OmniPidPursuitController::publishStuckStatus(bool stuck)
{
  if (!publish_stuck_status_ || !stuck_pub_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = stuck;
  stuck_pub_->publish(msg);
}

void OmniPidPursuitController::resetPidControllers()
{
  move_pid_ = std::make_shared<PID>(
    control_duration_, v_linear_max_, v_linear_min_, translation_kp_, translation_kd_,
    translation_ki_);
  heading_pid_ = std::make_shared<PID>(
    control_duration_, v_angular_max_, v_angular_min_, rotation_kp_, rotation_kd_, rotation_ki_);
}

rcl_interfaces::msg::SetParametersResult OmniPidPursuitController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  bool should_reset_pid = false;

  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".translation_kp") {
        translation_kp_ = parameter.as_double();
        should_reset_pid = true;
      } else if (name == plugin_name_ + ".translation_ki") {
        translation_ki_ = parameter.as_double();
        should_reset_pid = true;
      } else if (name == plugin_name_ + ".translation_kd") {
        translation_kd_ = parameter.as_double();
        should_reset_pid = true;
      } else if (name == plugin_name_ + ".rotation_kp") {
        rotation_kp_ = parameter.as_double();
        should_reset_pid = true;
      } else if (name == plugin_name_ + ".rotation_ki") {
        rotation_ki_ = parameter.as_double();
        should_reset_pid = true;
      } else if (name == plugin_name_ + ".rotation_kd") {
        rotation_kd_ = parameter.as_double();
        should_reset_pid = true;
      } else if (name == plugin_name_ + ".limit_i_v") {
        limit_i_v_ = parameter.as_double();
      } else if (name == plugin_name_ + ".limit_i_w") {
        limit_i_w_ = parameter.as_double();
      } else if (name == plugin_name_ + ".transform_tolerance") {
        transform_tolerance_ = tf2::durationFromSec(parameter.as_double());
      } else if (name == plugin_name_ + ".lookahead_dist") {
        lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_lookahead_dist") {
        min_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        max_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        lookahead_time_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotate_to_heading_threshold") {
        rotate_to_heading_threshold_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
        min_approach_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".approach_velocity_scaling_dist") {
        approach_velocity_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_max") {
        v_linear_max_ = parameter.as_double();
        nominal_v_linear_max_ = v_linear_max_;
        should_reset_pid = true;
      } else if (name == plugin_name_ + ".v_linear_min") {
        v_linear_min_ = parameter.as_double();
        nominal_v_linear_min_ = v_linear_min_;
        should_reset_pid = true;
      } else if (name == plugin_name_ + ".v_angular_max") {
        v_angular_max_ = parameter.as_double();
        should_reset_pid = true;
      } else if (name == plugin_name_ + ".v_angular_min") {
        v_angular_min_ = parameter.as_double();
        should_reset_pid = true;
      } else if (name == plugin_name_ + ".curvature_min") {
        curvature_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_max") {
        curvature_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".reduction_ratio_at_high_curvature") {
        reduction_ratio_at_high_curvature_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_forward_dist") {
        curvature_forward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_backward_dist") {
        curvature_backward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_velocity_scaling_factor_rate") {
        max_velocity_scaling_factor_rate_ = parameter.as_double();
      } else if (name == plugin_name_ + ".escape_duration") {
        escape_duration_ = parameter.as_double();
      } else if (name == plugin_name_ + ".escape_linear_speed") {
        escape_linear_speed_ = parameter.as_double();
      } else if (name == plugin_name_ + ".stuck_duration_threshold") {
        stuck_duration_threshold_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_actual_speed_threshold") {
        min_actual_speed_threshold_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_command_speed_threshold") {
        max_command_speed_threshold_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == plugin_name_ + ".max_escape_attempts") {
        max_escape_attempts_ = parameter.as_int();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_interpolation") {
        use_interpolation_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".enable_rotation") {
        enable_rotation_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_rotate_to_heading") {
        use_rotate_to_heading_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".enable_curvature_speed_limit") {
        enable_curvature_speed_limit_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".enable_stuck_escape") {
        enable_stuck_escape_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".publish_stuck_status") {
        publish_stuck_status_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".fail_after_escape_attempts") {
        fail_after_escape_attempts_ = parameter.as_bool();
      }
    }
  }

  if (should_reset_pid) {
    resetPidControllers();
  }

  result.successful = true;
  return result;
}

}  // namespace pb_omni_pid_pursuit_controller

PLUGINLIB_EXPORT_CLASS(
  pb_omni_pid_pursuit_controller::OmniPidPursuitController, nav2_core::Controller)
