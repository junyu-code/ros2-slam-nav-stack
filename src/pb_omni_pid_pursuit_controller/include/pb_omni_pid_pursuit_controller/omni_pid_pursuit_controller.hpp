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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pb_omni_pid_pursuit_controller/pid.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace pb_omni_pid_pursuit_controller
{

class OmniPidPursuitController : public nav2_core::Controller
{
public:
  OmniPidPursuitController() = default;
  ~OmniPidPursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  bool transformPose(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  double getCostmapMaxExtent() const;

  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    double lookahead_dist, const nav_msgs::msg::Path & transformed_plan);

  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r) const;

  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed) const;
  double calculateDistanceToGoal(const geometry_msgs::msg::PoseStamped & robot_pose) const;
  double approachVelocityScalingFactor(const geometry_msgs::msg::PoseStamped & robot_pose) const;
  void applyApproachVelocityScaling(
    const geometry_msgs::msg::PoseStamped & robot_pose, double & linear_vel) const;
  void applyCurvatureLimitation(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double & linear_vel);

  double calculateCurvature(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double forward_dist, double backward_dist) const;
  double calculateCurvatureRadius(
    const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
    const geometry_msgs::msg::Point & far_point) const;
  void visualizeCurvaturePoints(
    const geometry_msgs::msg::PoseStamped & backward_pose,
    const geometry_msgs::msg::PoseStamped & forward_pose) const;
  std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path & path) const;
  geometry_msgs::msg::PoseStamped findPoseAtDistance(
    const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
    double target_distance) const;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publishStuckStatus(bool stuck);
  void resetPidControllers();

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("OmniPidPursuitController")};
  rclcpp::Clock::SharedPtr clock_;
  tf2::Duration transform_tolerance_;

  std::shared_ptr<PID> move_pid_;
  std::shared_ptr<PID> heading_pid_;

  nav_msgs::msg::Path global_plan_;
  std::mutex mutex_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    curvature_points_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr stuck_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  std::string odom_topic_{"/Odometry"};
  mutable std::mutex odom_mutex_;
  double current_linear_speed_{0.0};
  double current_angular_speed_{0.0};

  double translation_kp_{3.0};
  double translation_ki_{0.1};
  double translation_kd_{0.3};
  double rotation_kp_{3.0};
  double rotation_ki_{0.1};
  double rotation_kd_{0.3};
  double limit_i_v_{0.9};
  double limit_i_w_{0.9};

  double control_duration_{0.05};
  double max_robot_pose_search_dist_{0.0};
  double lookahead_dist_{0.35};
  bool use_velocity_scaled_lookahead_dist_{true};
  double min_lookahead_dist_{0.25};
  double max_lookahead_dist_{1.0};
  double lookahead_time_{0.8};
  bool use_interpolation_{true};
  bool enable_rotation_{true};
  bool use_rotate_to_heading_{false};
  double rotate_to_heading_threshold_{0.25};

  double min_approach_linear_velocity_{0.04};
  double approach_velocity_scaling_dist_{0.7};
  double v_linear_min_{-0.6};
  double v_linear_max_{0.6};
  double nominal_v_linear_min_{-0.6};
  double nominal_v_linear_max_{0.6};
  double v_angular_min_{-1.2};
  double v_angular_max_{1.2};

  bool enable_curvature_speed_limit_{true};
  double curvature_min_{0.4};
  double curvature_max_{0.9};
  double reduction_ratio_at_high_curvature_{0.55};
  double curvature_forward_dist_{0.7};
  double curvature_backward_dist_{0.3};
  double max_velocity_scaling_factor_rate_{0.8};
  double last_scaled_linear_vel_{0.0};

  bool enable_stuck_escape_{true};
  bool publish_stuck_status_{true};
  bool fail_after_escape_attempts_{false};
  int max_escape_attempts_{2};
  int escape_count_{0};
  bool in_escape_phase_{false};
  rclcpp::Time escape_start_time_;
  double escape_duration_{0.8};
  double escape_linear_speed_{0.25};
  rclcpp::Time stuck_start_time_;
  bool is_stuck_{false};
  double stuck_duration_threshold_{1.5};
  double min_actual_speed_threshold_{0.04};
  double max_command_speed_threshold_{0.18};
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_
