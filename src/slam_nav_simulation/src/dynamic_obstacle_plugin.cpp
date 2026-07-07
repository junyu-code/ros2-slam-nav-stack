#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

#include <gazebo/common/Console.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class DynamicObstaclePlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    start_pose_ = model_->WorldPose();

    start_.X() = GetDouble(sdf, "start_x", start_pose_.Pos().X());
    start_.Y() = GetDouble(sdf, "start_y", start_pose_.Pos().Y());
    start_.Z() = GetDouble(sdf, "height", start_pose_.Pos().Z());

    end_.X() = GetDouble(sdf, "end_x", start_.X());
    end_.Y() = GetDouble(sdf, "end_y", start_.Y());
    end_.Z() = start_.Z();

    speed_ = std::max(0.01, GetDouble(sdf, "speed", 0.35));
    pause_time_ = std::max(0.0, GetDouble(sdf, "pause_time", 0.6));
    robot_model_name_ = GetString(sdf, "robot_model", "mobile_robot");
    yield_radius_ = std::max(0.0, GetDouble(sdf, "yield_radius", 1.2));
    yield_resume_radius_ = std::max(yield_radius_, GetDouble(sdf, "yield_resume_radius", yield_radius_ + 0.35));

    const auto delta = end_ - start_;
    travel_distance_ = delta.Length();
    travel_time_ = std::max(0.01, travel_distance_ / speed_);
    forward_yaw_ = std::atan2(delta.Y(), delta.X());

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&DynamicObstaclePlugin::OnUpdate, this, std::placeholders::_1));

    gzmsg << "[DynamicObstaclePlugin] model=" << model_->GetName()
          << ", robot_model=" << robot_model_name_
          << ", speed=" << speed_
          << ", yield_radius=" << yield_radius_
          << ", yield_resume_radius=" << yield_resume_radius_ << "\n";
  }

private:
  static double GetDouble(const sdf::ElementPtr & sdf, const std::string & name, double fallback)
  {
    if (!sdf || !sdf->HasElement(name)) {
      return fallback;
    }
    return sdf->Get<double>(name);
  }

  static std::string GetString(
    const sdf::ElementPtr & sdf,
    const std::string & name,
    const std::string & fallback)
  {
    if (!sdf || !sdf->HasElement(name)) {
      return fallback;
    }
    return sdf->Get<std::string>(name);
  }

  void OnUpdate(const common::UpdateInfo & info)
  {
    if (!model_ || travel_distance_ < 1e-6) {
      return;
    }

    const double now = info.simTime.Double();
    if (!started_) {
      start_time_ = now;
      last_update_time_ = now;
      started_ = true;
    }

    const double dt = std::max(0.0, now - last_update_time_);
    last_update_time_ = now;

    // 动态障碍物是运动学物体，直接撞车会把定位撞偏；用下一帧候选位置提前让行。
    const double cycle_time = 2.0 * travel_time_ + 2.0 * pause_time_;
    double t = std::fmod(now - start_time_, cycle_time);
    if (t < 0.0) {
      t += cycle_time;
    }

    bool forward = true;
    double ratio = 0.0;

    if (t < travel_time_) {
      ratio = t / travel_time_;
      forward = true;
    } else if (t < travel_time_ + pause_time_) {
      ratio = 1.0;
      forward = true;
    } else if (t < 2.0 * travel_time_ + pause_time_) {
      ratio = 1.0 - ((t - travel_time_ - pause_time_) / travel_time_);
      forward = false;
    } else {
      ratio = 0.0;
      forward = false;
    }

    ratio = std::clamp(ratio, 0.0, 1.0);
    const auto position = start_ + (end_ - start_) * ratio;
    const double yaw = forward ? forward_yaw_ : forward_yaw_ + M_PI;

    if (shouldYieldToRobot(position)) {
      start_time_ += dt;
      model_->SetLinearVel(ignition::math::Vector3d::Zero);
      model_->SetAngularVel(ignition::math::Vector3d::Zero);
      return;
    }

    ignition::math::Pose3d pose(position, ignition::math::Quaterniond(0.0, 0.0, yaw));
    model_->SetWorldPose(pose);
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
  }

  bool shouldYieldToRobot(const ignition::math::Vector3d & candidate_pos)
  {
    if (yield_radius_ <= 0.0 || !model_) {
      yielding_ = false;
      return false;
    }

    const auto world = model_->GetWorld();
    if (!world) {
      yielding_ = false;
      return false;
    }

    const auto robot = world->ModelByName(robot_model_name_);
    if (!robot) {
      yielding_ = false;
      return false;
    }

    const auto robot_pos = robot->WorldPose().Pos();
    const double distance_xy = std::hypot(
      candidate_pos.X() - robot_pos.X(),
      candidate_pos.Y() - robot_pos.Y());

    if (yielding_) {
      yielding_ = distance_xy < yield_resume_radius_;
    } else {
      yielding_ = distance_xy < yield_radius_;
    }
    return yielding_;
  }

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;
  ignition::math::Pose3d start_pose_;
  ignition::math::Vector3d start_;
  ignition::math::Vector3d end_;
  double speed_{0.35};
  double pause_time_{0.6};
  double yield_radius_{0.0};
  double yield_resume_radius_{0.0};
  double travel_distance_{0.0};
  double travel_time_{1.0};
  double forward_yaw_{0.0};
  double start_time_{0.0};
  double last_update_time_{0.0};
  std::string robot_model_name_{"mobile_robot"};
  bool started_{false};
  bool yielding_{false};
};

GZ_REGISTER_MODEL_PLUGIN(DynamicObstaclePlugin)
}  // namespace gazebo
