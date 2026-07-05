#include <algorithm>
#include <cmath>

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

    const auto delta = end_ - start_;
    travel_distance_ = delta.Length();
    travel_time_ = std::max(0.01, travel_distance_ / speed_);
    forward_yaw_ = std::atan2(delta.Y(), delta.X());

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&DynamicObstaclePlugin::OnUpdate, this, std::placeholders::_1));
  }

private:
  static double GetDouble(const sdf::ElementPtr & sdf, const std::string & name, double fallback)
  {
    if (!sdf || !sdf->HasElement(name)) {
      return fallback;
    }
    return sdf->Get<double>(name);
  }

  void OnUpdate(const common::UpdateInfo & info)
  {
    if (!model_ || travel_distance_ < 1e-6) {
      return;
    }

    const double now = info.simTime.Double();
    if (!started_) {
      start_time_ = now;
      started_ = true;
    }

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

    ignition::math::Pose3d pose(position, ignition::math::Quaterniond(0.0, 0.0, yaw));
    model_->SetWorldPose(pose);
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
  }

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;
  ignition::math::Pose3d start_pose_;
  ignition::math::Vector3d start_;
  ignition::math::Vector3d end_;
  double speed_{0.35};
  double pause_time_{0.6};
  double travel_distance_{0.0};
  double travel_time_{1.0};
  double forward_yaw_{0.0};
  double start_time_{0.0};
  bool started_{false};
};

GZ_REGISTER_MODEL_PLUGIN(DynamicObstaclePlugin)
}  // namespace gazebo
