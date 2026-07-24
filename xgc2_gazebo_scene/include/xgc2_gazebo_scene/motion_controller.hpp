#ifndef XGC2_GAZEBO_SCENE_MOTION_CONTROLLER_HPP_
#define XGC2_GAZEBO_SCENE_MOTION_CONTROLLER_HPP_

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <cstdint>
#include <string>
#include <vector>

namespace xgc2_gazebo_scene {

enum class MotionMode { kHold, kConstantTwist, kPingPong, kCircle };

struct MotionConfiguration {
    MotionMode mode = MotionMode::kHold;
    ignition::math::Vector3d linear_velocity = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d angular_velocity = ignition::math::Vector3d::Zero;
    std::vector<ignition::math::Vector3d> waypoints;
    double speed = 0.0;
};

struct MotionSample {
    ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;
    ignition::math::Vector3d linear_velocity = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d angular_velocity = ignition::math::Vector3d::Zero;
};

class MotionController {
  public:
    MotionController() = default;

    bool Configure(const MotionConfiguration& configuration, const ignition::math::Pose3d& observed_pose,
                   double simulation_time, std::string* error);
    MotionSample Sample(double simulation_time) const;

    MotionMode mode() const { return configuration_.mode; }
    std::string modeName() const;

  private:
    MotionConfiguration configuration_;
    ignition::math::Pose3d origin_pose_ = ignition::math::Pose3d::Zero;
    double start_time_ = 0.0;
};

bool ParseMotionMode(const std::string& value, MotionMode* mode);
std::string MotionModeName(MotionMode mode);

} // namespace xgc2_gazebo_scene

#endif // XGC2_GAZEBO_SCENE_MOTION_CONTROLLER_HPP_
