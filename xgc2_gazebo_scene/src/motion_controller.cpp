#include "xgc2_gazebo_scene/motion_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace xgc2_gazebo_scene {
namespace {

bool FiniteVector(const ignition::math::Vector3d& value) {
    return std::isfinite(value.X()) && std::isfinite(value.Y()) && std::isfinite(value.Z());
}

} // namespace

bool ParseMotionMode(const std::string& value, MotionMode* mode) {
    if (value == "hold") {
        *mode = MotionMode::kHold;
        return true;
    }
    if (value == "constant_twist") {
        *mode = MotionMode::kConstantTwist;
        return true;
    }
    if (value == "ping_pong") {
        *mode = MotionMode::kPingPong;
        return true;
    }
    return false;
}

std::string MotionModeName(MotionMode mode) {
    switch (mode) {
    case MotionMode::kHold:
        return "hold";
    case MotionMode::kConstantTwist:
        return "constant_twist";
    case MotionMode::kPingPong:
        return "ping_pong";
    }
    return "hold";
}

bool MotionController::Configure(const MotionConfiguration& configuration, const ignition::math::Pose3d& observed_pose,
                                 double simulation_time, std::string* error) {
    if (!std::isfinite(simulation_time) || !FiniteVector(observed_pose.Pos())) {
        *error = "motion origin and simulation time must be finite";
        return false;
    }
    if (!FiniteVector(configuration.linear_velocity) || !FiniteVector(configuration.angular_velocity)) {
        *error = "motion velocity must be finite";
        return false;
    }
    if (configuration.mode == MotionMode::kPingPong) {
        if (configuration.waypoints.size() != 2) {
            *error = "ping_pong requires exactly two waypoints";
            return false;
        }
        if (!FiniteVector(configuration.waypoints[0]) || !FiniteVector(configuration.waypoints[1]) ||
            !std::isfinite(configuration.speed) || configuration.speed <= 0.0) {
            *error = "ping_pong waypoints must be finite and speed must be positive";
            return false;
        }
        if ((configuration.waypoints[1] - configuration.waypoints[0]).Length() <=
            std::numeric_limits<double>::epsilon()) {
            *error = "ping_pong waypoints must be distinct";
            return false;
        }
    }

    configuration_ = configuration;
    origin_pose_ = observed_pose;
    start_time_ = simulation_time;
    error->clear();
    return true;
}

MotionSample MotionController::Sample(double simulation_time) const {
    MotionSample sample;
    sample.pose = origin_pose_;
    const double elapsed = std::max(0.0, simulation_time - start_time_);

    if (configuration_.mode == MotionMode::kHold) {
        return sample;
    }
    if (configuration_.mode == MotionMode::kConstantTwist) {
        sample.pose.Pos() = origin_pose_.Pos() + configuration_.linear_velocity * elapsed;
        const double angular_speed = configuration_.angular_velocity.Length();
        if (angular_speed > std::numeric_limits<double>::epsilon()) {
            const ignition::math::Vector3d axis = configuration_.angular_velocity / angular_speed;
            const ignition::math::Quaterniond delta(axis, angular_speed * elapsed);
            sample.pose.Rot() = delta * origin_pose_.Rot();
            sample.pose.Rot().Normalize();
        }
        sample.linear_velocity = configuration_.linear_velocity;
        sample.angular_velocity = configuration_.angular_velocity;
        return sample;
    }

    const ignition::math::Vector3d start = configuration_.waypoints[0];
    const ignition::math::Vector3d finish = configuration_.waypoints[1];
    const ignition::math::Vector3d displacement = finish - start;
    const double length = displacement.Length();
    const ignition::math::Vector3d direction = displacement / length;
    const double one_way_time = length / configuration_.speed;
    const double phase = std::fmod(elapsed, 2.0 * one_way_time);
    if (phase <= one_way_time) {
        sample.pose.Pos() = start + direction * (configuration_.speed * phase);
        sample.linear_velocity = direction * configuration_.speed;
    } else {
        const double return_time = phase - one_way_time;
        sample.pose.Pos() = finish - direction * (configuration_.speed * return_time);
        sample.linear_velocity = direction * -configuration_.speed;
    }
    return sample;
}

std::string MotionController::modeName() const {
    return MotionModeName(configuration_.mode);
}

} // namespace xgc2_gazebo_scene
