// Stand-ins for the managed obstacles of GazeboSceneSystemPlugin and random
// scenes made of them, for obstacle_messages_test. Not part of the package interface.
#pragma once

#include "xgc2_gazebo_scene/motion_controller.hpp"
#include "xgc2_gazebo_scene/obstacle_messages.hpp"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace xgc2_gazebo_scene {
namespace fixture {

/// The few members of a Gazebo model the 30 Hz publication reads.
struct Model {
    ignition::math::Vector3d linear_velocity;
    ignition::math::Vector3d angular_velocity;
    bool is_static = true;
    ignition::math::Vector3d WorldLinearVel() const { return linear_velocity; }
    ignition::math::Vector3d WorldAngularVel() const { return angular_velocity; }
    bool IsStatic() const { return is_static; }
};

/// GazeboSceneSystemPlugin::ManagedObstacle without the Gazebo.
struct Obstacle {
    std::shared_ptr<Model> model;
    std::vector<xgc2_geometry_msgs::ConvexBodyInstance> parts;
    ignition::math::Pose3d observed_pose = ignition::math::Pose3d::Zero;
    MotionController controller;
    bool controlled = false;
};

/// Reads an obstacle the way the 30 Hz publication reports it: the controller's
/// sample while a motion drives it, Gazebo's velocities while it does not.
/// `Obstacle` has `model` (WorldLinearVel, WorldAngularVel, IsStatic),
/// `observed_pose`, `controlled`, `controller` (Sample, modeName) and
/// the collision parts.
template <class Obstacle>
void SampleObstacle(const Obstacle& obstacle, double simulation_time, ObstacleDynamics* dynamics) {
    dynamics->pose = obstacle.observed_pose;
    if (obstacle.controlled) {
        const MotionSample sample = obstacle.controller.Sample(simulation_time);
        dynamics->linear_velocity = sample.linear_velocity;
        dynamics->angular_velocity = sample.angular_velocity;
    } else {
        dynamics->linear_velocity = obstacle.model->WorldLinearVel();
        dynamics->angular_velocity = obstacle.model->WorldAngularVel();
    }
    dynamics->is_static =
        obstacle.model->IsStatic() && (!obstacle.controlled || obstacle.controller.mode() == MotionMode::kHold);
}

using Obstacles = std::map<std::string, Obstacle>;

class Scene {
  public:
    explicit Scene(unsigned seed) : random_(seed) {}

    const Obstacles& obstacles() const { return obstacles_; }

    /// Adds `count` obstacles with `parts` parts each (0: one to four parts, mixed shapes).
    void Add(int count, int parts) {
        for (int index = 0; index < count; ++index) {
            const std::string id = Hex(next_id_++);
            // The scene adapter's names: the managed prefix, "scene_" and the hex of the id.
            const std::string name = "scene_" + id;
            Obstacle obstacle;
            obstacle.model = std::make_shared<Model>();
            const int count_of_parts = parts > 0 ? parts : static_cast<int>(1 + Integer(0, 3));
            for (int part = 0; part < count_of_parts; ++part)
                obstacle.parts.push_back(MakePart(name, part));
            obstacles_.emplace(name, std::move(obstacle));
        }
    }

    /// Removes up to `count` obstacles, chosen at random.
    void Remove(int count) {
        for (int index = 0; index < count && !obstacles_.empty(); ++index) {
            auto victim = obstacles_.begin();
            std::advance(victim, Integer(0, obstacles_.size() - 1));
            obstacles_.erase(victim);
        }
    }

    /// Moves everything: poses, velocities and motions. `fraction` of
    /// the obstacles are driven by a motion of any mode, the rest are free.
    void Advance(double simulation_time, double fraction) {
        for (auto& item : obstacles_) {
            Obstacle& obstacle = item.second;
            obstacle.observed_pose = RandomPose();
            obstacle.model->linear_velocity = RandomVector(2.0);
            obstacle.model->angular_velocity = RandomVector(1.0);
            obstacle.model->is_static = Real(0.0, 1.0) < 0.8;
            if (Real(0.0, 1.0) < 0.2)
                Control(&obstacle, simulation_time, Real(0.0, 1.0) < fraction);
        }
    }

  private:
    static std::string Hex(std::uint64_t value) {
        std::ostringstream out;
        out << std::hex << std::setfill('0') << std::setw(16) << value << std::setw(24) << value * 2654435761u;
        return out.str();
    }

    std::uint64_t Integer(std::uint64_t first, std::uint64_t last) {
        return std::uniform_int_distribution<std::uint64_t>(first, last)(random_);
    }

    double Real(double first, double last) { return std::uniform_real_distribution<double>(first, last)(random_); }

    ignition::math::Vector3d RandomVector(double extent) {
        return {Real(-extent, extent), Real(-extent, extent), Real(-extent, extent)};
    }

    ignition::math::Pose3d RandomPose() {
        ignition::math::Quaterniond rotation(Real(-1, 1), Real(-1, 1), Real(-1, 1), Real(-1, 1) + 2.5);
        rotation.Normalize();
        return {RandomVector(50.0), rotation};
    }

    geometry_msgs::Vector3 RandomSize() {
        geometry_msgs::Vector3 size;
        size.x = Real(0.1, 4.0);
        size.y = Real(0.1, 4.0);
        size.z = Real(0.1, 4.0);
        return size;
    }

    xgc2_geometry_msgs::ConvexBodyInstance MakePart(const std::string& name, int index) {
        xgc2_geometry_msgs::ConvexBodyInstance part;
        part.name = "body/part_" + Hex(index) + name.substr(name.size() > 8 ? name.size() - 8 : 0);
        part.pose = PoseMessage(RandomPose());
        switch (Integer(0, 3)) {
        case 0:
            part.geometry_type = "cube";
            part.scale = RandomSize();
            break;
        case 1:
            part.geometry_type = "sphere";
            part.scale.x = part.scale.y = part.scale.z = Real(0.1, 2.0);
            break;
        case 2:
            part.geometry_type = "cylinder";
            part.scale.x = part.scale.y = Real(0.1, 2.0);
            part.scale.z = Real(0.1, 4.0);
            break;
        default:
            part.geometry_type = "convex_mesh:file:///tmp/xgc2-scene-meshes-" + Hex(Integer(0, 1000)).substr(10, 6) +
                                 "/" + Hex(index) + Hex(index + 1) + ".obj";
            if (Integer(0, 2) == 0)
                part.geometry_type += "#submesh=sub_" + std::to_string(Integer(0, 9));
            if (Integer(0, 2) == 0)
                part.geometry_type += "#centered";
            part.scale = RandomSize();
            break;
        }
        return part;
    }

    void Control(Obstacle* obstacle, double simulation_time, bool controlled) {
        obstacle->controlled = controlled;
        if (!controlled)
            return;
        MotionConfiguration configuration;
        configuration.mode = static_cast<MotionMode>(Integer(0, 3));
        configuration.linear_velocity = RandomVector(2.0);
        configuration.angular_velocity = RandomVector(1.0);
        configuration.speed = Real(0.5, 2.0);
        const ignition::math::Vector3d start = obstacle->observed_pose.Pos();
        if (configuration.mode == MotionMode::kPingPong)
            configuration.waypoints = {start, start + ignition::math::Vector3d(Real(1, 5), Real(1, 5), Real(0, 1))};
        else if (configuration.mode == MotionMode::kCircle)
            configuration.waypoints = {start + ignition::math::Vector3d(Real(1, 5), Real(1, 5), 0)};
        std::string error;
        if (!obstacle->controller.Configure(configuration, obstacle->observed_pose, simulation_time, &error))
            obstacle->controlled = false;
    }

    std::mt19937 random_;
    std::uint64_t next_id_ = 1;
    Obstacles obstacles_;
};

} // namespace fixture
} // namespace xgc2_gazebo_scene
