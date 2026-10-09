#pragma once

#include "xgc2_gazebo_scene/ConvexPart.h"
#include "xgc2_gazebo_scene/ObstacleDefinition.h"
#include "xgc2_gazebo_scene/ObstacleState.h"
#include "xgc2_gazebo_scene/ObstacleStateArray.h"
#include "xgc2_geometry_msgs/ConvexBodyArray.h"
#include "xgc2_geometry_msgs/ConvexBodyInstance.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ros/time.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace xgc2_gazebo_scene {

inline geometry_msgs::Point PointMessage(const ignition::math::Vector3d& value) {
    geometry_msgs::Point message;
    message.x = value.X();
    message.y = value.Y();
    message.z = value.Z();
    return message;
}

inline geometry_msgs::Pose PoseMessage(const ignition::math::Pose3d& value) {
    geometry_msgs::Pose message;
    message.position = PointMessage(value.Pos());
    message.orientation.x = value.Rot().X();
    message.orientation.y = value.Rot().Y();
    message.orientation.z = value.Rot().Z();
    message.orientation.w = value.Rot().W();
    return message;
}

inline ignition::math::Pose3d IgnitionPose(const geometry_msgs::Pose& value) {
    return {{value.position.x, value.position.y, value.position.z},
            {value.orientation.w, value.orientation.x, value.orientation.y, value.orientation.z}};
}

inline geometry_msgs::Twist TwistMessage(const ignition::math::Vector3d& linear,
                                         const ignition::math::Vector3d& angular) {
    geometry_msgs::Twist message;
    message.linear.x = linear.X();
    message.linear.y = linear.Y();
    message.linear.z = linear.Z();
    message.angular.x = angular.X();
    message.angular.y = angular.Y();
    message.angular.z = angular.Z();
    return message;
}

inline geometry_msgs::Vector3 VectorMessage(const ignition::math::Vector3d& value) {
    geometry_msgs::Vector3 message;
    message.x = value.X();
    message.y = value.Y();
    message.z = value.Z();
    return message;
}

inline std::string StandardGeometryType(const ConvexPart& part) {
    switch (part.shape) {
    case ConvexPart::SHAPE_BOX:
        return "cube";
    case ConvexPart::SHAPE_SPHERE:
        return "sphere";
    case ConvexPart::SHAPE_CYLINDER:
        return "cylinder";
    case ConvexPart::SHAPE_CONVEX_MESH: {
        std::string type = "convex_mesh:" + part.mesh_uri;
        if (!part.mesh_submesh.empty()) {
            type += "#submesh=" + part.mesh_submesh;
        }
        if (part.mesh_center_submesh) {
            type += "#centered";
        }
        return type;
    }
    default:
        return "";
    }
}

inline geometry_msgs::Vector3 StandardInstanceScale(const ConvexPart& part) {
    switch (part.shape) {
    case ConvexPart::SHAPE_BOX:
        return part.size;
    case ConvexPart::SHAPE_SPHERE:
        return VectorMessage({part.radius, part.radius, part.radius});
    case ConvexPart::SHAPE_CYLINDER:
        return VectorMessage({part.radius, part.radius, part.length});
    case ConvexPart::SHAPE_CONVEX_MESH:
        return part.mesh_scale;
    default:
        return geometry_msgs::Vector3{};
    }
}

/// What changes in an obstacle's messages from one publication to the next.
struct ObstacleDynamics {
    ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;
    ignition::math::Vector3d linear_velocity = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d angular_velocity = ignition::math::Vector3d::Zero;
    std::string motion_mode;
    std::uint64_t motion_revision = 0;
    bool is_static = false;
};

/// The obstacle state and body instance messages published every 33 ms of
/// simulation time, kept between publications.
///
/// An obstacle's name, model name, generation and its parts' instance ids,
/// names, geometry types and scales change only when the set of obstacles or
/// a definition does, which is when Reset() is called. Building them anew for
/// every publication copied about two strings per obstacle and one per part
/// (about 1,000 at 500 obstacles), grew two vectors and destroyed both
/// messages again. Set() writes only the numbers that move, and the motion
/// mode when it differs, so publishing an unchanged set allocates nothing.
/// The messages hold the same values the rebuilt ones did: the arithmetic is
/// the same expressions on the same doubles.
class ObstacleMessages {
  public:
    /// The fixed data of one obstacle, in publication order.
    struct Obstacle {
        std::string name;
        std::string model_name;
        std::uint64_t generation = 0;
        const ObstacleDefinition* definition = nullptr;
    };

    ObstacleMessages() {
        state_.header.frame_id = "world";
        instances_.header.frame_id = "world";
    }

    /// Replaces the set of obstacles. Everything it keeps is copied: the
    /// definitions are read here only.
    void Reset(const std::vector<Obstacle>& obstacles) {
        state_.obstacles.clear();
        state_.obstacles.reserve(obstacles.size());
        instances_.instances.clear();
        first_instance_.clear();
        first_instance_.reserve(obstacles.size() + 1);
        local_poses_.clear();
        std::int32_t instance_id = 1;
        for (const auto& obstacle : obstacles) {
            ObstacleState state;
            state.name = obstacle.name;
            state.model_name = obstacle.model_name;
            state.generation = obstacle.generation;
            state_.obstacles.push_back(std::move(state));
            first_instance_.push_back(instances_.instances.size());
            const bool single_part = obstacle.definition->parts.size() == 1;
            for (const auto& part : obstacle.definition->parts) {
                xgc2_geometry_msgs::ConvexBodyInstance instance;
                instance.id = instance_id++;
                instance.name = single_part ? obstacle.name : obstacle.name + "/" + part.part_id;
                instance.geometry_type = StandardGeometryType(part);
                instance.scale = StandardInstanceScale(part);
                instances_.instances.push_back(std::move(instance));
                local_poses_.push_back(IgnitionPose(part.local_pose));
            }
        }
        first_instance_.push_back(instances_.instances.size());
    }

    /// Starts a publication.
    void Begin(const ros::Time& stamp, const std::string& scene_epoch, std::uint64_t scene_revision) {
        state_.header.stamp = stamp;
        if (state_.scene_epoch != scene_epoch) {
            state_.scene_epoch = scene_epoch;
        }
        state_.scene_revision = scene_revision;
        instances_.header = state_.header;
    }

    /// Writes the dynamic fields of obstacle `index` (publication order).
    void Set(std::size_t index, const ObstacleDynamics& dynamics) {
        ObstacleState& state = state_.obstacles[index];
        state.pose = PoseMessage(dynamics.pose);
        state.twist = TwistMessage(dynamics.linear_velocity, dynamics.angular_velocity);
        if (state.motion_mode != dynamics.motion_mode) {
            state.motion_mode = dynamics.motion_mode;
        }
        state.motion_revision = dynamics.motion_revision;
        for (std::size_t part = first_instance_[index]; part < first_instance_[index + 1]; ++part) {
            const ignition::math::Pose3d& local_pose = local_poses_[part];
            const ignition::math::Pose3d world_pose = dynamics.pose * local_pose;
            const ignition::math::Vector3d offset = dynamics.pose.Rot().RotateVector(local_pose.Pos());
            xgc2_geometry_msgs::ConvexBodyInstance& instance = instances_.instances[part];
            instance.pose = PoseMessage(world_pose);
            instance.is_static = dynamics.is_static;
            instance.velocity = TwistMessage(dynamics.linear_velocity + dynamics.angular_velocity.Cross(offset),
                                             dynamics.angular_velocity);
        }
    }

    /// The number of obstacles the messages were last reset to.
    std::size_t size() const { return state_.obstacles.size(); }

    const ObstacleStateArray& state() const { return state_; }
    const xgc2_geometry_msgs::ConvexBodyArray& instances() const { return instances_; }

  private:
    ObstacleStateArray state_;
    xgc2_geometry_msgs::ConvexBodyArray instances_;
    std::vector<std::size_t> first_instance_;
    std::vector<ignition::math::Pose3d> local_poses_;
};

} // namespace xgc2_gazebo_scene
