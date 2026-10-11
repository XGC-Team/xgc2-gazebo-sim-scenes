#pragma once

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

/// What changes in an obstacle's messages from one publication to the next.
struct ObstacleDynamics {
    ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;
    ignition::math::Vector3d linear_velocity = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d angular_velocity = ignition::math::Vector3d::Zero;
    bool is_static = false;
};

/// The body instance message published every 33 ms of simulation time, kept
/// between publications. Reset() caches names, ids, geometry types, scales
/// and local collision poses. Set() writes only the dynamic fields, so
/// publishing an unchanged set allocates nothing.
class ObstacleMessages {
  public:
    /// The fixed data of one obstacle, in publication order.
    struct Obstacle {
        std::string name;
        const std::vector<xgc2_geometry_msgs::ConvexBodyInstance>* parts = nullptr;
    };

    ObstacleMessages() {
        instances_.header.frame_id = "world";
    }

    /// Replaces the set of obstacles. Everything it keeps is copied: the
    /// local parts are read here only.
    void Reset(const std::vector<Obstacle>& obstacles) {
        instances_.instances.clear();
        first_instance_.clear();
        first_instance_.reserve(obstacles.size() + 1);
        local_poses_.clear();
        std::int32_t instance_id = 1;
        for (const auto& obstacle : obstacles) {
            first_instance_.push_back(instances_.instances.size());
            const bool single_part = obstacle.parts->size() == 1;
            for (const auto& part : *obstacle.parts) {
                xgc2_geometry_msgs::ConvexBodyInstance instance = part;
                instance.id = instance_id++;
                instance.name = single_part ? obstacle.name : obstacle.name + "/" + part.name;
                instances_.instances.push_back(std::move(instance));
                local_poses_.push_back(IgnitionPose(part.pose));
            }
        }
        first_instance_.push_back(instances_.instances.size());
    }

    /// Starts a publication.
    void Begin(const ros::Time& stamp) { instances_.header.stamp = stamp; }

    /// Writes the dynamic fields of obstacle `index` (publication order).
    void Set(std::size_t index, const ObstacleDynamics& dynamics) {
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
    std::size_t size() const { return first_instance_.empty() ? 0 : first_instance_.size() - 1; }

    const xgc2_geometry_msgs::ConvexBodyArray& instances() const { return instances_; }

  private:
    xgc2_geometry_msgs::ConvexBodyArray instances_;
    std::vector<std::size_t> first_instance_;
    std::vector<ignition::math::Pose3d> local_poses_;
};

} // namespace xgc2_gazebo_scene
