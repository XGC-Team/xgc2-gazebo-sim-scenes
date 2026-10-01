// Frozen copy of GazeboSceneSystemPlugin::PublishState as of 1.4.1-1, before
// ObstacleMessages: both messages are built from scratch for every
// publication. Used only by obstacle_messages_test and
// obstacle_messages_benchmark, as the reference the cached messages must
// equal byte for byte. Do not change it.
#pragma once

#include "xgc2_gazebo_scene/motion_controller.hpp"
#include "xgc2_gazebo_scene/obstacle_messages.hpp"

#include <cstdint>
#include <string>
#include <utility>

namespace xgc2_gazebo_scene {
namespace legacy {

/// `Obstacles` maps a logical name to an obstacle with `model`, `generation`,
/// `definition`, `observed_pose`, `controlled`, `controller` and
/// `motion_revision`, as the plugin's ManagedObstacle does. `publish_state`
/// and `publish_instances` take the finished messages, as the publishers do.
template <class Obstacles, class StatePublisher, class InstancesPublisher>
void PublishState(const Obstacles& obstacles, const ros::Time& stamp, const std::string& scene_epoch,
                  std::uint64_t scene_revision, double simulation_time, StatePublisher&& publish_state,
                  InstancesPublisher&& publish_instances) {
    ObstacleStateArray message;
    message.header.stamp = stamp;
    message.header.frame_id = "world";
    message.scene_epoch = scene_epoch;
    message.scene_revision = scene_revision;
    for (const auto& item : obstacles) {
        const auto& obstacle = item.second;
        ObstacleState state;
        state.name = item.first;
        state.model_name = obstacle.model->GetName();
        state.generation = obstacle.generation;
        state.pose = PoseMessage(obstacle.observed_pose);
        if (obstacle.controlled) {
            const MotionSample sample = obstacle.controller.Sample(simulation_time);
            state.twist = TwistMessage(sample.linear_velocity, sample.angular_velocity);
            state.motion_mode = obstacle.controller.modeName();
        } else {
            state.twist = TwistMessage(obstacle.model->WorldLinearVel(), obstacle.model->WorldAngularVel());
            state.motion_mode = "uncontrolled";
        }
        state.motion_revision = obstacle.motion_revision;
        message.obstacles.push_back(std::move(state));
    }
    publish_state(message);

    xgc2_geometry_msgs::ConvexBodyArray instances;
    instances.header = message.header;
    std::int32_t instance_id = 1;
    for (const auto& item : obstacles) {
        const auto& obstacle = item.second;
        ignition::math::Vector3d linear_velocity;
        ignition::math::Vector3d angular_velocity;
        if (obstacle.controlled) {
            const MotionSample sample = obstacle.controller.Sample(simulation_time);
            linear_velocity = sample.linear_velocity;
            angular_velocity = sample.angular_velocity;
        } else {
            linear_velocity = obstacle.model->WorldLinearVel();
            angular_velocity = obstacle.model->WorldAngularVel();
        }
        const bool is_static =
            obstacle.model->IsStatic() && (!obstacle.controlled || obstacle.controller.modeName() == "hold");
        const bool single_part = obstacle.definition.parts.size() == 1;
        for (const auto& part : obstacle.definition.parts) {
            const ignition::math::Pose3d local_pose = IgnitionPose(part.local_pose);
            const ignition::math::Pose3d world_pose = obstacle.observed_pose * local_pose;
            const ignition::math::Vector3d offset = obstacle.observed_pose.Rot().RotateVector(local_pose.Pos());
            xgc2_geometry_msgs::ConvexBodyInstance instance;
            instance.id = instance_id++;
            instance.name = single_part ? item.first : item.first + "/" + part.part_id;
            instance.geometry_type = StandardGeometryType(part);
            instance.pose = PoseMessage(world_pose);
            instance.scale = StandardInstanceScale(part);
            instance.is_static = is_static;
            instance.velocity = TwistMessage(linear_velocity + angular_velocity.Cross(offset), angular_velocity);
            instances.instances.push_back(std::move(instance));
        }
    }
    publish_instances(instances);
}

} // namespace legacy
} // namespace xgc2_gazebo_scene
