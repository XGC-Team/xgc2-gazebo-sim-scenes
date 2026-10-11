#include "obstacle_fixture.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace xgc2_gazebo_scene {
namespace {

using fixture::Obstacles;
using fixture::Scene;

std::vector<ObstacleMessages::Obstacle> Fixed(const Obstacles& obstacles) {
    std::vector<ObstacleMessages::Obstacle> fixed;
    for (const auto& item : obstacles)
        fixed.push_back({item.first, &item.second.parts});
    return fixed;
}

void Publish(ObstacleMessages* messages, const Obstacles& obstacles, const ros::Time& stamp, double simulation_time) {
    messages->Begin(stamp);
    ObstacleDynamics dynamics;
    std::size_t index = 0;
    for (const auto& item : obstacles) {
        SampleObstacle(item.second, simulation_time, &dynamics);
        messages->Set(index++, dynamics);
    }
}

void ExpectVector(const geometry_msgs::Vector3& actual, const ignition::math::Vector3d& expected) {
    EXPECT_DOUBLE_EQ(actual.x, expected.X());
    EXPECT_DOUBLE_EQ(actual.y, expected.Y());
    EXPECT_DOUBLE_EQ(actual.z, expected.Z());
}

void ExpectInstances(ObstacleMessages* messages, const Obstacles& obstacles, double simulation_time, int round) {
    const ros::Time stamp(static_cast<std::uint32_t>(simulation_time), static_cast<std::uint32_t>(round * 1000));
    Publish(messages, obstacles, stamp, simulation_time);
    const auto& published = messages->instances();
    EXPECT_EQ(published.header.stamp, stamp);
    EXPECT_EQ(published.header.frame_id, "world");
    std::size_t index = 0;
    for (const auto& item : obstacles) {
        const auto& obstacle = item.second;
        const MotionSample motion = obstacle.controlled ? obstacle.controller.Sample(simulation_time) : MotionSample{};
        const ignition::math::Vector3d linear =
            obstacle.controlled ? motion.linear_velocity : obstacle.model->WorldLinearVel();
        const ignition::math::Vector3d angular =
            obstacle.controlled ? motion.angular_velocity : obstacle.model->WorldAngularVel();
        const bool is_static =
            obstacle.model->IsStatic() && (!obstacle.controlled || obstacle.controller.mode() == MotionMode::kHold);
        for (const auto& part : obstacle.parts) {
            ASSERT_LT(index, published.instances.size());
            const auto& actual = published.instances[index++];
            EXPECT_EQ(actual.id, static_cast<std::int32_t>(index));
            EXPECT_EQ(actual.name, obstacle.parts.size() == 1 ? item.first : item.first + "/" + part.name);
            EXPECT_EQ(actual.geometry_type, part.geometry_type);
            EXPECT_EQ(actual.scale, part.scale);
            EXPECT_EQ(actual.is_static, is_static);
            const ignition::math::Pose3d local_pose = IgnitionPose(part.pose);
            const ignition::math::Vector3d offset = obstacle.observed_pose.Rot().RotateVector(local_pose.Pos());
            const ignition::math::Vector3d position = obstacle.observed_pose.Pos() + offset;
            EXPECT_DOUBLE_EQ(actual.pose.position.x, position.X());
            EXPECT_DOUBLE_EQ(actual.pose.position.y, position.Y());
            EXPECT_DOUBLE_EQ(actual.pose.position.z, position.Z());
            const ignition::math::Quaterniond rotation = obstacle.observed_pose.Rot() * local_pose.Rot();
            EXPECT_DOUBLE_EQ(actual.pose.orientation.x, rotation.X());
            EXPECT_DOUBLE_EQ(actual.pose.orientation.y, rotation.Y());
            EXPECT_DOUBLE_EQ(actual.pose.orientation.z, rotation.Z());
            EXPECT_DOUBLE_EQ(actual.pose.orientation.w, rotation.W());
            ExpectVector(actual.velocity.linear, linear + angular.Cross(offset));
            ExpectVector(actual.velocity.angular, angular);
        }
    }
    EXPECT_EQ(published.instances.size(), index);
}

TEST(ObstacleMessages, AnEmptySceneHasNoInstances) {
    const Obstacles none;
    ObstacleMessages messages;
    messages.Reset({});
    ExpectInstances(&messages, none, 1.0, 0);
    EXPECT_TRUE(messages.instances().instances.empty());
}

TEST(ObstacleMessages, FollowTheObstaclesWhenTheSetChanges) {
    Scene scene(7);
    scene.Add(6, 0);
    ObstacleMessages messages;
    int round = 0;
    for (const auto& change : {std::make_pair(4, 0), std::make_pair(-3, 0), std::make_pair(5, 2),
                               std::make_pair(-20, 0), std::make_pair(2, 1), std::make_pair(9, 0)}) {
        if (change.first > 0)
            scene.Add(change.first, change.second);
        else
            scene.Remove(-change.first);
        messages.Reset(Fixed(scene.obstacles()));
        for (int step = 0; step < 6; ++step, ++round) {
            scene.Advance(2.0 + round, 0.5);
            ExpectInstances(&messages, scene.obstacles(), 2.0 + round, round);
        }
    }
}

TEST(ObstacleMessages, EveryMotionModeSuppliesInstanceVelocities) {
    Scene scene(99);
    scene.Add(40, 1);
    ObstacleMessages messages;
    messages.Reset(Fixed(scene.obstacles()));
    std::set<std::string> modes;
    for (int round = 0; round < 30; ++round) {
        scene.Advance(1.0 + round, 1.0);
        ExpectInstances(&messages, scene.obstacles(), 1.0 + round, round);
        for (const auto& item : scene.obstacles())
            modes.insert(item.second.controlled ? item.second.controller.modeName() : "uncontrolled");
    }
    EXPECT_EQ(modes, (std::set<std::string>{"circle", "constant_twist", "hold", "ping_pong", "uncontrolled"}));
}

TEST(ObstacleMessages, PublishingAnUnchangedSetReusesTheMessages) {
    Scene scene(5);
    scene.Add(20, 0);
    scene.Add(4, 8);
    ObstacleMessages messages;
    messages.Reset(Fixed(scene.obstacles()));
    scene.Advance(1.0, 0.5);
    Publish(&messages, scene.obstacles(), ros::Time(1, 0), 1.0);

    const auto* instances = messages.instances().instances.data();
    const std::size_t instance_count = messages.instances().instances.size();
    std::vector<const char*> strings;
    for (const auto& instance : messages.instances().instances)
        strings.insert(strings.end(), {instance.name.data(), instance.geometry_type.data()});

    for (int round = 2; round < 20; ++round) {
        scene.Advance(round, 0.5);
        ExpectInstances(&messages, scene.obstacles(), round, round);
    }
    EXPECT_EQ(messages.instances().instances.data(), instances);
    EXPECT_EQ(messages.instances().instances.size(), instance_count);
    std::vector<const char*> after;
    for (const auto& instance : messages.instances().instances)
        after.insert(after.end(), {instance.name.data(), instance.geometry_type.data()});
    EXPECT_EQ(after, strings) << "no string was reallocated";
}

TEST(ObstacleMessages, KnowsHowManyObstaclesItWasResetTo) {
    Scene scene(11);
    ObstacleMessages messages;
    EXPECT_EQ(messages.size(), 0u);
    scene.Add(7, 2);
    messages.Reset(Fixed(scene.obstacles()));
    EXPECT_EQ(messages.size(), 7u);
    scene.Remove(3);
    messages.Reset(Fixed(scene.obstacles()));
    EXPECT_EQ(messages.size(), 4u);
}

TEST(ObstacleMessages, InstanceIdsRunFromOneThroughEveryPartInPublicationOrder) {
    Scene scene(3);
    scene.Add(5, 3);
    scene.Add(2, 1);
    ObstacleMessages messages;
    messages.Reset(Fixed(scene.obstacles()));
    std::int32_t expected = 1;
    for (const auto& instance : messages.instances().instances)
        EXPECT_EQ(instance.id, expected++);
    EXPECT_EQ(expected, 5 * 3 + 2 + 1);
    // One part is named after the obstacle, several after the obstacle and the part.
    const auto& first = scene.obstacles().begin()->second;
    const std::string name = scene.obstacles().begin()->first;
    EXPECT_EQ(first.parts.size() == 1 ? name : name + "/" + first.parts[0].name,
              messages.instances().instances[0].name);
}

} // namespace
} // namespace xgc2_gazebo_scene

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
