#include "legacy_obstacle_messages.hpp"
#include "obstacle_fixture.hpp"

#include <gtest/gtest.h>
#include <ros/serialization.h>

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
using Bytes = std::vector<std::uint8_t>;

template <class Message> Bytes Serialize(const Message& message) {
    namespace serialization = ros::serialization;
    Bytes bytes(serialization::serializationLength(message));
    serialization::OStream stream(bytes.data(), static_cast<std::uint32_t>(bytes.size()));
    serialization::serialize(stream, message);
    return bytes;
}

struct Published {
    Bytes state;
    Bytes instances;
};

Published Legacy(const Obstacles& obstacles, const ros::Time& stamp, const std::string& epoch, std::uint64_t revision,
                 double simulation_time) {
    Published published;
    legacy::PublishState(
        obstacles, stamp, epoch, revision, simulation_time,
        [&](const ObstacleStateArray& message) {
            published.state = Serialize(message);
        },
        [&](const xgc2_geometry_msgs::ConvexBodyArray& message) {
            published.instances = Serialize(message);
        });
    return published;
}

std::vector<ObstacleMessages::Obstacle> Fixed(const Obstacles& obstacles) {
    std::vector<ObstacleMessages::Obstacle> fixed;
    for (const auto& item : obstacles)
        fixed.push_back({item.first, item.second.model->GetName(), item.second.generation, &item.second.definition});
    return fixed;
}

void Publish(ObstacleMessages* messages, const Obstacles& obstacles, const ros::Time& stamp, const std::string& epoch,
             std::uint64_t revision, double simulation_time) {
    messages->Begin(stamp, epoch, revision);
    ObstacleDynamics dynamics;
    std::size_t index = 0;
    for (const auto& item : obstacles) {
        SampleObstacle(item.second, simulation_time, &dynamics);
        messages->Set(index++, dynamics);
    }
}

// What the plugin does each 33 ms: the messages that were rebuilt from the
// obstacles and the ones kept between publications must serialize to the same
// bytes.
void ExpectSame(ObstacleMessages* messages, const Obstacles& obstacles, double simulation_time, int round) {
    const ros::Time stamp(static_cast<std::uint32_t>(simulation_time), static_cast<std::uint32_t>(round * 1000));
    const std::string epoch = round % 7 == 0 ? "world:1700000000000000000" : "world:1700000000000000001";
    const std::uint64_t revision = 3 + static_cast<std::uint64_t>(round) / 5;
    const Published expected = Legacy(obstacles, stamp, epoch, revision, simulation_time);
    Publish(messages, obstacles, stamp, epoch, revision, simulation_time);
    EXPECT_EQ(Serialize(messages->state()), expected.state) << "state, round " << round;
    EXPECT_EQ(Serialize(messages->instances()), expected.instances) << "instances, round " << round;
}

TEST(ObstacleMessages, AnEmptySceneIsTwoEmptyArrays) {
    const Obstacles none;
    ObstacleMessages messages;
    messages.Reset({});
    ExpectSame(&messages, none, 1.0, 0);
    EXPECT_TRUE(messages.state().obstacles.empty());
    EXPECT_TRUE(messages.instances().instances.empty());
    EXPECT_EQ(messages.state().header.frame_id, "world");
}

TEST(ObstacleMessages, SerializeToTheBytesOfTheRebuiltMessagesEveryPublication) {
    Scene scene(20261001);
    scene.Add(10, 0);
    scene.Add(3, 1);
    scene.Add(2, 32);
    ObstacleMessages messages;
    messages.Reset(Fixed(scene.obstacles()));
    for (int round = 0; round < 80; ++round) {
        const double simulation_time = 0.5 + round / 30.0;
        scene.Advance(simulation_time, 0.6);
        ExpectSame(&messages, scene.obstacles(), simulation_time, round);
    }
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
            ExpectSame(&messages, scene.obstacles(), 2.0 + round, round);
        }
    }
}

TEST(ObstacleMessages, EveryMotionModeIsPublishedByName) {
    Scene scene(99);
    scene.Add(40, 1);
    ObstacleMessages messages;
    messages.Reset(Fixed(scene.obstacles()));
    std::set<std::string> modes;
    for (int round = 0; round < 30; ++round) {
        scene.Advance(1.0 + round, 1.0);
        ExpectSame(&messages, scene.obstacles(), 1.0 + round, round);
        for (const auto& state : messages.state().obstacles)
            modes.insert(state.motion_mode);
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
    Publish(&messages, scene.obstacles(), ros::Time(1, 0), "world:1", 1, 1.0);

    const auto* states = messages.state().obstacles.data();
    const auto* instances = messages.instances().instances.data();
    const std::size_t instance_count = messages.instances().instances.size();
    std::vector<const char*> strings;
    for (const auto& state : messages.state().obstacles)
        strings.insert(strings.end(), {state.name.data(), state.model_name.data()});
    for (const auto& instance : messages.instances().instances)
        strings.insert(strings.end(), {instance.name.data(), instance.geometry_type.data()});

    for (int round = 2; round < 20; ++round) {
        scene.Advance(round, 0.5);
        Publish(&messages, scene.obstacles(), ros::Time(round, 0), "world:1", 1, round);
    }
    EXPECT_EQ(messages.state().obstacles.data(), states);
    EXPECT_EQ(messages.instances().instances.data(), instances);
    EXPECT_EQ(messages.instances().instances.size(), instance_count);
    std::vector<const char*> after;
    for (const auto& state : messages.state().obstacles)
        after.insert(after.end(), {state.name.data(), state.model_name.data()});
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
    EXPECT_EQ(first.definition.parts.size() == 1 ? name : name + "/" + first.definition.parts[0].part_id,
              messages.instances().instances[0].name);
}

} // namespace
} // namespace xgc2_gazebo_scene

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
