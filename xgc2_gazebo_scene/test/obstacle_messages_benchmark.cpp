// The 30 Hz obstacle state and body instance publication of
// GazeboSceneSystemPlugin, without Gazebo, on the real message types.
//
// "rebuilt" is what 1.4.1-1 did for every publication (legacy_obstacle_messages.hpp):
// build both messages from scratch. "kept" is ObstacleMessages: the names,
// ids, geometry types and scales are set once and a publication writes only
// the numbers that move. "build" times the messages alone; "publish" adds what
// a latched roscpp publisher always does with them, serialize into a fresh
// buffer. Heap allocations are counted over both. The two must serialize to
// the same bytes; the benchmark fails if they do not.
//
// Build and run: test/run_obstacle_messages_benchmark.sh
#include "legacy_obstacle_messages.hpp"
#include "obstacle_fixture.hpp"

#include <ros/serialization.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <string>
#include <utility>
#include <vector>

namespace {

std::atomic<std::size_t> g_allocations{0};

} // namespace

// noinline: GCC would otherwise pair an inlined malloc-based new with the free
// in delete and report a mismatched allocation.
#define XGC2_NOINLINE __attribute__((noinline))
XGC2_NOINLINE void* operator new(std::size_t size) {
    ++g_allocations;
    if (void* memory = std::malloc(size == 0 ? 1 : size))
        return memory;
    throw std::bad_alloc();
}
XGC2_NOINLINE void* operator new[](std::size_t size) {
    return operator new(size);
}
XGC2_NOINLINE void operator delete(void* memory) noexcept {
    std::free(memory);
}
XGC2_NOINLINE void operator delete[](void* memory) noexcept {
    std::free(memory);
}
XGC2_NOINLINE void operator delete(void* memory, std::size_t) noexcept {
    std::free(memory);
}
XGC2_NOINLINE void operator delete[](void* memory, std::size_t) noexcept {
    std::free(memory);
}

namespace {

using namespace xgc2_gazebo_scene;
using Clock = std::chrono::steady_clock;

struct Cost {
    double microseconds = 0.0;
    double allocations = 0.0;
};

template <class Message> std::size_t Publish(const Message& message) {
    // What a latched ros::Publisher::publish does: serialize into a new buffer.
    const ros::SerializedMessage serialized = ros::serialization::serializeMessage(message);
    return serialized.num_bytes;
}

std::vector<std::uint8_t> Bytes(const ros::SerializedMessage& serialized) {
    return std::vector<std::uint8_t>(serialized.buf.get(), serialized.buf.get() + serialized.num_bytes);
}

template <class Function> Cost Measure(int publications, Function&& publication) {
    // Warm up, then time `publications` of them.
    for (int round = 0; round < 3; ++round)
        publication();
    const std::size_t allocations_before = g_allocations;
    const auto start = Clock::now();
    for (int round = 0; round < publications; ++round)
        publication();
    const double elapsed = std::chrono::duration<double, std::micro>(Clock::now() - start).count();
    return {elapsed / publications, static_cast<double>(g_allocations - allocations_before) / publications};
}

int Run() {
    struct Case {
        int obstacles, parts;
    };
    std::printf("%10s %6s %9s | %26s | %26s | %9s\n", "obstacles", "parts", "instances", "build us (rebuilt -> kept)",
                "publish us (rebuilt -> kept)", "allocations");
    int mismatches = 0;
    for (const Case& scene_case : {Case{80, 32}, Case{200, 2}, Case{500, 2}, Case{500, 1}}) {
        fixture::Scene scene(20261001);
        scene.Add(scene_case.obstacles, scene_case.parts);
        scene.Advance(1.0, 0.1);
        const fixture::Obstacles& obstacles = scene.obstacles();
        const ros::Time stamp(12, 345);
        const std::string epoch = "world:1700000000000000000";
        const int publications = std::max(20, 20000 / (scene_case.obstacles * scene_case.parts) + 20);

        std::size_t sink = 0;
        const Cost rebuilt_build = Measure(publications, [&] {
            legacy::PublishState(
                obstacles, stamp, epoch, 7, 1.0,
                [&](const ObstacleStateArray& message) {
                    sink += message.obstacles.size();
                },
                [&](const xgc2_geometry_msgs::ConvexBodyArray& message) {
                    sink += message.instances.size();
                });
        });
        const Cost rebuilt_publish = Measure(publications, [&] {
            legacy::PublishState(
                obstacles, stamp, epoch, 7, 1.0,
                [&](const ObstacleStateArray& message) {
                    sink += Publish(message);
                },
                [&](const xgc2_geometry_msgs::ConvexBodyArray& message) {
                    sink += Publish(message);
                });
        });

        ObstacleMessages messages;
        std::vector<ObstacleMessages::Obstacle> fixed;
        for (const auto& item : obstacles)
            fixed.push_back(
                {item.first, item.second.model->GetName(), item.second.generation, &item.second.definition});
        messages.Reset(fixed);
        ObstacleDynamics dynamics;
        const auto build = [&] {
            messages.Begin(stamp, epoch, 7);
            std::size_t index = 0;
            for (const auto& item : obstacles) {
                SampleObstacle(item.second, 1.0, &dynamics);
                messages.Set(index++, dynamics);
            }
        };
        const Cost kept_build = Measure(publications, [&] {
            build();
            sink += messages.state().obstacles.size() + messages.instances().instances.size();
        });
        const Cost kept_publish = Measure(publications, [&] {
            build();
            sink += Publish(messages.state());
            sink += Publish(messages.instances());
        });

        // Same bytes?
        std::vector<std::uint8_t> state_bytes, instances_bytes;
        legacy::PublishState(
            obstacles, stamp, epoch, 7, 1.0,
            [&](const ObstacleStateArray& message) {
                state_bytes = Bytes(ros::serialization::serializeMessage(message));
            },
            [&](const xgc2_geometry_msgs::ConvexBodyArray& message) {
                instances_bytes = Bytes(ros::serialization::serializeMessage(message));
            });
        build();
        if (Bytes(ros::serialization::serializeMessage(messages.state())) != state_bytes ||
            Bytes(ros::serialization::serializeMessage(messages.instances())) != instances_bytes) {
            ++mismatches;
            std::fprintf(stderr, "MISMATCH for %d obstacles x %d parts\n", scene_case.obstacles, scene_case.parts);
        }

        std::printf("%10d %6d %9zu | %9.1f -> %7.1f %5.1fx | %9.1f -> %7.1f %5.1fx | %5.0f -> %.0f\n",
                    scene_case.obstacles, scene_case.parts, messages.instances().instances.size(),
                    rebuilt_build.microseconds, kept_build.microseconds,
                    rebuilt_build.microseconds / kept_build.microseconds, rebuilt_publish.microseconds,
                    kept_publish.microseconds, rebuilt_publish.microseconds / kept_publish.microseconds,
                    rebuilt_publish.allocations, kept_publish.allocations);
        if (sink == 1)
            std::puts("");
    }
    return mismatches == 0 ? 0 : 1;
}

} // namespace

int main() {
    try {
        return Run();
    } catch (...) {
        std::fputs("benchmark failed\n", stderr);
        return 2;
    }
}
