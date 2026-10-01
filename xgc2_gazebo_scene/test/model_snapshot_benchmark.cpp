// Model bookkeeping of the scene plugins, without Gazebo.
//
// GazeboSceneSystemPlugin, per world update. "rebuild" is what 1.4.0-2 did
// on every update: copy the model list twice (World::Models() returns the
// vector by value), rebuild the contact table and rediscover managed
// obstacles into fresh maps. "snapshot" is what an unchanged world costs
// now: one list copy and ModelSnapshot::Matches.
//
// SceneAuthoringWorldPlugin, per scene state message (30 Hz from the scene
// runtime, every obstacle). "ModelByName" is 1.4.0-2: two lookups per
// obstacle, each a depth-first walk of the entity tree that copies every
// node's scoped name and name (Gazebo 11 Base::GetByName). "cached" is one
// list copy, ModelSnapshot::Matches and two map lookups per obstacle.
//
// Names come from Base::GetName/GetScopedName (strings returned by value).
// Build and run: test/run_model_snapshot_benchmark.sh (any C++17 compiler).
#include "xgc2_gazebo_scene/model_snapshot.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

struct Model {
    std::string name;
    bool is_static = false;
    std::string GetName() const { return name; }
    bool IsStatic() const { return is_static; }
};

using ModelPtr = std::shared_ptr<Model>;

struct Descriptor {
    std::string name;
    bool is_static = false;
    bool is_managed_obstacle = false;
};

const std::string kPrefix = "xgc2_obstacle_";

std::string LogicalName(const std::string& model_name) {
    if (model_name.compare(0, kPrefix.size(), kPrefix) != 0)
        return "";
    return model_name.substr(kPrefix.size());
}

// A Gazebo 11 entity: name lookups follow Base::GetByName.
struct Entity : std::enable_shared_from_this<Entity> {
    std::string name, scoped_name;
    bool is_static = true;
    std::vector<std::shared_ptr<Entity>> children;
    std::string GetName() const { return name; }
    std::string GetScopedName() const { return scoped_name; }
    bool IsStatic() const { return is_static; }
    std::shared_ptr<Entity> GetByName(const std::string& wanted) {
        if (GetScopedName() == wanted || GetName() == wanted)
            return shared_from_this();
        std::shared_ptr<Entity> result;
        for (auto child = children.begin(); child != children.end() && result == nullptr; ++child)
            result = (*child)->GetByName(wanted);
        return result;
    }
    std::shared_ptr<Entity> Add(const std::string& child_name, bool child_static = true) {
        auto child = std::make_shared<Entity>();
        child->name = child_name;
        child->scoped_name = scoped_name.empty() ? child_name : scoped_name + "::" + child_name;
        child->is_static = child_static;
        children.push_back(child);
        return child;
    }
};

double Microseconds(std::chrono::steady_clock::time_point start, int updates) {
    return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count() / updates;
}

int Run() {
    std::printf("%7s %10s %12s %12s %7s\n", "robots", "obstacles", "rebuild us", "snapshot us", "ratio");
    for (const auto& size : {std::make_pair(8, 50), std::make_pair(32, 200), std::make_pair(100, 500)}) {
        std::vector<ModelPtr> world{std::make_shared<Model>(Model{"ground_plane", true})};
        for (int index = 0; index < size.second; ++index)
            world.push_back(std::make_shared<Model>(Model{kPrefix + "scene_" + std::to_string(index), true}));
        for (int index = 1; index <= size.first; ++index)
            world.push_back(std::make_shared<Model>(Model{"uav" + std::to_string(index), false}));
        const auto models_copy = [&world]() {
            return world;
        };

        std::map<std::string, ModelPtr> obstacles;
        for (const auto& model : world)
            if (!LogicalName(model->GetName()).empty())
                obstacles.emplace(LogicalName(model->GetName()), model);
        const int updates = 2000;
        std::size_t sink = 0;

        auto start = std::chrono::steady_clock::now();
        std::map<std::string, Descriptor> contact_models;
        for (int update = 0; update < updates; ++update) {
            contact_models.clear();
            for (const auto& model : models_copy()) {
                Descriptor descriptor;
                descriptor.name = model->GetName();
                descriptor.is_static = model->IsStatic();
                descriptor.is_managed_obstacle = !LogicalName(descriptor.name).empty();
                contact_models.emplace(descriptor.name, std::move(descriptor));
            }
            std::map<std::string, ModelPtr> current;
            for (const auto& model : models_copy()) {
                const std::string logical_name = LogicalName(model->GetName());
                if (!logical_name.empty())
                    current.emplace(logical_name, model);
            }
            for (const auto& item : obstacles) {
                const auto found = current.find(item.first);
                sink += found == current.end() || found->second != item.second ? 1 : 0;
            }
            for (const auto& item : current)
                sink += obstacles.count(item.first) == 0 ? 1 : 0;
        }
        const double rebuild = Microseconds(start, updates);

        xgc2_gazebo_scene::ModelSnapshot<ModelPtr, std::weak_ptr<Model>> snapshot;
        snapshot.Update(world);
        start = std::chrono::steady_clock::now();
        for (int update = 0; update < updates; ++update)
            sink += snapshot.Matches(models_copy()) ? 1 : 0;
        const double matches = Microseconds(start, updates);

        std::printf("%7d %10d %12.1f %12.1f %6.1fx\n", size.first, size.second, rebuild, matches, rebuild / matches);
        if (sink == 1)
            std::puts("");
    }

    struct Scene {
        int robots, obstacles, collisions;
    };
    std::printf("\nscene state message: %7s %10s %10s %12s %10s %7s\n", "robots", "obstacles", "parts",
                "ModelByName us", "cached us", "ratio");
    for (const Scene& scene : {Scene{7, 80, 32}, Scene{32, 200, 2}, Scene{100, 500, 2}}) {
        // Scene bodies load first; robots spawn after. FS150: 6 links with a
        // collision each and 5 joints.
        auto root = std::make_shared<Entity>();
        root->Add("ground_plane")->Add("link")->Add("collision");
        std::vector<std::string> names;
        for (int index = 0; index < scene.obstacles; ++index) {
            names.push_back("xgc2_obstacle_scene_tree_" + std::to_string(index));
            const auto body = root->Add(names.back())->Add("body");
            for (int part = 0; part < scene.collisions; ++part)
                body->Add("part_" + std::to_string(part));
        }
        for (int index = 1; index <= scene.robots; ++index) {
            const auto robot = root->Add("uav" + std::to_string(index), false);
            for (const char* link : {"base_link", "/imu_link", "rotor_0", "rotor_1", "rotor_2", "rotor_3"})
                robot->Add(link, false)->Add(std::string(link) + "_collision", false);
            for (const char* joint : {"/imu_joint", "rotor_0_joint", "rotor_1_joint", "rotor_2_joint", "rotor_3_joint"})
                robot->Add(joint, false);
        }
        const auto models_copy = [&root]() {
            return root->children;
        };
        const int messages = std::max(4, 4000 / scene.obstacles);
        std::size_t sink = 0;
        auto start = std::chrono::steady_clock::now();
        for (int message = 0; message < messages; ++message)
            for (int pass = 0; pass < 2; ++pass)
                for (const auto& name : names)
                    sink += root->GetByName(name) != nullptr ? 1 : 0;
        const double walk = Microseconds(start, messages);

        xgc2_gazebo_scene::ModelSnapshot<std::shared_ptr<Entity>, std::weak_ptr<Entity>> snapshot;
        std::map<std::string, std::shared_ptr<Entity>> lookup;
        const auto receive = [&]() {
            const auto models = models_copy();
            if (!snapshot.Matches(models)) {
                lookup.clear();
                snapshot.Update(models);
            }
            for (int pass = 0; pass < 2; ++pass)
                for (const auto& name : names) {
                    auto found = lookup.find(name);
                    if (found == lookup.end())
                        found = lookup.emplace(name, root->GetByName(name)).first;
                    sink += found->second != nullptr ? 1 : 0;
                }
        };
        // The first message after a model list change fills the cache with one
        // ModelByName per obstacle (half the old per-message cost); time the
        // messages after it.
        receive();
        const int cached_messages = 25 * messages;
        start = std::chrono::steady_clock::now();
        for (int message = 0; message < cached_messages; ++message)
            receive();
        const double cached = Microseconds(start, cached_messages);
        std::printf("%28d %10d %10d %12.1f %10.1f %6.0fx\n", scene.robots, scene.obstacles, scene.collisions, walk,
                    cached, walk / cached);
        if (sink == 1)
            std::puts("");
    }
    return 0;
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
