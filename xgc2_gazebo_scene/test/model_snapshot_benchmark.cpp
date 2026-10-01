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
// node's scoped name and name (Gazebo 11 Base::GetByName). "index" is one
// list copy, ModelIndex::Refresh (ModelSnapshot::Matches) and two hash
// lookups per obstacle.
//
// SceneAuthoringWorldPlugin::Apply, only the model lookups of its steps and
// its 10 ms wait loops. Loading N obstacles: three loops over all of them
// before the insert (owned elsewhere? current pose? insert), then one pass per
// wait period while the factory loads them, each walking the obstacles in
// order until the first that is not there yet, then the confirming pass.
// Replacing N obstacles: every pass asks whether the N old names are gone
// (they are, at once: retirement renames) and the N retired names are gone
// (they go in order). "ModelByName" walks the tree for each name, as 1.4.1-1
// did; "index" is a Refresh and hash lookups per pass.
//
// Names come from Base::GetName/GetScopedName (strings returned by value).
// The tree is compact and warm in the cache; Gazebo's nodes are scattered heap
// objects, and World::ModelByName also holds the model-loading mutex for each
// walk, which keeps the factory waiting. Neither is modelled. Build and run:
// test/run_model_snapshot_benchmark.sh (any C++17 compiler).
#include "xgc2_gazebo_scene/model_index.hpp"
#include "xgc2_gazebo_scene/model_snapshot.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <map>
#include <memory>
#include <set>
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

void AddRobots(const std::shared_ptr<Entity>& root, int robots) {
    // FS150: 6 links with a collision each and 5 joints.
    for (int index = 1; index <= robots; ++index) {
        const auto robot = root->Add("uav" + std::to_string(index), false);
        for (const char* link : {"base_link", "/imu_link", "rotor_0", "rotor_1", "rotor_2", "rotor_3"})
            robot->Add(link, false)->Add(std::string(link) + "_collision", false);
        for (const char* joint : {"/imu_joint", "rotor_0_joint", "rotor_1_joint", "rotor_2_joint", "rotor_3_joint"})
            robot->Add(joint, false);
    }
}

std::shared_ptr<Entity> RobotsWorld(int robots) {
    auto root = std::make_shared<Entity>();
    root->Add("ground_plane")->Add("link")->Add("collision");
    AddRobots(root, robots);
    return root;
}

std::string ObstacleName(int index) {
    return "xgc2_obstacle_scene_" + std::to_string(index);
}

std::string RetiredName(int index) {
    return "xgc2_retired_scene_" + std::to_string(index);
}

void AddObstacles(const std::shared_ptr<Entity>& root, int first, int last) {
    for (int index = first; index < last; ++index) {
        const auto body = root->Add(ObstacleName(index))->Add("body");
        body->Add("part_0");
        body->Add("part_1");
    }
}

using Index = xgc2_gazebo_scene::ModelIndex<std::shared_ptr<Entity>, std::weak_ptr<Entity>>;

struct ApplyCost {
    std::size_t lookups = 0;
    std::size_t refreshes = 0;
    std::size_t walks = 0; // ModelByName walks the index flow still makes
    std::size_t sink = 0;  // keeps the lookups from being optimized away
    double milliseconds = 0.0;
};

// Times a callable and adds its cost.
struct Meter {
    ApplyCost cost;
    std::size_t& sink = cost.sink;
    template <class Function> void Time(Function&& function) {
        const auto start = std::chrono::steady_clock::now();
        function();
        cost.milliseconds +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    }
};

// What each wait pass of Apply does with the world once the factory has
// loaded `present` obstacles: the predicate asks for the obstacles in order
// and stops at the first that is not there.
template <class Lookup> void LoadPass(Meter& meter, int obstacles, Lookup&& lookup) {
    for (int index = 0; index < obstacles; ++index) {
        ++meter.cost.lookups;
        if (!lookup(ObstacleName(index)))
            break;
    }
}

ApplyCost LoadByWalking(int robots, int obstacles, int passes) {
    const auto root = RobotsWorld(robots);
    Meter meter;
    const auto walk = [&](const std::string& name) {
        return root->GetByName(name) != nullptr;
    };
    meter.Time([&] {
        for (int loop = 0; loop < 3; ++loop) // owned elsewhere? current pose? insert
            for (int index = 0; index < obstacles; ++index) {
                ++meter.cost.lookups;
                meter.sink += walk(ObstacleName(index)) ? 1 : 0;
            }
    });
    int loaded = 0;
    for (int pass = 1; pass <= passes; ++pass) {
        const int present = static_cast<int>(static_cast<long long>(obstacles) * pass / passes);
        AddObstacles(root, loaded, present);
        loaded = present;
        meter.Time([&] {
            LoadPass(meter, obstacles, walk);
        });
    }
    return meter.cost;
}

ApplyCost LoadByIndex(int robots, int obstacles, int passes) {
    const auto root = RobotsWorld(robots);
    Meter meter;
    Index index;
    const auto find = [&](const std::string& name) {
        return index.Find(name) != nullptr;
    };
    const auto refresh = [&] {
        ++meter.cost.refreshes;
        index.Refresh(root->children);
    };
    meter.Time([&] {
        refresh();
        for (int loop = 0; loop < 3; ++loop)
            for (int each = 0; each < obstacles; ++each) {
                ++meter.cost.lookups;
                meter.sink += find(ObstacleName(each)) ? 1 : 0;
            }
    });
    int loaded = 0;
    for (int pass = 1; pass <= passes; ++pass) {
        const int present = static_cast<int>(static_cast<long long>(obstacles) * pass / passes);
        AddObstacles(root, loaded, present);
        loaded = present;
        meter.Time([&] {
            refresh();
            LoadPass(meter, obstacles, find);
        });
    }
    return meter.cost;
}

// Retiring renames every old obstacle at once, then Gazebo deletes the retired
// bodies in order, `obstacles / passes` of them between two passes.
void RetireObstacle(const std::shared_ptr<Entity>& root, int index) {
    for (auto& child : root->children)
        if (child->name == ObstacleName(index)) {
            child->name = child->scoped_name = RetiredName(index);
            return;
        }
}

void DeleteRetired(const std::shared_ptr<Entity>& root, int first, int last) {
    for (int index = first; index < last; ++index) {
        auto& children = root->children;
        for (auto child = children.begin(); child != children.end(); ++child)
            if ((*child)->name == RetiredName(index)) {
                children.erase(child);
                break;
            }
    }
}

// SceneBodiesGone: true when none of the names is in the world.
template <class Name, class Lookup> bool Gone(Meter& meter, int count, Name&& name, Lookup&& lookup) {
    for (int index = 0; index < count; ++index) {
        ++meter.cost.lookups;
        if (lookup(name(index)))
            return false;
    }
    return true;
}

template <class Setup, class Pass>
ApplyCost Replace(int robots, int obstacles, int passes, Setup&& setup, Pass&& pass) {
    const auto root = RobotsWorld(robots);
    AddObstacles(root, 0, obstacles);
    Meter meter;
    setup(meter, root);
    for (int index = 0; index < obstacles; ++index)
        RetireObstacle(root, index);
    int deleted = 0;
    for (int period = 1; period <= passes; ++period) {
        const int gone = static_cast<int>(static_cast<long long>(obstacles) * period / passes);
        DeleteRetired(root, deleted, gone);
        deleted = gone;
        pass(meter, root);
    }
    return meter.cost;
}

ApplyCost ReplaceByWalking(int robots, int obstacles, int passes) {
    return Replace(
        robots, obstacles, passes,
        [&](Meter& meter, const std::shared_ptr<Entity>& root) {
            meter.Time([&] {
                for (int index = 0; index < obstacles; ++index) {
                    ++meter.cost.lookups;
                    meter.sink += root->GetByName(ObstacleName(index)) != nullptr ? 1 : 0;
                }
            });
        },
        [&](Meter& meter, const std::shared_ptr<Entity>& root) {
            const auto walk = [&](const std::string& name) {
                return root->GetByName(name) != nullptr;
            };
            meter.Time([&] {
                // SceneBodiesGone(remove) && SceneBodiesGone(retiring) && !HasRetiredSceneModels()
                if (Gone(meter, obstacles, ObstacleName, walk) && Gone(meter, obstacles, RetiredName, walk)) {
                    for (const auto& model : root->children)
                        meter.sink += model->GetName().compare(0, 19, "xgc2_retired_scene_") == 0 ? 1 : 0;
                }
            });
        });
}

ApplyCost ReplaceByIndex(int robots, int obstacles, int passes) {
    Index index;
    const auto find = [&](const std::string& name) {
        return index.Find(name) != nullptr;
    };
    std::vector<std::string> retired;
    retired.reserve(obstacles);
    for (int each = 0; each < obstacles; ++each)
        retired.push_back(RetiredName(each));
    std::set<std::string> confirmed;
    return Replace(
        robots, obstacles, passes,
        [&](Meter& meter, const std::shared_ptr<Entity>& root) {
            meter.Time([&] {
                ++meter.cost.refreshes;
                index.Refresh(root->children);
                for (int each = 0; each < obstacles; ++each) {
                    ++meter.cost.lookups;
                    meter.sink += find(ObstacleName(each)) ? 1 : 0;
                }
            });
        },
        [&](Meter& meter, const std::shared_ptr<Entity>& root) {
            // The retired bodies are confirmed with ModelByName, once each, when the index has dropped them.
            const auto walk = [&](const std::string& name) {
                ++meter.cost.walks;
                return root->GetByName(name) != nullptr;
            };
            meter.Time([&] {
                ++meter.cost.refreshes;
                index.Refresh(root->children);
                if (Gone(meter, obstacles, ObstacleName, find) &&
                    xgc2_gazebo_scene::AllGone(index, retired, walk, &confirmed)) {
                    for (const auto& model : root->children)
                        meter.sink += model->GetName().compare(0, 19, "xgc2_retired_scene_") == 0 ? 1 : 0;
                }
            });
        });
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
    std::printf("\nscene state message: %7s %10s %10s %14s %10s %7s %14s\n", "robots", "obstacles", "parts",
                "ModelByName us", "index us", "ratio", "after change us");
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
        AddRobots(root, scene.robots);
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

        xgc2_gazebo_scene::ModelIndex<std::shared_ptr<Entity>, std::weak_ptr<Entity>> index;
        const auto receive = [&]() {
            index.Refresh(models_copy());
            for (int pass = 0; pass < 2; ++pass)
                for (const auto& name : names)
                    sink += index.Find(name) != nullptr ? 1 : 0;
        };
        // The first message after a model list change rebuilds the index; time
        // the messages after it, then a message after every change.
        receive();
        const int cached_messages = 25 * messages;
        start = std::chrono::steady_clock::now();
        for (int message = 0; message < cached_messages; ++message)
            receive();
        const double cached = Microseconds(start, cached_messages);
        std::vector<xgc2_gazebo_scene::ModelIndex<std::shared_ptr<Entity>, std::weak_ptr<Entity>>> fresh(messages);
        start = std::chrono::steady_clock::now();
        for (auto& each : fresh) {
            each.Refresh(models_copy());
            for (int pass = 0; pass < 2; ++pass)
                for (const auto& name : names)
                    sink += each.Find(name) != nullptr ? 1 : 0;
        }
        const double after_change = Microseconds(start, messages);
        std::printf("%28d %10d %10d %14.1f %10.1f %6.0fx %14.1f\n", scene.robots, scene.obstacles, scene.collisions,
                    walk, cached, walk / cached, after_change);
        if (sink == 1)
            std::puts("");
    }

    struct Apply {
        int robots, obstacles, passes;
    };
    const std::vector<Apply> applies{Apply{7, 80, 10}, Apply{32, 200, 20}, Apply{100, 500, 25}};
    std::printf("\napply, load N obstacles: %7s %10s %7s | %9s %14s | %9s %9s %6s %9s | %7s\n", "robots", "obstacles",
                "passes", "walks", "ModelByName ms", "finds", "refreshes", "walks", "index ms", "ratio");
    for (const Apply& apply : applies) {
        const ApplyCost walked = LoadByWalking(apply.robots, apply.obstacles, apply.passes);
        const ApplyCost indexed = LoadByIndex(apply.robots, apply.obstacles, apply.passes);
        std::printf("%32d %10d %7d | %9zu %14.2f | %9zu %9zu %6zu %9.3f | %6.0fx\n", apply.robots, apply.obstacles,
                    apply.passes, walked.lookups, walked.milliseconds, indexed.lookups, indexed.refreshes,
                    indexed.walks, indexed.milliseconds, walked.milliseconds / indexed.milliseconds);
        if (walked.sink + indexed.sink == 1)
            std::puts("");
    }
    std::printf("\napply, replace N obstacles: %5s %10s %7s | %9s %14s | %9s %9s %6s %9s | %7s\n", "robots",
                "obstacles", "passes", "walks", "ModelByName ms", "finds", "refreshes", "walks", "index ms", "ratio");
    for (const Apply& apply : applies) {
        const ApplyCost walked = ReplaceByWalking(apply.robots, apply.obstacles, apply.passes);
        const ApplyCost indexed = ReplaceByIndex(apply.robots, apply.obstacles, apply.passes);
        std::printf("%32d %10d %7d | %9zu %14.2f | %9zu %9zu %6zu %9.3f | %6.0fx\n", apply.robots, apply.obstacles,
                    apply.passes, walked.lookups, walked.milliseconds, indexed.lookups, indexed.refreshes,
                    indexed.walks, indexed.milliseconds, walked.milliseconds / indexed.milliseconds);
        if (walked.sink + indexed.sink == 1)
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
