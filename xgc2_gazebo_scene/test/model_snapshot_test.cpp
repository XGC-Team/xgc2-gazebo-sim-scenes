#include "xgc2_gazebo_scene/model_snapshot.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <new>
#include <string>
#include <utility>
#include <vector>

namespace xgc2_gazebo_scene {
namespace {

struct FakeModel {
    std::string name;
    bool is_static = false;
    std::string GetName() const { return name; }
    bool IsStatic() const { return is_static; }
};

using FakePtr = std::shared_ptr<FakeModel>;
using Snapshot = ModelSnapshot<FakePtr, std::weak_ptr<FakeModel>>;

FakePtr Make(const std::string& name, bool is_static) {
    return std::make_shared<FakeModel>(FakeModel{name, is_static});
}

std::vector<FakePtr> World() {
    std::vector<FakePtr> models;
    models.reserve(5);
    models.push_back(Make("ground_plane", true));
    models.push_back(Make("xgc2_obstacle_scene_1", true));
    models.push_back(Make("xgc2_obstacle_scene_2", true));
    models.push_back(Make("uav1", false));
    models.push_back(Make("uav2", false));
    return models;
}

TEST(ModelSnapshot, AnEmptySnapshotMatchesOnlyAnEmptyWorld) {
    const Snapshot snapshot;
    EXPECT_TRUE(snapshot.Matches({}));
    EXPECT_FALSE(snapshot.Matches(World()));
}

TEST(ModelSnapshot, AnUnchangedListMatches) {
    const auto models = World();
    Snapshot snapshot;
    snapshot.Update(models);
    EXPECT_TRUE(snapshot.Matches(models));
    // World::Models() returns a fresh copy of the same pointers every update.
    EXPECT_TRUE(snapshot.Matches(std::vector<FakePtr>(models.begin(), models.end())));
}

TEST(ModelSnapshot, EveryChangeTheSceneStateDependsOnIsSeen) {
    auto models = World();
    Snapshot snapshot;
    snapshot.Update(models);

    auto added = models;
    added.push_back(Make("uav3", false));
    EXPECT_FALSE(snapshot.Matches(added));

    auto removed = models;
    removed.pop_back();
    EXPECT_FALSE(snapshot.Matches(removed));

    auto reordered = models;
    std::swap(reordered[1], reordered[2]);
    EXPECT_FALSE(snapshot.Matches(reordered));

    // A respawned robot: same name and flag, new object.
    auto respawned = models;
    respawned[3] = Make("uav1", false);
    EXPECT_FALSE(snapshot.Matches(respawned));

    // The scene authoring plugin renames a body before removing it.
    models[1]->name = "xgc2_retired_scene_7";
    EXPECT_FALSE(snapshot.Matches(models));
    models[1]->name = "xgc2_obstacle_scene_1";
    EXPECT_TRUE(snapshot.Matches(models));

    models[4]->is_static = true;
    EXPECT_FALSE(snapshot.Matches(models));
    snapshot.Update(models);
    EXPECT_TRUE(snapshot.Matches(models));
}

TEST(ModelSnapshot, AModelReplacedAtTheSameAddressIsSeen) {
    alignas(FakeModel) unsigned char storage[sizeof(FakeModel)];
    const auto at = [&storage](const std::string& name) {
        return FakePtr(new (storage) FakeModel{name, false}, [](FakeModel* model) {
            model->~FakeModel();
        });
    };
    auto models = World();
    models.push_back(at("uav3"));
    Snapshot snapshot;
    snapshot.Update(models);
    const void* address = models.back().get();
    // Delete the model; a new one with the same name lands at the same address.
    models.back().reset();
    models.back() = at("uav3");
    ASSERT_EQ(models.back().get(), address);
    EXPECT_FALSE(snapshot.Matches(models));
    snapshot.Update(models);
    EXPECT_TRUE(snapshot.Matches(models));
}

} // namespace
} // namespace xgc2_gazebo_scene

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
