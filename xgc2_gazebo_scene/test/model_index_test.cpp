#include "xgc2_gazebo_scene/model_index.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace xgc2_gazebo_scene {
namespace {

// A Gazebo 11 entity: World::ModelByName is Base::GetByName on the root, a
// depth-first walk that matches the name or scoped name of any kind of entity,
// then keeps the answer if it is a model.
struct Entity : std::enable_shared_from_this<Entity> {
    std::string name;
    bool is_model = false;
    bool is_static = true;
    std::vector<std::shared_ptr<Entity>> children;
    std::string GetName() const { return name; }
    bool IsStatic() const { return is_static; }
    std::shared_ptr<Entity> GetByName(const std::string& wanted) {
        if (name == wanted)
            return shared_from_this();
        for (const auto& child : children)
            if (auto found = child->GetByName(wanted))
                return found;
        return nullptr;
    }
};

using EntityPtr = std::shared_ptr<Entity>;
using Index = ModelIndex<EntityPtr, std::weak_ptr<Entity>>;

struct World {
    EntityPtr root = std::make_shared<Entity>();
    std::vector<EntityPtr> listed; // World::Models(): the models at the root, in load order
    std::size_t walks = 0;

    EntityPtr Add(const std::string& name, bool is_static = true) {
        auto model = std::make_shared<Entity>();
        model->name = name;
        model->is_model = true;
        model->is_static = is_static;
        auto link = std::make_shared<Entity>();
        link->name = "link_of_" + name;
        auto collision = std::make_shared<Entity>();
        collision->name = "collision_of_" + name;
        link->children.push_back(collision);
        model->children.push_back(link);
        root->children.push_back(model);
        listed.push_back(model);
        return model;
    }
    // World::RemoveModel does these two things in this order, one after the
    // other under the physics update mutex.
    void TakeOffTheList(const EntityPtr& model) {
        for (auto entry = listed.begin(); entry != listed.end(); ++entry)
            if (*entry == model) {
                listed.erase(entry);
                return;
            }
    }
    void DestroyEntities(const EntityPtr& model) {
        for (auto child = root->children.begin(); child != root->children.end(); ++child)
            if (*child == model) {
                root->children.erase(child);
                return;
            }
    }
    void Remove(const EntityPtr& model) {
        TakeOffTheList(model);
        DestroyEntities(model);
    }
    std::vector<EntityPtr> Models() const { return listed; }
    EntityPtr ModelByName(const std::string& name) {
        ++walks;
        const auto found = root->GetByName(name);
        return found && found->is_model ? found : nullptr;
    }
};

TEST(ModelIndex, AnEmptyIndexFindsNothing) {
    const Index index;
    EXPECT_EQ(index.Find("anything"), nullptr);
    Index refreshed;
    EXPECT_FALSE(refreshed.Refresh({}));
    EXPECT_EQ(refreshed.Find(""), nullptr);
}

TEST(ModelIndex, FindsEveryModelByItsName) {
    World world;
    const auto first = world.Add("xgc2_obstacle_scene_1");
    const auto second = world.Add("uav1", false);
    Index index;
    EXPECT_TRUE(index.Refresh(world.Models()));
    EXPECT_EQ(index.Find("xgc2_obstacle_scene_1"), first);
    EXPECT_EQ(index.Find("uav1"), second);
    EXPECT_EQ(index.Find("uav2"), nullptr);
    EXPECT_EQ(index.Find("link_of_uav1"), nullptr) << "only models are indexed";
}

TEST(ModelIndex, IsRebuiltOnlyWhenTheListChanges) {
    World world;
    const auto kept = world.Add("xgc2_obstacle_scene_1");
    world.Add("xgc2_obstacle_scene_2");
    Index index;
    ASSERT_TRUE(index.Refresh(world.Models()));
    // World::Models() returns a fresh copy of the same pointers every time.
    EXPECT_FALSE(index.Refresh(world.Models()));
    EXPECT_FALSE(index.Refresh(world.Models()));

    const auto added = world.Add("xgc2_obstacle_scene_3");
    EXPECT_EQ(index.Find("xgc2_obstacle_scene_3"), nullptr) << "answers are those of the last Refresh";
    EXPECT_TRUE(index.Refresh(world.Models()));
    EXPECT_EQ(index.Find("xgc2_obstacle_scene_3"), added);
    EXPECT_EQ(index.Find("xgc2_obstacle_scene_1"), kept);

    world.Remove(added);
    EXPECT_TRUE(index.Refresh(world.Models()));
    EXPECT_EQ(index.Find("xgc2_obstacle_scene_3"), nullptr);

    kept->is_static = false;
    EXPECT_TRUE(index.Refresh(world.Models()));
    EXPECT_FALSE(index.Refresh(world.Models()));
}

TEST(ModelIndex, RetirementRenamesBeforeRemoving) {
    World world;
    const auto model = world.Add("xgc2_obstacle_scene_1");
    Index index;
    index.Refresh(world.Models());
    // The scene adapter renames a body, then asks Gazebo to delete it.
    model->name = "xgc2_retired_scene_7";
    EXPECT_TRUE(index.Refresh(world.Models()));
    EXPECT_EQ(index.Find("xgc2_obstacle_scene_1"), nullptr) << "the original name is free at once";
    EXPECT_EQ(index.Find("xgc2_retired_scene_7"), model);
    world.Remove(model);
    EXPECT_TRUE(index.Refresh(world.Models()));
    EXPECT_EQ(index.Find("xgc2_retired_scene_7"), nullptr);
}

TEST(ModelIndex, ARespawnedModelIsTheNewOne) {
    World world;
    const auto old_model = world.Add("uav1", false);
    Index index;
    index.Refresh(world.Models());
    world.Remove(old_model);
    const auto new_model = world.Add("uav1", false);
    ASSERT_NE(old_model, new_model);
    EXPECT_TRUE(index.Refresh(world.Models()));
    EXPECT_EQ(index.Find("uav1"), new_model);
}

TEST(ModelIndex, HoldsModelsWeakly) {
    World world;
    auto model = world.Add("xgc2_obstacle_scene_1");
    Index index;
    index.Refresh(world.Models());
    const long references = model.use_count();
    EXPECT_EQ(index.Find("xgc2_obstacle_scene_1"), model);
    EXPECT_EQ(model.use_count(), references) << "the index keeps no reference";
    // The world drops the model: it is gone for Find() even before the next Refresh().
    world.Remove(model);
    model.reset();
    EXPECT_EQ(index.Find("xgc2_obstacle_scene_1"), nullptr);
    EXPECT_TRUE(index.Refresh(world.Models()));
}

TEST(ModelIndex, ANameListedTwiceAnswersWithTheFirst) {
    World world;
    const auto first = world.Add("twin");
    world.Add("twin");
    Index index;
    index.Refresh(world.Models());
    EXPECT_EQ(index.Find("twin"), first);
    EXPECT_EQ(world.ModelByName("twin"), first);
}

// The one place the index differs from ModelByName, which is why the adapter's
// lookups carry reserved prefixes: a link named like a model hides the model
// from the walk, because the walk stops at the first entity of any kind.
TEST(ModelIndex, ANameSharedWithAnotherKindOfEntityAnswersWithTheModel) {
    World world;
    const auto robot = world.Add("robot");
    robot->children.front()->name = "xgc2_obstacle_scene_1";
    const auto scene = world.Add("xgc2_obstacle_scene_1");
    Index index;
    index.Refresh(world.Models());
    EXPECT_EQ(world.ModelByName("xgc2_obstacle_scene_1"), nullptr);
    EXPECT_EQ(index.Find("xgc2_obstacle_scene_1"), scene);
}

// Gazebo drops a model from the model list and then destroys its entities. The
// index follows the list, so it forgets the model first; a body only counts as
// removed when ModelByName, which sees the entity tree, agrees.
TEST(ModelIndex, ARetiredBodyIsGoneOnlyWhenItsEntitiesAre) {
    World world;
    const auto body = world.Add("xgc2_retired_scene_7");
    world.Add("xgc2_retired_scene_8");
    Index index;
    index.Refresh(world.Models());
    const auto walk = [&world](const std::string& name) {
        return world.ModelByName(name) != nullptr;
    };
    std::set<std::string> confirmed;
    const std::set<std::string> retiring{"xgc2_retired_scene_7"};

    EXPECT_FALSE(AllGone(index, retiring, walk, &confirmed)) << "still listed";
    world.TakeOffTheList(body);
    index.Refresh(world.Models());
    EXPECT_EQ(index.Find("xgc2_retired_scene_7"), nullptr) << "the index has forgotten it";
    EXPECT_FALSE(AllGone(index, retiring, walk, &confirmed)) << "its entities are still in the world";
    EXPECT_TRUE(confirmed.empty());
    world.DestroyEntities(body);
    EXPECT_TRUE(AllGone(index, retiring, walk, &confirmed));
    EXPECT_EQ(confirmed, retiring);
}

TEST(ModelIndex, AGoneBodyIsConfirmedOnce) {
    World world;
    const auto first = world.Add("xgc2_retired_scene_1");
    const auto second = world.Add("xgc2_retired_scene_2");
    for (int index = 0; index < 20; ++index)
        world.Add("uav" + std::to_string(index), false);
    world.Remove(first);
    world.Remove(second);
    Index index;
    index.Refresh(world.Models());
    const auto walk = [&world](const std::string& name) {
        return world.ModelByName(name) != nullptr;
    };
    std::set<std::string> confirmed;
    const std::set<std::string> retiring{"xgc2_retired_scene_1", "xgc2_retired_scene_2"};
    world.walks = 0;
    for (int pass = 0; pass < 50; ++pass)
        EXPECT_TRUE(AllGone(index, retiring, walk, &confirmed));
    EXPECT_EQ(world.walks, 2u) << "one walk per name, however many passes";
}

TEST(ModelIndex, APresentBodyIsNeverWalkedFor) {
    World world;
    world.Add("xgc2_retired_scene_1");
    Index index;
    index.Refresh(world.Models());
    std::set<std::string> confirmed;
    world.walks = 0;
    for (int pass = 0; pass < 50; ++pass)
        EXPECT_FALSE(AllGone(
            index, std::set<std::string>{"xgc2_retired_scene_1"},
            [&world](const std::string& name) {
                return world.ModelByName(name) != nullptr;
            },
            &confirmed));
    EXPECT_EQ(world.walks, 0u);
}

// Whatever happens to the world, once Refresh() has followed it the index
// answers as World::ModelByName does, for present and absent names alike.
TEST(ModelIndex, AgreesWithModelByNameThroughRandomWorldChanges) {
    std::mt19937 random(20261001);
    World world;
    Index index;
    std::vector<std::string> universe;
    universe.reserve(40);
    for (int name = 0; name < 40; ++name)
        universe.push_back((name % 3 == 0 ? "xgc2_retired_scene_" : "xgc2_obstacle_scene_") + std::to_string(name));
    const auto pick = [&random](std::size_t count) {
        return std::uniform_int_distribution<std::size_t>(0, count - 1)(random);
    };
    constexpr int kSteps = 1500;
    int rebuilds = 0;
    int answers = 0;
    for (int step = 0; step < kSteps; ++step) {
        auto models = world.Models();
        const std::size_t operation = pick(10);
        if (operation < 3) { // a model appears
            world.Add(universe[pick(universe.size())], pick(2) == 0);
        } else if (operation < 6) { // a model is removed
            if (!models.empty())
                world.Remove(models[pick(models.size())]);
        } else if (operation < 8) { // a model is renamed, as retirement does
            if (!models.empty())
                models[pick(models.size())]->name = universe[pick(universe.size())];
        } else if (!models.empty()) { // a model changes between static and dynamic
            models[pick(models.size())]->is_static ^= true;
        }
        // Names are unique at the root of a real world. A duplicate, made here
        // by adding or renaming onto an existing name, answers with the first
        // model in both.
        rebuilds += index.Refresh(world.Models()) ? 1 : 0;
        for (const auto& name : universe) {
            ASSERT_EQ(index.Find(name), world.ModelByName(name)) << name << " at step " << step;
            ++answers;
        }
    }
    EXPECT_GT(rebuilds, kSteps / 2);
    EXPECT_EQ(answers, kSteps * static_cast<int>(universe.size()));
}

} // namespace
} // namespace xgc2_gazebo_scene

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
