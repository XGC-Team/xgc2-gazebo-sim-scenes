#include "xgc2_gazebo_scene/physical_contact_filter.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace xgc2_gazebo_scene {
namespace {

ContactModelDescriptor Model(const std::string& name, bool is_static, bool is_managed_obstacle = false) {
    ContactModelDescriptor result;
    result.name = name;
    result.is_static = is_static;
    result.is_managed_obstacle = is_managed_obstacle;
    return result;
}

TEST(PhysicalContactFilter, EmptyPrefixTracksAllDynamicActors) {
    const std::vector<std::string> prefixes;
    EXPECT_TRUE(IsForbiddenPhysicalContact(Model("ugv1", false), Model("xgc2_obstacle_post", true, true), prefixes));
    EXPECT_TRUE(IsForbiddenPhysicalContact(Model("ugv1", false), Model("ugv2", false), prefixes));
}

TEST(PhysicalContactFilter, AllowsGroundAndSelfContacts) {
    const std::vector<std::string> prefixes;
    EXPECT_FALSE(IsForbiddenPhysicalContact(Model("ugv1", false), Model("ground_plane", true), prefixes));
    EXPECT_FALSE(IsForbiddenPhysicalContact(Model("ugv1", false), Model("ugv1", false), prefixes));
}

TEST(PhysicalContactFilter, PrefixesLimitTrackedActorsWithoutHardCodedInstances) {
    const std::vector<std::string> prefixes{"uav"};
    EXPECT_TRUE(IsForbiddenPhysicalContact(Model("uav6", false), Model("xgc2_obstacle_knot", true, true), prefixes));
    EXPECT_TRUE(IsForbiddenPhysicalContact(Model("uav1", false), Model("uav2", false), prefixes));
    EXPECT_FALSE(IsForbiddenPhysicalContact(Model("scout1", false), Model("xgc2_obstacle_knot", true, true), prefixes));
    EXPECT_FALSE(IsForbiddenPhysicalContact(Model("uav1", false), Model("scout1", false), prefixes));
}

TEST(PhysicalContactFilter, ManagedMovingObstaclesAreNeverMisclassifiedAsRobots) {
    const std::vector<std::string> prefixes;
    EXPECT_FALSE(IsForbiddenPhysicalContact(Model("xgc2_obstacle_a", false, true),
                                            Model("xgc2_obstacle_b", false, true), prefixes));
}

TEST(PhysicalContactFilter, EmptyConfiguredPrefixDoesNotMatchEverything) {
    const std::vector<std::string> prefixes{""};
    EXPECT_FALSE(MatchesTrackedPrefix("uav1", prefixes));
}

} // namespace
} // namespace xgc2_gazebo_scene

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
