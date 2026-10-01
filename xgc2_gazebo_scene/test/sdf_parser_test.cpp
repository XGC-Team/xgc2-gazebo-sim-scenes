#include "obstacle_sdf.hpp"
#include "xgc2_gazebo_scene/model_sdf_parser.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <random>
#include <string>
#include <vector>

namespace xgc2_gazebo_scene {
namespace {

const char* const kRejected = "<rejected>";

std::size_t Count(const sdf::ElementPtr& parent, const std::string& name) {
    std::size_t count = 0;
    for (auto child = parent->GetElement(name); child; child = child->GetNextElement(name))
        ++count;
    return count;
}

// What the scene adapter did for every string before ModelSdfParser: a new
// sdf::SDF, sdf::init, sdf::readString.
std::string FreshParse(const std::string& sdf_string) {
    sdf::SDFPtr parsed(new sdf::SDF());
    sdf::init(parsed);
    if (!sdf::readString(sdf_string, parsed))
        return kRejected;
    return parsed->Root()->GetElement("model")->ToString("");
}

std::string ReusedParse(ModelSdfParser& parser, const std::string& sdf_string) {
    if (!parser.Parse(sdf_string))
        return kRejected;
    return parser.Model()->ToString("");
}

std::vector<std::string> Strings() {
    std::vector<std::string> strings;
    strings.reserve(20);
    for (int index = 0; index < 12; ++index)
        strings.emplace_back(ObstacleSdf(index, 1 + index % 5));
    strings.emplace_back(ObstacleSdf(99, 32));
    // Strings libsdformat rejects, and ones it only complains about.
    strings.emplace_back("");
    strings.emplace_back("<sdf version='1.6'><model name='broken'>");
    strings.emplace_back("<model name='no_sdf_root'/>");
    strings.emplace_back("<sdf version='1.6'><model><link name='body'/></model></sdf>");
    strings.emplace_back("<sdf version='1.6'><model name='m'><link name='a'/><link name='a'/></model></sdf>");
    strings.emplace_back(
        "<sdf version='1.6'><model name='m'><link name='body'><collision name='c'><geometry><torus/></geometry>"
        "</collision></link></model></sdf>");
    strings.emplace_back("<sdf version='1.4'><model name='old_version'><link name='body'/></model></sdf>");
    return strings;
}

TEST(ModelSdfParser, ParsesEveryStringLikeAFreshlyInitializedSdf) {
    const std::vector<std::string> strings = Strings();
    std::vector<std::string> expected;
    expected.reserve(strings.size());
    for (const auto& sdf_string : strings)
        expected.push_back(FreshParse(sdf_string));
    // The fixture must exercise both outcomes.
    int rejected = 0;
    for (const auto& result : expected)
        rejected += result == kRejected ? 1 : 0;
    ASSERT_GE(rejected, 4);
    ASSERT_LT(rejected, static_cast<int>(strings.size()));

    // One parser, in an order that puts rejected strings between accepted ones
    // and a large model before a small one.
    ModelSdfParser parser;
    std::mt19937 random(20261001);
    for (int round = 0; round < 400; ++round) {
        const std::size_t pick = std::uniform_int_distribution<std::size_t>(0, strings.size() - 1)(random);
        ASSERT_EQ(ReusedParse(parser, strings[pick]), expected[pick]) << "string " << pick << ", round " << round;
    }
}

TEST(ModelSdfParser, ForgetsTheStringBeforeIt) {
    ModelSdfParser parser;
    ASSERT_TRUE(parser.Parse(ObstacleSdf(1, 8)));
    EXPECT_EQ(Count(parser.Model()->GetElement("link"), "collision"), 8u);
    ASSERT_TRUE(parser.Parse(ObstacleSdf(2, 2)));
    EXPECT_EQ(Count(parser.Model()->GetElement("link"), "collision"), 2u);
    EXPECT_EQ(parser.Model()->Get<std::string>("name"), "xgc2_obstacle_scene_2");
}

TEST(ModelSdfParser, HasNoModelAfterARejectedString) {
    ModelSdfParser parser;
    EXPECT_EQ(parser.Model(), nullptr) << "before any string";
    ASSERT_TRUE(parser.Parse(ObstacleSdf(1, 1)));
    ASSERT_NE(parser.Model(), nullptr);
    EXPECT_FALSE(parser.Parse("<sdf version='1.6'><model name='broken'>"));
    EXPECT_EQ(parser.Model(), nullptr);
    EXPECT_TRUE(parser.Parse(ObstacleSdf(3, 1)));
    ASSERT_NE(parser.Model(), nullptr);
    EXPECT_EQ(parser.Model()->Get<std::string>("name"), "xgc2_obstacle_scene_3");
}

} // namespace
} // namespace xgc2_gazebo_scene

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
