#include "xgc2_gazebo_scene/convex_mesh_geometry.hpp"

#include <gazebo/common/SystemPaths.hh>
#include <gtest/gtest.h>

#include <array>
#include <string>

namespace xgc2_gazebo_scene {
namespace {

bool ContainsVertex(const ConvexMeshGeometry& geometry, const ignition::math::Vector3d& expected) {
    for (const auto& vertex : geometry.vertices) {
        // Gazebo's Assimp path stores OBJ coordinates as floats internally.
        if (vertex.Equal(expected, 1.0e-6)) {
            return true;
        }
    }
    return false;
}

TEST(ConvexMeshGeometryTest, LoadsExactScaledIcosahedron) {
    gazebo::common::SystemPaths::Instance()->AddModelPaths(XGC2_SCENE_TEST_MODEL_PATH);
    ConvexMeshGeometry geometry;
    std::string error;
    ASSERT_TRUE(LoadConvexMeshGeometry("model://xgc2_convex_icosahedron_1/meshes/icosahedron_1.obj", {2.0, 0.5, 3.0},
                                       "", false, &geometry, &error))
        << error;
    EXPECT_EQ(12u, geometry.vertices.size());
    EXPECT_EQ(20u, geometry.triangles.size());
    EXPECT_TRUE(ContainsVertex(geometry, {0.0, 0.19, 1.845}));
    EXPECT_TRUE(ContainsVertex(geometry, {-1.23, 0.0, -1.14}));
}

TEST(ConvexMeshGeometryTest, LoadsExactHexagonalPrism) {
    gazebo::common::SystemPaths::Instance()->AddModelPaths(XGC2_SCENE_TEST_MODEL_PATH);
    ConvexMeshGeometry geometry;
    std::string error;
    ASSERT_TRUE(LoadConvexMeshGeometry("model://xgc2_convex_ugv_hexagon_1/meshes/ugv_hexagon_1.obj", {0.9, 0.9, 1.6},
                                       "", false, &geometry, &error))
        << error;
    EXPECT_EQ(12u, geometry.vertices.size());
    EXPECT_EQ(20u, geometry.triangles.size());
    EXPECT_TRUE(ContainsVertex(geometry, {0.2104443, 0.1215, -0.48}));
}

TEST(ConvexMeshGeometryTest, RejectsOpenMesh) {
    ConvexMeshGeometry geometry;
    geometry.vertices = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    geometry.triangles = {{{0, 1, 2}}, {{0, 3, 1}}, {{1, 3, 2}}};
    std::string error;
    EXPECT_FALSE(ValidateClosedConvexMesh(geometry, &error));
    EXPECT_FALSE(error.empty());
}

TEST(ConvexMeshGeometryTest, RejectsInvalidScale) {
    ConvexMeshGeometry geometry;
    std::string error;
    EXPECT_FALSE(LoadConvexMeshGeometry("model://xgc2_convex_icosahedron_1/meshes/icosahedron_1.obj", {1.0, 0.0, 1.0},
                                        "", false, &geometry, &error));
    EXPECT_NE(std::string::npos, error.find("scale"));
}

} // namespace
} // namespace xgc2_gazebo_scene

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
