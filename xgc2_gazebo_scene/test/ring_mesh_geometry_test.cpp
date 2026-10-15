#include "xgc2_gazebo_scene/ring_mesh_geometry.hpp"
#include "xgc2_gazebo_scene/scene_model.hpp"

#include <gazebo/common/Mesh.hh>
#include <gazebo/common/MeshManager.hh>
#include <gtest/gtest.h>
#include <ode/ode.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <map>
#include <set>
#include <unistd.h>

namespace xgc2_gazebo_scene {
namespace {
using Point = std::array<double, 3>;

// Independent numerical oracle, including all stationary-point basins. Not a
// normal-sweep assumption: valid on and inside the tight-curvature envelope.
double EllipseDistance(const Point& p, double a, double b) {
    constexpr double pi = 3.14159265358979323846;
    const auto distance2 = [&](double u) {
        const double x = p[0] - a * std::cos(u), y = p[1] - b * std::sin(u);
        return x * x + y * y + p[2] * p[2];
    };
    double best = std::numeric_limits<double>::infinity();
    for (unsigned i = 0; i < 64; ++i) {
        const double u = 2 * pi * i / 64;
        if (distance2(u) > distance2(u - 2 * pi / 64) || distance2(u) > distance2(u + 2 * pi / 64))
            continue;
        double lo = u - 2 * pi / 64, hi = u + 2 * pi / 64;
        for (unsigned j = 0; j < 60; ++j) {
            const double x = lo + (hi - lo) / 3, y = hi - (hi - lo) / 3;
            if (distance2(x) < distance2(y))
                hi = y;
            else
                lo = x;
        }
        best = std::min(best, distance2((lo + hi) / 2));
    }
    return std::sqrt(best);
}

void CheckMesh(double a, double b, double r) {
    const auto mesh = MakeRingMesh(a, b, r);
    ASSERT_FALSE(mesh.triangles.empty());
    std::map<std::pair<unsigned, unsigned>, int> edges;
    double volume6 = 0, vertex_residual = 0, triangle_residual = 0;
    for (const auto& face : mesh.triangles) {
        for (unsigned k = 0; k < 3; ++k) {
            const auto x = face[k], y = face[(k + 1) % 3];
            edges[std::minmax(x, y)] += x < y ? 1 : -1;
        }
        const auto& p = mesh.vertices[face[0]];
        const auto& q = mesh.vertices[face[1]];
        const auto& s = mesh.vertices[face[2]];
        const Point u{q[0] - p[0], q[1] - p[1], q[2] - p[2]};
        const Point v{s[0] - p[0], s[1] - p[1], s[2] - p[2]};
        const Point cross{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
        ASSERT_GT(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2], 1e-28);
        volume6 += p[0] * cross[0] + p[1] * cross[1] + p[2] * cross[2];
    }
    for (const auto& edge : edges)
        EXPECT_EQ(edge.second, 0);
    EXPECT_EQ(mesh.vertices.size() - edges.size() + mesh.triangles.size(), 0u); // genus-one closed surface
    EXPECT_GT(volume6, 0);
    const std::size_t step = std::max<std::size_t>(1, mesh.vertices.size() / 256);
    for (std::size_t i = 0; i < mesh.vertices.size(); i += step)
        vertex_residual = std::max(vertex_residual, std::abs(EllipseDistance(mesh.vertices[i], a, b) - r));
    for (std::size_t i = 0; i < mesh.triangles.size(); i += std::max<std::size_t>(1, mesh.triangles.size() / 512)) {
        Point p{};
        for (auto index : mesh.triangles[i])
            for (unsigned k = 0; k < 3; ++k)
                p[k] += mesh.vertices[index][k] / 3;
        triangle_residual = std::max(triangle_residual, std::abs(EllipseDistance(p, a, b) - r));
    }
    EXPECT_LE(vertex_residual, mesh.centerline_error + 1e-9);
    EXPECT_LT(triangle_residual, std::min(r, std::min(a, b) - r) * .1);
    EXPECT_LE(triangle_residual, mesh.surface_error_bound);
    std::cout << "ring " << a << ' ' << b << ' ' << r << " vertices " << mesh.vertices.size() << " faces "
              << mesh.triangles.size() << " centerline_bound " << mesh.centerline_error << " total_conservative_bound "
              << mesh.surface_error_bound << " sampled_vertex_residual " << vertex_residual
              << " sampled_triangle_residual " << triangle_residual << '\n';

    ASSERT_NE(dInitODE2(0), 0);
    std::vector<double> vertices;
    std::vector<dTriIndex> indices;
    for (const auto& p : mesh.vertices)
        vertices.insert(vertices.end(), p.begin(), p.end());
    for (const auto& t : mesh.triangles)
        indices.insert(indices.end(), t.begin(), t.end());
    const auto data = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildDouble(data, vertices.data(), 3 * sizeof(double), mesh.vertices.size(), indices.data(),
                                indices.size(), 3 * sizeof(dTriIndex));
    const auto solid = dCreateTriMesh(nullptr, data, nullptr, nullptr, nullptr);
    const auto ray = dCreateRay(nullptr, 4 * (a + b + r));
    dContactGeom contact;
    dGeomRaySet(ray, 0, 0, -2 * r, 0, 0, 1);
    EXPECT_EQ(dCollide(ray, solid, 1, &contact, sizeof(contact)), 0); // through the hole
    dGeomRaySet(ray, a, 0, 2 * r, 0, 0, -1);
    EXPECT_EQ(dCollide(ray, solid, 1, &contact, sizeof(contact)), 1); // physical tube, not a hollow OBJ shell miss
    dGeomDestroy(ray);
    dGeomDestroy(solid);
    dGeomTriMeshDataDestroy(data);
    dCloseODE();
}

TEST(RingMesh, Circle) {
    CheckMesh(1, 1, .05);
}
TEST(RingMesh, SevenEllipse) {
    CheckMesh(1, 1.2, .05);
}
TEST(RingMesh, ThickBeyondCurvature) {
    CheckMesh(2, 1, .8);
}
TEST(RingMesh, NearMinimumRadiusPreservesHole) {
    CheckMesh(1.2, 1, .99);
}
TEST(RingMesh, RejectsInvalidOrUnrepresentableInputsExplicitly) {
    EXPECT_THROW(MakeRingMesh(1, 1, 1), std::invalid_argument);
    EXPECT_THROW(MakeRingMesh(2, 1, -1), std::invalid_argument);
    EXPECT_THROW(MakeRingMesh(2, 1, std::numeric_limits<double>::infinity()), std::invalid_argument);
    EXPECT_THROW(MakeRingMesh(2, 1, 1 - 1e-14), std::runtime_error);
}

TEST(RingMesh, CompilesOnePartToOneNativeMeshCollisionAndVisual) {
    const auto directory = std::filesystem::temp_directory_path() / ("xgc-ring-" + std::to_string(getpid()));
    std::filesystem::create_directories(directory);
    xgc2_geometry_msgs::SceneSnapshot scene;
    scene.scene_id = "ring";
    scene.epoch = "epoch";
    scene.header.frame_id = "world";
    xgc2_geometry_msgs::SceneObstacle obstacle;
    obstacle.id = "o";
    obstacle.pose.orientation.w = 1;
    xgc2_geometry_msgs::ScenePart part;
    part.id = "ring";
    part.pose.orientation.w = 1;
    part.geometry.type = "ring";
    part.geometry.radii[0] = 2;
    part.geometry.radii[1] = 1;
    part.geometry.tube_radius = .8;
    part.color.a = 1;
    obstacle.parts.push_back(part);
    scene.obstacles.push_back(obstacle);
    std::map<std::string, SceneModel> models;
    std::string error;
    ASSERT_TRUE(CompileScene(scene, directory.string(), &models, &error)) << error;
    ASSERT_EQ(models.size(), 1u);
    ASSERT_EQ(models.begin()->second.collisions.size(), 1u);
    const auto& model = models.begin()->second;
    EXPECT_EQ(model.collisions.front().geometry.type, "ring");
    EXPECT_FALSE(model.collisions.front().mesh_uri.empty());
    ModelSdfParser parser;
    ASSERT_TRUE(parser.Parse(model.sdf));
    const auto link = parser.Model()->GetElement("link");
    const auto collision = link->GetElement("collision"), visual = link->GetElement("visual");
    EXPECT_FALSE(collision->GetNextElement("collision"));
    EXPECT_FALSE(visual->GetNextElement("visual"));
    EXPECT_EQ(collision->Get<std::string>("name"), visual->Get<std::string>("name"));
    EXPECT_EQ(collision->GetElement("geometry")->GetElement("mesh")->Get<std::string>("uri"),
              visual->GetElement("geometry")->GetElement("mesh")->Get<std::string>("uri"));
    const auto loaded = gazebo::common::MeshManager::Instance()->Load(model.collisions.front().mesh_uri);
    ASSERT_NE(loaded, nullptr);
    std::vector<double> native_vertices;
    std::vector<dTriIndex> native_indices;
    for (unsigned i = 0; i < loaded->GetSubMeshCount(); ++i) {
        const auto submesh = loaded->GetSubMesh(i);
        const auto base = native_vertices.size() / 3;
        for (unsigned j = 0; j < submesh->GetVertexCount(); ++j) {
            const auto v = submesh->Vertex(j);
            native_vertices.insert(native_vertices.end(), {v.X(), v.Y(), v.Z()});
        }
        for (unsigned j = 0; j < submesh->GetIndexCount(); ++j)
            native_indices.push_back(base + submesh->GetIndex(j));
    }
    ASSERT_NE(dInitODE2(0), 0);
    const auto native_data = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildDouble(native_data, native_vertices.data(), 3 * sizeof(double), native_vertices.size() / 3,
                                native_indices.data(), native_indices.size(), 3 * sizeof(dTriIndex));
    const auto native_solid = dCreateTriMesh(nullptr, native_data, nullptr, nullptr, nullptr);
    const auto native_ray = dCreateRay(nullptr, 10);
    dContactGeom native_contact;
    dGeomRaySet(native_ray, 0, 0, -2, 0, 0, 1);
    EXPECT_EQ(dCollide(native_ray, native_solid, 1, &native_contact, sizeof(native_contact)), 0);
    dGeomRaySet(native_ray, 2, 0, 2, 0, 0, -1);
    EXPECT_EQ(dCollide(native_ray, native_solid, 1, &native_contact, sizeof(native_contact)), 1);
    dGeomDestroy(native_ray);
    dGeomDestroy(native_solid);
    dGeomTriMeshDataDestroy(native_data);
    dCloseODE();
    std::filesystem::remove_all(directory);
}

TEST(RingMesh, TwentyActualEllipsesCost) {
    const auto begin = std::chrono::steady_clock::now();
    std::size_t vertices = 0;
    for (unsigned i = 0; i < 20; ++i)
        vertices += MakeRingMesh(1 + .01 * i, 1.2 - .01 * i, .05).vertices.size();
    std::cout << "twenty_ring_vertices " << vertices << " seconds "
              << std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count() << '\n';
}
} // namespace
} // namespace xgc2_gazebo_scene
