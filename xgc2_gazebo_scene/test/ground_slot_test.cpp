#include "xgc2_gazebo_scene/native_scene_controller.hpp"

#include <gazebo/common/Mesh.hh>
#include <gazebo/common/MeshManager.hh>
#include <gazebo/common/SystemPaths.hh>
#include <gtest/gtest.h>
#include <ode/ode.h>
#include <sdf/sdf.hh>

#include <filesystem>
#include <fstream>

namespace xgc2_gazebo_scene {
namespace {

namespace fs = std::filesystem;

struct WorkingDirectory {
    fs::path previous;
    explicit WorkingDirectory(const fs::path& next) : previous(fs::current_path()) { fs::current_path(next); }
    ~WorkingDirectory() {
        std::error_code error;
        fs::current_path(previous, error);
    }
};

void ExpectSameInstalledSurface(const std::string& produced_sdf, const std::string& installed_sdf) {
    sdf::SDFPtr produced(new sdf::SDF());
    sdf::SDFPtr installed(new sdf::SDF());
    if (!sdf::init(produced) || !sdf::readString(produced_sdf, produced) || !sdf::init(installed) ||
        !sdf::readString(installed_sdf, installed))
        throw std::runtime_error("ground SDF could not be parsed");
    const auto produced_surface =
        produced->Root()->GetElement("model")->GetElement("link")->GetElement("collision")->GetElement("surface");
    const auto installed_surface =
        installed->Root()->GetElement("model")->GetElement("link")->GetElement("collision")->GetElement("surface");
    EXPECT_DOUBLE_EQ(produced_surface->GetElement("friction")->GetElement("ode")->Get<double>("mu"),
                     installed_surface->GetElement("friction")->GetElement("ode")->Get<double>("mu"));
    EXPECT_DOUBLE_EQ(produced_surface->GetElement("friction")->GetElement("ode")->Get<double>("mu2"),
                     installed_surface->GetElement("friction")->GetElement("ode")->Get<double>("mu2"));
    EXPECT_EQ(produced_surface->GetElement("contact")->Get<std::string>("collide_bitmask"),
              installed_surface->GetElement("contact")->Get<std::string>("collide_bitmask"));
}

std::string Document(const std::string& models, const std::string& obstacles, const std::string& ground) {
    return std::string(
               R"({"schema":"xgc2.scene.compiled.v1","id":"review","frame":"world","geometries":[{"type":"box","size":[1,1,1]}],"models":)") +
           models + R"(,"obstacles":)" + obstacles + R"(,"ground":)" + ground + "}";
}

const char* kBox =
    R"({"box":{"parts":[{"id":"slab","pose":{"position":[0,0,0],"orientation":[0,0,0,1]},"geometryIndex":0,"color":[1,0.5,0.1,1]}]}})";
const char* kHeld =
    R"([{"id":"held","name":"Held","model":"box","pose":{"position":[0,0,0],"orientation":[0,0,0,1]},"motion":{"type":"hold"}}])";

TEST(GroundSlot, SwitchesModesWithoutRebuildingUnchangedGround) {
    EXPECT_TRUE(IsReservedConsumerGroundId(kConsumerGroundEntityId));
    EXPECT_FALSE(IsReservedConsumerGroundId("ground_plane"));
    const auto mesh = fs::temp_directory_path() / "xgc-ground-numeric";
    fs::create_directories(mesh);
    const auto plane = InspectCompiledConsumer(Document(kBox, kHeld, R"({"mode":"default_plane","z":1.5})"),
                                               mesh.string(), fs::temp_directory_path().string());
    ASSERT_TRUE(plane.ground.has_value());
    EXPECT_EQ(plane.ground->name, kConsumerGroundName);
    EXPECT_EQ(plane.ground->sdf.find("ground_plane"), std::string::npos);
    EXPECT_EQ(plane.obstacles.count("held"), 1u);
    EXPECT_EQ(plane.moving, 0u);
    sdf::SDFPtr parsed(new sdf::SDF());
    ASSERT_TRUE(sdf::init(parsed));
    ASSERT_TRUE(sdf::readString(plane.ground->sdf, parsed));
    EXPECT_EQ(parsed->Root()->GetElement("model")->Get<std::string>("name"), kConsumerGroundName);
    EXPECT_NEAR(parsed->Root()->GetElement("model")->Get<ignition::math::Pose3d>("pose").Pos().Z(), 1.5, 1e-9);
    const auto stock = gazebo::common::SystemPaths::Instance()->FindFileURI("model://ground_plane/model.sdf");
    ASSERT_FALSE(stock.empty());
    std::ifstream stock_file(stock);
    const std::string stock_bytes((std::istreambuf_iterator<char>(stock_file)), {});
    ExpectSameInstalledSurface(plane.ground->sdf, stock_bytes);

    const auto moved = InspectCompiledConsumer(
        Document(
            kBox,
            R"([{"id":"other","name":"Other","model":"box","pose":{"position":[2,0,0],"orientation":[0,0,0,1]},"motion":{"type":"hold"}}])",
            R"({"mode":"default_plane","z":1.5})"),
        mesh.string(), fs::temp_directory_path().string());
    ASSERT_TRUE(moved.ground.has_value());
    EXPECT_TRUE(SamePhysicalGround(*plane.ground, *moved.ground));
    EXPECT_NE(plane.obstacles.begin()->first, moved.obstacles.begin()->first);

    const auto higher = InspectCompiledConsumer(Document(kBox, kHeld, R"({"mode":"default_plane","z":2.5})"),
                                                mesh.string(), fs::temp_directory_path().string());
    ASSERT_TRUE(higher.ground.has_value());
    EXPECT_FALSE(SamePhysicalGround(*plane.ground, *higher.ground));
    sdf::SDFPtr higher_sdf(new sdf::SDF());
    ASSERT_TRUE(sdf::init(higher_sdf));
    ASSERT_TRUE(sdf::readString(higher.ground->sdf, higher_sdf));
    EXPECT_NEAR(higher_sdf->Root()->GetElement("model")->Get<ignition::math::Pose3d>("pose").Pos().Z(), 2.5, 1e-9);

    const auto none = InspectCompiledConsumer(Document(kBox, kHeld, R"({"mode":"none"})"), mesh.string(),
                                              fs::temp_directory_path().string());
    EXPECT_FALSE(none.ground.has_value());
    const auto moving = InspectCompiledConsumer(
        Document(
            kBox,
            R"([{"id":"held","name":"Held","model":"box","pose":{"position":[0,0,0],"orientation":[0,0,0,1]},"motion":{"type":"hold"}},{"id":"mover","name":"Mover","model":"box","pose":{"position":[0,0,0],"orientation":[0,0,0,1]},"motion":{"type":"constant_twist","linear":[1,0,0],"angular":[0,0,0]}}])",
            R"({"mode":"default_plane","z":1.5})"),
        mesh.string(), fs::temp_directory_path().string());
    EXPECT_EQ(moving.moving, 1u);
    EXPECT_TRUE(SamePhysicalGround(*plane.ground, *moving.ground));
    fs::remove_all(mesh);
}

TEST(GroundSlot, ModelGroundKeepsTheSerializedHole) {
    const auto root = fs::temp_directory_path() / "xgc-ground-hole";
    fs::remove_all(root);
    fs::create_directories(root / "meshes");
    std::ofstream(root / "meshes" / "hole.obj") << R"(v -1 -1 0
v 1 -1 0
v 1 1 0
v -1 1 0
v -0.2 -0.2 0
v 0.2 -0.2 0
v 0.2 0.2 0
v -0.2 0.2 0
f 1 2 6
f 1 6 5
f 2 3 7
f 2 7 6
f 3 4 8
f 3 8 7
f 4 1 5
f 4 5 8
)";
    std::ofstream(root / "model.sdf") << R"(<?xml version="1.0"?>
<sdf version="1.6"><model name="authored"><pose>9 0 0 0 0 0</pose>
<link name="body"><visual name="shell"><geometry><mesh><uri>meshes/hole.obj</uri></mesh></geometry></visual>
<collision name="shell"><geometry><mesh><uri>meshes/hole.obj</uri></mesh></geometry></collision>
</link></model></sdf>)";
    const auto file = (root / "model.sdf").string();
    const auto models =
        std::string(
            R"({"shell":{"parts":[{"id":"query","pose":{"position":[0,0,0],"orientation":[0,0,0,1]},"geometryIndex":0,"color":[1,0.5,0.1,1]}],"resource":{"format":"sdformat-model","file":")") +
        file + R"("}}})";
    const auto slice = InspectCompiledConsumer(
        Document(models, "[]",
                 R"({"mode":"model","model":"shell","pose":{"position":[4,5,6],"orientation":[0,0,0,1]}})"),
        (root / "numeric").string(), root.string());
    ASSERT_TRUE(slice.ground.has_value());
    EXPECT_TRUE(slice.obstacles.empty());
    EXPECT_EQ(slice.ground->name, kConsumerGroundName);
    sdf::SDFPtr parsed(new sdf::SDF());
    ASSERT_TRUE(sdf::init(parsed));
    ASSERT_TRUE(sdf::readString(slice.ground->sdf, parsed));
    const auto model = parsed->Root()->GetElement("model");
    EXPECT_NEAR(model->Get<ignition::math::Pose3d>("pose").Pos().X(), 4, 1e-9);
    EXPECT_NEAR(model->Get<ignition::math::Pose3d>("pose").Pos().Z(), 6, 1e-9);
    const auto uri = model->GetElement("link")
                         ->GetElement("visual")
                         ->GetElement("geometry")
                         ->GetElement("mesh")
                         ->Get<std::string>("uri");
    EXPECT_EQ(uri, (root / "meshes" / "hole.obj").lexically_normal().string());
    std::ifstream authored(file);
    const std::string authored_bytes((std::istreambuf_iterator<char>(authored)), {});
    EXPECT_NE(authored_bytes.find("meshes/hole.obj"), std::string::npos);
    WorkingDirectory away("/");
    const auto loaded = gazebo::common::MeshManager::Instance()->Load(uri);
    ASSERT_NE(loaded, nullptr);
    ASSERT_NE(dInitODE2(0), 0);
    std::vector<double> vertices;
    std::vector<dTriIndex> indices;
    const auto submesh = loaded->GetSubMesh(0);
    for (unsigned i = 0; i < submesh->GetVertexCount(); ++i) {
        const auto vertex = submesh->Vertex(i);
        vertices.push_back(vertex.X());
        vertices.push_back(vertex.Y());
        vertices.push_back(vertex.Z());
    }
    for (unsigned i = 0; i < submesh->GetIndexCount(); ++i)
        indices.push_back(static_cast<dTriIndex>(submesh->GetIndex(i)));
    const auto data = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildDouble(data, vertices.data(), 3 * sizeof(double), static_cast<int>(submesh->GetVertexCount()),
                                indices.data(), static_cast<int>(indices.size()), 3 * sizeof(dTriIndex));
    const auto solid = dCreateTriMesh(nullptr, data, nullptr, nullptr, nullptr);
    const auto ray = dCreateRay(nullptr, 4);
    dContactGeom contact;
    dGeomRaySet(ray, 0, 0, 1, 0, 0, -1);
    EXPECT_EQ(dCollide(ray, solid, 1, &contact, sizeof(contact)), 0);
    dGeomDestroy(ray);
    dGeomDestroy(solid);
    dGeomTriMeshDataDestroy(data);
    dCloseODE();
    fs::remove_all(root);
}

} // namespace
} // namespace xgc2_gazebo_scene
