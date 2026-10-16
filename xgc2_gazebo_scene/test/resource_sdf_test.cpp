#include "xgc2_gazebo_scene/native_scene_controller.hpp"

#include <gazebo/common/Mesh.hh>
#include <gazebo/common/MeshManager.hh>
#include <gtest/gtest.h>
#include <ode/ode.h>
#include <sdf/sdf.hh>

#include <filesystem>
#include <fstream>
#include <set>

namespace xgc2_gazebo_scene {
namespace {

namespace fs = std::filesystem;

struct WorkingDirectory {
    fs::path previous;
    explicit WorkingDirectory(const fs::path& next) : previous(fs::current_path()) { fs::current_path(next); }
    ~WorkingDirectory() { std::error_code error; fs::current_path(previous, error); }
};

std::string SerializedMeshUri(const std::string& sdf) {
    sdf::SDFPtr parsed(new sdf::SDF());
    if (!sdf::init(parsed) || !sdf::readString(sdf, parsed))
        throw std::runtime_error("serialized scene SDF could not be parsed");
    const auto model = parsed->Root()->GetElement("model");
    for (auto link = model->GetElement("link"); link; link = link->GetNextElement("link")) {
        if (!link->HasElement("visual"))
            continue;
        const auto geometry = link->GetElement("visual")->GetElement("geometry");
        if (!geometry->HasElement("mesh"))
            continue;
        return geometry->GetElement("mesh")->Get<std::string>("uri");
    }
    throw std::runtime_error("serialized scene SDF has no mesh URI");
}

class ResourceSdf : public testing::Test {
  protected:
    void SetUp() override {
        root = fs::temp_directory_path() / "xgc-authored-resource";
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
        Write("model.sdf", Model());
    }
    void TearDown() override { fs::remove_all(root); }
    void Write(const std::string& name, const std::string& bytes) const {
        std::ofstream output(root / name);
        output << bytes;
    }
    static std::string Model() {
        return R"(<?xml version="1.0"?>
<sdf version="1.6"><model name="authored"><pose>9 0 0 0 0 0</pose>
<link name="body">
  <visual name="shell"><geometry><mesh><uri>meshes/hole.obj</uri><scale>1 1 1</scale></mesh></geometry>
    <material><ambient>0.1 0.2 0.8 1</ambient><diffuse>0.1 0.2 0.8 1</diffuse></material></visual>
  <collision name="shell"><geometry><mesh><uri>meshes/hole.obj</uri><scale>1 1 1</scale></mesh></geometry></collision>
</link>
<link name="mast"><pose>0 0 1 0 0 0</pose>
  <visual name="tip"><geometry><box><size>0.1 0.1 0.2</size></box></geometry>
    <material><ambient>0.2 0.9 0.3 1</ambient></material></visual>
  <collision name="tip"><geometry><box><size>0.1 0.1 0.2</size></box></geometry></collision>
</link>
</model></sdf>
)";
    }
    std::string Document(const std::string& file, const std::string& motion) const {
        return std::string(R"({"schema":"xgc2.scene.compiled.v1","id":"review","frame":"world","geometries":[{"type":"box","size":[1,2,3]}],"models":{"shell":{"parts":[{"id":"query","pose":{"position":[0,0,0],"orientation":[0,0,0,1]},"geometryIndex":0,"color":[1,0.5,0.1,1]}],"resource":{"format":"sdformat-model","file":")") +
               file + R"("}},"floor":{"parts":[{"id":"slab","pose":{"position":[0,0,0],"orientation":[0,0,0,1]},"geometryIndex":0,"color":[0.5,0.5,0.5,1]}]}},"obstacles":[{"id":"instance0","name":"Shell","model":"shell","pose":{"position":[4,5,6],"orientation":[0,0,0,1]},"motion":)" +
               motion + R"(}],"ground":{"mode":"model","model":"floor","pose":{"position":[0,0,-1],"orientation":[0,0,0,1]}}})";
    }
    fs::path root;
};

TEST_F(ResourceSdf, KeepsMaterialLinksAndMeshHole) {
    const auto models = BuildCompiledSceneModels(Document((root / "model.sdf").string(), R"({"type":"hold"})"),
                                                  (root / "numeric").string(), root.string());
    ASSERT_EQ(models.size(), 1u);
    ASSERT_EQ(models.count("instance0"), 1u);
    EXPECT_EQ(models.count("floor"), 0u);
    EXPECT_EQ(models.count("ground"), 0u);
    const auto& model = models.at("instance0");
    EXPECT_TRUE(model.authored_resource);
    EXPECT_EQ(model.collisions.size(), 1u);
    EXPECT_EQ(model.collisions.front().geometry.type, xgc2_math::geometry::GeometryType::Box);
    EXPECT_EQ(model.sdf.find("1 0.5 0.1"), std::string::npos);
    sdf::SDFPtr parsed(new sdf::SDF());
    ASSERT_TRUE(sdf::init(parsed));
    ASSERT_TRUE(sdf::readString(model.sdf, parsed));
    const auto root_model = parsed->Root()->GetElement("model");
    EXPECT_EQ(root_model->Get<std::string>("name"), SceneModelName("instance0"));
    const auto pose = root_model->Get<ignition::math::Pose3d>("pose");
    EXPECT_NEAR(pose.Pos().X(), 4, 1e-9);
    EXPECT_NEAR(pose.Pos().Y(), 5, 1e-9);
    EXPECT_NEAR(pose.Pos().Z(), 6, 1e-9);
    std::set<std::string> links;
    ignition::math::Color ambient;
    std::string mesh;
    for (auto link = root_model->GetElement("link"); link; link = link->GetNextElement("link")) {
        links.insert(link->Get<std::string>("name"));
        if (!link->HasElement("visual"))
            continue;
        const auto visual = link->GetElement("visual");
        if (visual->Get<std::string>("name") == "shell") {
            ambient = visual->GetElement("material")->Get<ignition::math::Color>("ambient");
            mesh = visual->GetElement("geometry")->GetElement("mesh")->Get<std::string>("uri");
        }
    }
    EXPECT_EQ(links, (std::set<std::string>{"body", "mast"}));
    EXPECT_NEAR(ambient.R(), 0.1, 1e-6);
    EXPECT_NEAR(ambient.G(), 0.2, 1e-6);
    EXPECT_NEAR(ambient.B(), 0.8, 1e-6);
    const auto sealed = (root / "meshes" / "hole.obj").lexically_normal().string();
    EXPECT_EQ(mesh, sealed);
    EXPECT_EQ(SerializedMeshUri(model.sdf), sealed);
    std::ifstream authored(root / "model.sdf");
    const std::string authored_bytes((std::istreambuf_iterator<char>(authored)), {});
    EXPECT_NE(authored_bytes.find("meshes/hole.obj"), std::string::npos);
    EXPECT_EQ(authored_bytes.find(sealed), std::string::npos);
    WorkingDirectory away("/");
    const auto loaded = gazebo::common::MeshManager::Instance()->Load(mesh);
    ASSERT_NE(loaded, nullptr);
    ASSERT_GE(loaded->GetSubMeshCount(), 1u);
    EXPECT_EQ(loaded->GetSubMesh(0)->GetIndexCount(), 24u);

    ASSERT_NE(dInitODE2(0), 0);
    std::vector<double> vertices;
    std::vector<dTriIndex> indices;
    const auto submesh = loaded->GetSubMesh(0);
    for (unsigned i = 0; i < submesh->GetVertexCount(); ++i) {
        const auto vertex = submesh->Vertex(i);
        vertices.insert(vertices.end(), {vertex.X(), vertex.Y(), vertex.Z()});
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
    dGeomRaySet(ray, 0.6, 0, 1, 0, 0, -1);
    EXPECT_EQ(dCollide(ray, solid, 1, &contact, sizeof(contact)), 1);
    dGeomDestroy(ray);
    dGeomDestroy(solid);
    dGeomTriMeshDataDestroy(data);
    dCloseODE();
}

TEST_F(ResourceSdf, RejectsIncludeEntityAndWaypoints) {
    Write("include.sdf",
          "<sdf version=\"1.6\"><model name=\"m\"><link name=\"l\"><collision name=\"c\"><geometry><box><size>1 1 1</size></box></geometry></collision></link><include><uri>&#47;tmp&#47;outside-asset-root</uri></include></model></sdf>");
    EXPECT_THROW(BuildCompiledSceneModels(Document((root / "include.sdf").string(), R"({"type":"hold"})"),
                                           (root / "numeric").string(), root.string()),
                 std::invalid_argument);
    EXPECT_THROW(BuildCompiledSceneModels(Document((root / "model.sdf").string(), R"({"type":"waypoints"})"),
                                           (root / "numeric").string(), root.string()),
                 std::invalid_argument);
}

TEST_F(ResourceSdf, ResolvesSiblingFilesInsideTheAssetRoot) {
    fs::create_directories(root / "models" / "bridge");
    fs::create_directories(root / "packages" / "bridge" / "meshes");
    fs::copy_file(root / "meshes" / "hole.obj", root / "packages" / "bridge" / "meshes" / "hole.obj");
    Write("models/bridge/model.sdf", R"(<?xml version="1.0"?>
<sdf version="1.6"><model name="bridge"><link name="bridge_link">
  <collision name="collision"><geometry><mesh><uri>../../packages/bridge/meshes/hole.obj</uri></mesh></geometry></collision>
  <visual name="visual"><geometry><mesh><uri>../../packages/bridge/meshes/hole.obj</uri></mesh></geometry></visual>
</link></model></sdf>)");
    const auto models = BuildCompiledSceneModels(Document((root / "models" / "bridge" / "model.sdf").string(), R"({"type":"hold"})"),
                                                  (root / "numeric").string(), root.string());
    ASSERT_EQ(models.count("instance0"), 1u);
    const auto sealed = (root / "packages" / "bridge" / "meshes" / "hole.obj").lexically_normal().string();
    const auto uri = SerializedMeshUri(models.at("instance0").sdf);
    EXPECT_EQ(uri, sealed);
    {
        WorkingDirectory away("/");
        ASSERT_NE(gazebo::common::MeshManager::Instance()->Load(uri), nullptr);
    }
    std::ifstream authored(root / "models" / "bridge" / "model.sdf");
    const std::string authored_bytes((std::istreambuf_iterator<char>(authored)), {});
    EXPECT_NE(authored_bytes.find("../../packages/bridge/meshes/hole.obj"), std::string::npos);
    const auto outside = root.parent_path() / "outside-grant";
    fs::create_directories(outside);
    std::ofstream(outside / "secret.obj") << "v 0 0 0\n";
    Write("models/bridge/escape.sdf", R"(<?xml version="1.0"?>
<sdf version="1.6"><model name="bridge"><link name="bridge_link">
  <collision name="collision"><geometry><mesh><uri>../../../outside-grant/secret.obj</uri></mesh></geometry></collision>
</link></model></sdf>)");
    EXPECT_THROW(BuildCompiledSceneModels(Document((root / "models" / "bridge" / "escape.sdf").string(), R"({"type":"hold"})"),
                                           (root / "numeric").string(), root.string()),
                 std::invalid_argument);
    fs::remove_all(outside);
    fs::create_symlink(root / "meshes" / "hole.obj", root / "meshes" / "linked.obj");
    Write("linked.sdf", R"(<?xml version="1.0"?>
<sdf version="1.6"><model name="m"><link name="l">
  <collision name="c"><geometry><mesh><uri>meshes/linked.obj</uri></mesh></geometry></collision>
</link></model></sdf>)");
    EXPECT_THROW(BuildCompiledSceneModels(Document((root / "linked.sdf").string(), R"({"type":"hold"})"),
                                           (root / "numeric").string(), root.string()),
                 std::invalid_argument);
}

TEST_F(ResourceSdf, AcceptsInstalledGazeboMediaAndRejectsOtherFileUris) {
    Write("stock.sdf", R"(<?xml version="1.0"?>
<sdf version="1.6"><model name="box"><link name="l">
  <collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision>
  <visual name="v"><geometry><box><size>1 1 1</size></box></geometry>
    <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
  </visual>
</link></model></sdf>)");
    const auto models = BuildCompiledSceneModels(Document((root / "stock.sdf").string(), R"({"type":"hold"})"),
                                                  (root / "numeric").string(), root.string());
    EXPECT_NE(models.at("instance0").sdf.find("file://media/materials/scripts/gazebo.material"), std::string::npos);
    fs::create_directories(root / "materials" / "scripts");
    std::ofstream(root / "materials" / "scripts" / "box.material") << "material Box/Authored {}\n";
    Write("script.sdf", R"(<?xml version="1.0"?>
<sdf version="1.6"><model name="box"><link name="l">
  <collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision>
  <visual name="v"><geometry><box><size>1 1 1</size></box></geometry>
    <material><script><uri>materials/scripts</uri><name>Box/Authored</name></script></material>
  </visual>
</link></model></sdf>)");
    const auto scripted = BuildCompiledSceneModels(Document((root / "script.sdf").string(), R"({"type":"hold"})"),
                                                    (root / "numeric").string(), root.string());
    const auto script_uri = (root / "materials" / "scripts").lexically_normal().string();
    EXPECT_NE(scripted.at("instance0").sdf.find(script_uri), std::string::npos);
    Write("script-dir-mesh.sdf", R"(<?xml version="1.0"?>
<sdf version="1.6"><model name="box"><link name="l">
  <collision name="c"><geometry><mesh><uri>materials/scripts</uri></mesh></geometry></collision>
</link></model></sdf>)");
    EXPECT_THROW(BuildCompiledSceneModels(Document((root / "script-dir-mesh.sdf").string(), R"({"type":"hold"})"),
                                           (root / "numeric").string(), root.string()),
                 std::invalid_argument);
    Write("remote.sdf", R"(<?xml version="1.0"?>
<sdf version="1.6"><model name="box"><link name="l">
  <visual name="v"><geometry><mesh><uri>file:///etc/passwd</uri></mesh></geometry></visual>
  <collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision>
</link></model></sdf>)");
    EXPECT_THROW(BuildCompiledSceneModels(Document((root / "remote.sdf").string(), R"({"type":"hold"})"),
                                           (root / "numeric").string(), root.string()),
                 std::invalid_argument);
    Write("network.sdf", R"(<?xml version="1.0"?>
<sdf version="1.6"><model name="box"><link name="l">
  <visual name="v"><geometry><mesh><uri>https://example.com/mesh.obj</uri></mesh></geometry></visual>
  <collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision>
</link></model></sdf>)");
    EXPECT_THROW(BuildCompiledSceneModels(Document((root / "network.sdf").string(), R"({"type":"hold"})"),
                                           (root / "numeric").string(), root.string()),
                 std::invalid_argument);
}

} // namespace
} // namespace xgc2_gazebo_scene
