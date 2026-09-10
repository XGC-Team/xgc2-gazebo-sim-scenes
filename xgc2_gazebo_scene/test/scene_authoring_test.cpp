#include "xgc2_gazebo_scene/scene_model.hpp"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <xgc2_geometry_msgs/ApplyScene.h>
#include <xgc2_geometry_msgs/SceneState.h>

#include <cmath>
#include <functional>
#include <limits>
#include <string>

namespace xgc2_gazebo_scene {
namespace {

gazebo::physics::WorldPtr world;

geometry_msgs::Pose Pose(double x = 0, double y = 0, double z = 0) {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = 1;
    return pose;
}

xgc2_geometry_msgs::ScenePart Part(const std::string& type, const std::string& id = "body") {
    xgc2_geometry_msgs::ScenePart part;
    part.id = id;
    part.pose = Pose();
    part.color.r = 0.8;
    part.color.g = 0.4;
    part.color.b = 0.2;
    part.color.a = 0.7;
    part.geometry.type = type;
    part.geometry.size.x = 1;
    part.geometry.size.y = 2;
    part.geometry.size.z = 3;
    part.geometry.radius = 0.4;
    part.geometry.height = 1.5;
    if (type == "convex") {
        // Deliberately non-uniformly scaled tetrahedron, not a bounding primitive.
        for (const auto& point : {ignition::math::Vector3d(0, 0, 0), {2, 0, 0}, {0, 0.7, 0}, {0, 0, 1.4}}) {
            geometry_msgs::Point vertex;
            vertex.x = point.X();
            vertex.y = point.Y();
            vertex.z = point.Z();
            part.geometry.vertices.push_back(vertex);
        }
        part.geometry.triangles = {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
    }
    return part;
}

xgc2_geometry_msgs::SceneObstacle Obstacle(const std::string& id, const std::string& shape, double x = 10) {
    xgc2_geometry_msgs::SceneObstacle obstacle;
    obstacle.id = id;
    obstacle.name = id;
    obstacle.pose = Pose(x, 0, 2);
    obstacle.parts.push_back(Part(shape));
    return obstacle;
}

bool Eventually(const std::function<bool()>& predicate, double timeout = 3.0) {
    const auto end = ros::WallTime::now() + ros::WallDuration(timeout);
    do {
        if (predicate())
            return true;
        ros::WallDuration(0.01).sleep();
    } while (ros::WallTime::now() < end);
    return false;
}

class SceneAuthoringTest : public testing::Test {
  protected:
    void SetUp() override {
        static unsigned epoch = 0;
        scene_.header.frame_id = "world";
        scene_.scene_id = "test";
        scene_.epoch = "test_epoch_" + std::to_string(++epoch);
        scene_.revision = 1;
        client_ = node_.serviceClient<xgc2_geometry_msgs::ApplyScene>("/xgc/scene/gazebo/apply");
        ASSERT_TRUE(client_.waitForExistence(ros::Duration(5)));
        ASSERT_TRUE(Apply()) << result_.message;
        world->SetPaused(true);
    }

    bool Apply() {
        xgc2_geometry_msgs::ApplyScene service;
        service.request.scene = scene_;
        if (!client_.call(service))
            return false;
        result_ = service.response;
        return result_.success;
    }

    gazebo::physics::ModelPtr Model(const std::string& id) { return world->ModelByName(SceneModelName(id)); }

    ros::NodeHandle node_;
    ros::ServiceClient client_;
    xgc2_geometry_msgs::SceneSnapshot scene_;
    xgc2_geometry_msgs::ApplyScene::Response result_;
};

TEST_F(SceneAuthoringTest, CreatesEveryShapeAndKeepsTheDoorwayOpenWhilePaused) {
    const auto time_before = world->SimTime();
    scene_.revision++;
    scene_.obstacles = {Obstacle("box", "box", 10), Obstacle("sphere", "sphere", 15),
                        Obstacle("cylinder", "cylinder", 20), Obstacle("capsule", "capsule", 25),
                        Obstacle("polyhedron", "convex", 30)};
    scene_.obstacles.back().pose.orientation.z = std::sin(0.35);
    scene_.obstacles.back().pose.orientation.w = std::cos(0.35);
    auto gate = Obstacle("gate", "box", 0);
    gate.pose = Pose();
    gate.parts.clear();
    for (const int side : {-1, 1}) {
        auto column = Part("box", side < 0 ? "left" : "right");
        column.pose = Pose(side * 1.5, 0, 1.25);
        column.geometry.size.x = 0.5;
        column.geometry.size.y = 0.5;
        column.geometry.size.z = 2.5;
        gate.parts.push_back(column);
    }
    auto lintel = Part("box", "lintel");
    lintel.pose = Pose(0, 0, 2.75);
    lintel.geometry.size.x = 3.5;
    lintel.geometry.size.y = 0.5;
    lintel.geometry.size.z = 0.5;
    gate.parts.push_back(lintel);
    scene_.obstacles.push_back(gate);
    ASSERT_TRUE(Apply()) << result_.message;
    EXPECT_EQ(scene_.epoch, result_.epoch);
    EXPECT_EQ(scene_.revision, result_.applied_revision);
    EXPECT_EQ(time_before, world->SimTime()) << "editing a paused world must not advance simulation";
    EXPECT_TRUE(world->IsPaused());

    ASSERT_TRUE(Model("box"));
    auto box = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(
        Model("box")->GetLink("body")->GetCollisions()[0]->GetShape());
    ASSERT_TRUE(box);
    EXPECT_EQ(ignition::math::Vector3d(1, 2, 3), box->Size());
    auto sphere = boost::dynamic_pointer_cast<gazebo::physics::SphereShape>(
        Model("sphere")->GetLink("body")->GetCollisions()[0]->GetShape());
    ASSERT_TRUE(sphere);
    EXPECT_DOUBLE_EQ(0.4, sphere->GetRadius());
    auto cylinder = boost::dynamic_pointer_cast<gazebo::physics::CylinderShape>(
        Model("cylinder")->GetLink("body")->GetCollisions()[0]->GetShape());
    ASSERT_TRUE(cylinder);
    EXPECT_DOUBLE_EQ(1.5, cylinder->GetLength());
    EXPECT_DOUBLE_EQ(0.4, cylinder->GetRadius());
    ASSERT_EQ(3u, Model("capsule")->GetLink("body")->GetCollisions().size());
    const auto capsule_bounds = Model("capsule")->BoundingBox().Size();
    EXPECT_NEAR(0.8, capsule_bounds.X(), 1.0e-6);
    EXPECT_NEAR(2.3, capsule_bounds.Z(), 1.0e-6);
    auto mesh = boost::dynamic_pointer_cast<gazebo::physics::MeshShape>(
        Model("polyhedron")->GetLink("body")->GetCollisions()[0]->GetShape());
    ASSERT_TRUE(mesh);
    EXPECT_EQ(ignition::math::Vector3d::One, mesh->Scale());
    EXPECT_EQ(3u, Model("gate")->GetLink("body")->GetCollisions().size());

    auto ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        world->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr()));
    ASSERT_TRUE(ray);
    double distance = 0;
    std::string entity;
    ray->SetPoints({0, -2, 1.25}, {0, 2, 1.25});
    ray->GetIntersection(distance, entity);
    EXPECT_TRUE(entity.empty()) << "gate opening was filled by a collision: " << entity;
    ray->SetPoints({-1.5, -2, 1.25}, {-1.5, 2, 1.25});
    ray->GetIntersection(distance, entity);
    EXPECT_NE(std::string::npos, entity.find(SceneModelName("gate")));
    EXPECT_NEAR(1.75, distance, 1.0e-6);
    EXPECT_TRUE(world->ModelByName("ground_plane"));
    EXPECT_TRUE(world->ModelByName("robot_camera_sentinel"));
}

TEST_F(SceneAuthoringTest, UpdatesDimensionsPoseAndMembershipWithoutChangingOtherIdentity) {
    scene_.revision++;
    scene_.obstacles = {Obstacle("keep", "sphere"), Obstacle("change", "box", 15)};
    ASSERT_TRUE(Apply()) << result_.message;
    const auto kept = Model("keep");
    const auto kept_entity_id = kept->GetId();
    scene_.revision++;
    scene_.obstacles[1].parts[0].geometry.size.y = 4;
    scene_.obstacles[1].pose = Pose(17, -2, 4);
    scene_.obstacles.push_back(Obstacle("new", "cylinder", 25));
    ASSERT_TRUE(Apply()) << result_.message;
    EXPECT_EQ(kept_entity_id, Model("keep")->GetId());
    EXPECT_EQ(ignition::math::Vector3d(17, -2, 4), Model("change")->WorldPose().Pos());
    auto box = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(
        Model("change")->GetLink("body")->GetCollisions()[0]->GetShape());
    ASSERT_TRUE(box);
    EXPECT_DOUBLE_EQ(4, box->Size().Y());
    scene_.revision++;
    scene_.obstacles.erase(scene_.obstacles.begin() + 1);
    ASSERT_TRUE(Apply()) << result_.message;
    EXPECT_FALSE(Model("change"));
    EXPECT_TRUE(Model("new"));
    EXPECT_EQ(kept_entity_id, Model("keep")->GetId());
    scene_.revision++;
    scene_.obstacles.clear();
    ASSERT_TRUE(Apply()) << result_.message;
    EXPECT_FALSE(Model("keep"));
    EXPECT_FALSE(Model("new"));
    EXPECT_EQ(2u, world->Models().size());
}

TEST_F(SceneAuthoringTest, ZeroHeightCapsuleIsAnExactSingleSphere) {
    scene_.revision++;
    scene_.obstacles = {Obstacle("zero_capsule", "capsule")};
    scene_.obstacles[0].parts[0].geometry.height = 0;
    ASSERT_TRUE(Apply()) << result_.message;
    const auto collisions = Model("zero_capsule")->GetLink("body")->GetCollisions();
    ASSERT_EQ(1u, collisions.size());
    const auto sphere = boost::dynamic_pointer_cast<gazebo::physics::SphereShape>(collisions[0]->GetShape());
    ASSERT_TRUE(sphere);
    EXPECT_DOUBLE_EQ(0.4, sphere->GetRadius());
}

TEST_F(SceneAuthoringTest, PreservesSubMillimetrePoseDimensionsAndPartTransforms) {
    scene_.revision++;
    scene_.obstacles = {Obstacle("precision", "box")};
    auto& obstacle = scene_.obstacles[0];
    obstacle.pose = Pose(2.551648123, -4.181376456, 2.500000123);
    obstacle.parts[0].geometry.size.x = 1.234567891;
    obstacle.parts[0].pose = Pose(0.0123456789, -0.0234567891, 0.0345678912);
    obstacle.parts[0].pose.orientation.z = std::sin(0.123456789);
    obstacle.parts[0].pose.orientation.w = std::cos(0.123456789);
    ASSERT_TRUE(Apply()) << result_.message;
    const auto model = Model("precision");
    EXPECT_NEAR(obstacle.pose.position.x, model->WorldPose().Pos().X(), 1.0e-9);
    const auto collision = model->GetLink("body")->GetCollisions()[0];
    EXPECT_NEAR(obstacle.parts[0].pose.position.y, collision->RelativePose().Pos().Y(), 1.0e-9);
    const auto box = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(collision->GetShape());
    ASSERT_TRUE(box);
    EXPECT_NEAR(1.234567891, box->Size().X(), 1.0e-9);
}

TEST_F(SceneAuthoringTest, RejectsInvalidGeometryAndStaleWritesBeforeMutatingWorld) {
    scene_.revision++;
    scene_.obstacles = {Obstacle("original", "box")};
    ASSERT_TRUE(Apply()) << result_.message;
    const auto original = Model("original");
    const auto accepted = scene_;
    ASSERT_TRUE(Apply()) << "identical retry must be idempotent";
    EXPECT_EQ(original->GetId(), Model("original")->GetId());
    scene_.obstacles[0].pose.position.x += 1;
    EXPECT_FALSE(Apply());
    EXPECT_NE(std::string::npos, result_.message.find("reused"));
    scene_ = accepted;
    scene_.revision--;
    EXPECT_FALSE(Apply());
    EXPECT_NE(std::string::npos, result_.message.find("stale"));
    scene_ = accepted;
    scene_.revision++;
    scene_.obstacles.push_back(Obstacle("bad", "torus"));
    EXPECT_FALSE(Apply());
    EXPECT_NE(std::string::npos, result_.message.find("unsupported"));
    scene_.obstacles.back() = Obstacle("bad", "convex");
    scene_.obstacles.back().parts[0].geometry.triangles.resize(9);
    EXPECT_FALSE(Apply());
    scene_.obstacles.back() = Obstacle("original", "sphere");
    EXPECT_FALSE(Apply());
    scene_.obstacles.pop_back();
    scene_.obstacles[0].parts[0].geometry.size.x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(Apply());
    EXPECT_EQ(original->GetId(), Model("original")->GetId());
    EXPECT_EQ(accepted.revision, result_.applied_revision);
    scene_ = accepted;
    scene_.epoch = "replacement_" + accepted.epoch;
    ASSERT_TRUE(Apply());
    scene_ = accepted;
    scene_.revision += 10;
    EXPECT_FALSE(Apply());
    EXPECT_NE(std::string::npos, result_.message.find("retired"));
}

TEST_F(SceneAuthoringTest, DoesNotAdoptForeignModelsAndRepairsExternallyDeletedOwnedModels) {
    const auto foreign_name = SceneModelName("foreign");
    world->InsertModelString("<sdf version='1.6'><model name='" + foreign_name +
                             "'><static>true</static><link name='foreign'/></model></sdf>");
    ASSERT_TRUE(Eventually([&] {
        return static_cast<bool>(world->ModelByName(foreign_name));
    }));
    const auto foreign_id = world->ModelByName(foreign_name)->GetId();
    scene_.revision++;
    scene_.obstacles = {Obstacle("foreign", "box")};
    EXPECT_FALSE(Apply());
    EXPECT_NE(std::string::npos, result_.message.find("outside"));
    EXPECT_EQ(foreign_id, world->ModelByName(foreign_name)->GetId());
    world->RemoveModel(foreign_name);
    ASSERT_TRUE(Apply()) << result_.message;
    world->RemoveModel(foreign_name);
    ASSERT_TRUE(Apply()) << "same accepted snapshot can repair missing physical models";
    ASSERT_TRUE(Model("foreign"));
}

TEST_F(SceneAuthoringTest, RepairsTimedOutFactoryUpdateWithLastAcceptedRevision) {
    // A deliberately short deadline produces a real insertion timeout without
    // stopping the physics server. This adapter has its own namespace/ownership.
    auto plugin_sdf = world->SDF()->GetElement("plugin")->Clone();
    plugin_sdf->GetElement("scene_namespace")->Set(std::string("/xgc/scene_short"));
    plugin_sdf->GetElement("apply_timeout")->Set(0.000001);
    world->LoadPlugin("libxgc2_scene_authoring_world.so", "short_deadline_scene", plugin_sdf);
    auto client = node_.serviceClient<xgc2_geometry_msgs::ApplyScene>("/xgc/scene_short/gazebo/apply");
    ASSERT_TRUE(client.waitForExistence(ros::Duration(3)));
    xgc2_geometry_msgs::ApplyScene call;
    call.request.scene = scene_;
    ASSERT_TRUE(client.call(call));
    ASSERT_TRUE(call.response.success) << call.response.message;
    const auto accepted = call.request.scene;
    call.request.scene.revision++;
    call.request.scene.obstacles = {Obstacle("late_factory", "box")};
    ASSERT_TRUE(client.call(call));
    EXPECT_FALSE(call.response.success);
    EXPECT_EQ(accepted.revision, call.response.applied_revision);
    ASSERT_TRUE(Eventually([&] {
        return static_cast<bool>(Model("late_factory"));
    }));
    call.request.scene = accepted;
    ASSERT_TRUE(client.call(call));
    ASSERT_TRUE(call.response.success) << call.response.message;
    EXPECT_EQ(accepted.revision, call.response.applied_revision);
    EXPECT_FALSE(Model("late_factory"));
    ros::WallDuration(0.3).sleep();
    EXPECT_FALSE(Model("late_factory")) << "late factory insertion escaped corrective snapshot";
}

TEST_F(SceneAuthoringTest, FollowsOnlyCompleteCurrentRevisionStateAndRejectsMalformedPoses) {
    scene_.revision++;
    scene_.obstacles = {Obstacle("moving", "capsule")};
    scene_.obstacles[0].dynamic = true;
    ASSERT_TRUE(Apply()) << result_.message;
    auto publisher = node_.advertise<xgc2_geometry_msgs::SceneState>("/xgc/scene/state", 1);
    ASSERT_TRUE(Eventually([&] {
        return publisher.getNumSubscribers() > 0;
    }));
    xgc2_geometry_msgs::SceneState state;
    state.header.frame_id = "world";
    state.epoch = scene_.epoch;
    state.revision = scene_.revision;
    xgc2_geometry_msgs::SceneObstacleState obstacle;
    obstacle.id = "moving";
    obstacle.pose = Pose(12, -3, 2);
    state.obstacles.push_back(obstacle);
    publisher.publish(state);
    ASSERT_TRUE(Eventually([&] {
        return Model("moving")->WorldPose().Pos().Equal({12, -3, 2}, 1.0e-6);
    }));
    state.revision--;
    state.obstacles[0].pose.position.x = 40;
    publisher.publish(state);
    ros::WallDuration(0.1).sleep();
    EXPECT_DOUBLE_EQ(12, Model("moving")->WorldPose().Pos().X());
    state.revision++;
    state.obstacles[0].pose.orientation.w = 0;
    publisher.publish(state);
    ros::WallDuration(0.1).sleep();
    EXPECT_DOUBLE_EQ(12, Model("moving")->WorldPose().Pos().X());
    state.obstacles[0].pose.orientation.w = 1;
    publisher.publish(state);
    EXPECT_TRUE(Eventually([&] {
        return Model("moving")->WorldPose().Pos().X() == 40;
    }));
    ASSERT_TRUE(Apply()) << result_.message;
    EXPECT_DOUBLE_EQ(40, Model("moving")->WorldPose().Pos().X()) << "snapshot retry reset current motion state";
    scene_.revision++;
    scene_.obstacles.push_back(Obstacle("other", "box", 50));
    ASSERT_TRUE(Apply()) << result_.message;
    EXPECT_DOUBLE_EQ(40, Model("moving")->WorldPose().Pos().X()) << "editing another object reset current motion state";
}

} // namespace
} // namespace xgc2_gazebo_scene

int main(int argc, char** argv) {
    ros::init(argc, argv, "scene_authoring_test");
    testing::InitGoogleTest(&argc, argv);
    gazebo::setupServer();
    xgc2_gazebo_scene::world = gazebo::loadWorld(XGC2_SCENE_AUTHORING_TEST_WORLD);
    if (!xgc2_gazebo_scene::world)
        return 2;
    // Embedded physics tests do not start Server's sensor/plugin startup loop.
    // Explicitly load the same plugin element that gzserver loads from the world.
    xgc2_gazebo_scene::world->LoadPlugin("libxgc2_scene_authoring_world.so", "xgc2_scene_authoring",
                                         xgc2_gazebo_scene::world->SDF()->GetElement("plugin"));
    xgc2_gazebo_scene::world->SetPaused(true);
    xgc2_gazebo_scene::world->Run();
    const int result = RUN_ALL_TESTS();
    xgc2_gazebo_scene::world->Stop();
    gazebo::shutdown();
    xgc2_gazebo_scene::world.reset();
    return result;
}
