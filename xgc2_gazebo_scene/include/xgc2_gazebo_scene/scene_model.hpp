#pragma once

#include <gazebo/physics/PhysicsTypes.hh>
#include <ignition/math/Pose3.hh>
#include <xgc2_geometry_msgs/SceneSnapshot.h>

#include <map>
#include <string>
#include <vector>

namespace xgc2_gazebo_scene {

struct SceneCollision {
    std::string name;
    ignition::math::Pose3d pose;
    xgc2_geometry_msgs::SceneGeometry geometry;
    std::string mesh_uri;
};

struct SceneModel {
    std::string id;
    std::string name;
    ignition::math::Pose3d pose;
    // Body contains all collision/visual definitions, excluding the model pose.
    std::string body;
    std::string sdf;
    std::vector<SceneCollision> collisions;
};

// Encoding, rather than replacing punctuation, makes distinct stable IDs injective.
std::string SceneModelName(const std::string& id);
bool ValidateScenePose(const geometry_msgs::Pose& pose);
ignition::math::Pose3d ScenePose(const geometry_msgs::Pose& pose);

// Does not change the world. Convex meshes are validated, then materialized into
// an adapter-owned temporary directory so both Gazebo collision and rendering
// load the same mesh. No URI or filesystem path comes from an authoring command.
bool CompileScene(const xgc2_geometry_msgs::SceneSnapshot& scene, const std::string& mesh_directory,
                  std::map<std::string, SceneModel>* models, std::string* error);

// Gazebo's SDF factory clones values through low-precision strings. Restore the
// original typed parameters after allocation, under the physics update mutex.
bool ApplySceneModelParameters(const gazebo::physics::ModelPtr& model, const SceneModel& expected,
                               const ignition::math::Pose3d& current_pose);

// Inspect the runtime physics objects, not just the requested SDF or model name.
bool VerifySceneModel(const gazebo::physics::ModelPtr& model, const SceneModel& expected, bool verify_pose,
                      std::string* error);

} // namespace xgc2_gazebo_scene
