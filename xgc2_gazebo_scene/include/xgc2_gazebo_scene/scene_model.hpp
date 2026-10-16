#pragma once

#include "xgc2_gazebo_scene/model_sdf_parser.hpp"

#include <gazebo/physics/PhysicsTypes.hh>
#include <ignition/math/Pose3.hh>
#include <xgc2_geometry_msgs/SceneSnapshot.h>

#include <xgc2_math/geometry/geometry_payload.hpp>

#include <map>
#include <string>
#include <vector>

namespace xgc2_gazebo_scene {

struct SceneCollision {
    std::string name;
    ignition::math::Pose3d pose;
    xgc2_math::geometry::Geometry geometry;
    std::string mesh_uri;
};

struct SceneModel {
    std::string id;
    std::string name;
    ignition::math::Pose3d pose;
    // Body contains all collision/visual definitions, excluding the model pose.
    std::string body;
    // Gazebo instantiates sdf. collisions is the numeric compiled description used for queries.
    // An authored resource keeps the original SDF visuals, materials, links and collisions.
    std::string sdf;
    std::vector<SceneCollision> collisions;
    bool authored_resource = false;
    std::string source_file;
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

// Reads one sealed model SDF. Relative URIs must stay inside asset_root, then the derived runtime
// XML rebases them to those absolute sealed paths. file://media stays the installed Gazebo stock URI.
// Material script directories are allowed. The authored file is not rewritten.
// The instance pose replaces the model pose. Original links, visuals, materials and collisions stay.
bool AuthorResourceModel(const std::string& id, const ignition::math::Pose3d& instance_pose,
                         const std::string& sdf_file, const std::string& asset_root, SceneModel* model,
                         std::string* error);

// Gazebo model name of the consumer-owned ground. It is not ground_plane and not an obstacle id.
inline constexpr char kConsumerGroundName[] = "xgc2_scene_ground";
// Public entity id of that ground slot. Create must not occupy it.
inline constexpr char kConsumerGroundEntityId[] = "xgc-scene-ground";

inline bool IsReservedConsumerGroundId(const std::string& id) {
    return id == kConsumerGroundEntityId;
}

// Copies the installed ground_plane SDF, keeps its contact and friction, and places the surface at z.
bool MakeInstalledGroundPlane(double z, SceneModel* model, std::string* error);
// Rewrites only the derived model name so factory ACK cannot collide with stock ground_plane.
bool UseConsumerGroundName(SceneModel* model, std::string* error);
bool SamePhysicalGround(const SceneModel& kept, const SceneModel& next);

// Inspect the runtime physics objects, not just the requested SDF or model name.
bool VerifySceneModel(const gazebo::physics::ModelPtr& model, const SceneModel& expected, bool verify_pose,
                      std::string* error);

} // namespace xgc2_gazebo_scene
