#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/BoxShape.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/MeshShape.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/WorldState.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "xgc2_gazebo_scene/ConfigureMotions.h"
#include "xgc2_gazebo_scene/ConvexPart.h"
#include "xgc2_gazebo_scene/MotionSpec.h"
#include "xgc2_gazebo_scene/ObstacleDefinition.h"
#include "xgc2_gazebo_scene/ObstacleDefinitionArray.h"
#include "xgc2_gazebo_scene/ObstacleState.h"
#include "xgc2_gazebo_scene/ObstacleStateArray.h"
#include "xgc2_gazebo_scene/StopMotions.h"
#include "xgc2_gazebo_scene/convex_mesh_geometry.hpp"
#include "xgc2_gazebo_scene/motion_controller.hpp"
#include "xgc2_gazebo_scene/physical_contact_filter.hpp"
#include "xgc2_gazebo_scene/scene_ownership.hpp"
#include "xgc2_geometry_msgs/ConvexBodyArray.h"
#include "xgc2_geometry_msgs/ConvexBodyInstance.h"
#include "xgc2_geometry_msgs/GeometryLibrary.h"
#include "xgc2_geometry_msgs/GeometryTemplate.h"

namespace xgc2_gazebo_scene {
namespace {

constexpr char kDefaultManagedPrefix[] = "xgc2_obstacle_";
constexpr char kGeometryTopic[] = "/xgc2/simulation/obstacles/geometry";
constexpr char kStateTopic[] = "/xgc2/simulation/obstacles/state";
constexpr char kGeometryLibraryTopic[] = "/xgc2/simulation/obstacles/geometry_library";
constexpr char kInstancesTopic[] = "/xgc2/simulation/obstacles/instances";
constexpr char kPhysicalCollisionTopic[] = "/xgc2/simulation/physical_collision";
constexpr char kPhysicalCollisionDetailTopic[] = "/xgc2/simulation/physical_collision_detail";
constexpr char kConfigureService[] = "/xgc2/gazebo/obstacles/configure_motions";
constexpr char kStopService[] = "/xgc2/gazebo/obstacles/stop_motions";
constexpr double kPublishPeriod = 1.0 / 30.0;
constexpr double kExternalPositionTolerance = 1.0e-6;
constexpr double kExternalOrientationTolerance = 1.0e-6;
constexpr int kCylinderVertexCount = 16;

geometry_msgs::Point PointMessage(const ignition::math::Vector3d& value) {
    geometry_msgs::Point message;
    message.x = value.X();
    message.y = value.Y();
    message.z = value.Z();
    return message;
}

geometry_msgs::Pose PoseMessage(const ignition::math::Pose3d& value) {
    geometry_msgs::Pose message;
    message.position = PointMessage(value.Pos());
    message.orientation.x = value.Rot().X();
    message.orientation.y = value.Rot().Y();
    message.orientation.z = value.Rot().Z();
    message.orientation.w = value.Rot().W();
    return message;
}

ignition::math::Pose3d IgnitionPose(const geometry_msgs::Pose& value) {
    return {{value.position.x, value.position.y, value.position.z},
            {value.orientation.w, value.orientation.x, value.orientation.y, value.orientation.z}};
}

geometry_msgs::Twist TwistMessage(const ignition::math::Vector3d& linear, const ignition::math::Vector3d& angular) {
    geometry_msgs::Twist message;
    message.linear.x = linear.X();
    message.linear.y = linear.Y();
    message.linear.z = linear.Z();
    message.angular.x = angular.X();
    message.angular.y = angular.Y();
    message.angular.z = angular.Z();
    return message;
}

geometry_msgs::Vector3 VectorMessage(const ignition::math::Vector3d& value) {
    geometry_msgs::Vector3 message;
    message.x = value.X();
    message.y = value.Y();
    message.z = value.Z();
    return message;
}

void AppendBoxVertices(const ignition::math::Vector3d& size, std::vector<geometry_msgs::Point>* vertices) {
    const ignition::math::Vector3d half = size * 0.5;
    for (const double x : {-half.X(), half.X()}) {
        for (const double y : {-half.Y(), half.Y()}) {
            for (const double z : {-half.Z(), half.Z()}) {
                vertices->push_back(PointMessage({x, y, z}));
            }
        }
    }
}

void AppendSphereVertices(double radius, std::vector<geometry_msgs::Point>* vertices) {
    // An octahedron whose inradius equals the sphere radius is a conservative
    // outer approximation. Analytical consumers should use radius directly.
    const double extent = radius * std::sqrt(3.0);
    vertices->push_back(PointMessage({extent, 0, 0}));
    vertices->push_back(PointMessage({-extent, 0, 0}));
    vertices->push_back(PointMessage({0, extent, 0}));
    vertices->push_back(PointMessage({0, -extent, 0}));
    vertices->push_back(PointMessage({0, 0, extent}));
    vertices->push_back(PointMessage({0, 0, -extent}));
}

void AppendCylinderVertices(double radius, double length, std::vector<geometry_msgs::Point>* vertices) {
    constexpr double kPi = 3.14159265358979323846;
    const double outer_radius = radius / std::cos(kPi / kCylinderVertexCount);
    for (int index = 0; index < kCylinderVertexCount; ++index) {
        const double angle = 2.0 * kPi * index / kCylinderVertexCount;
        const double x = outer_radius * std::cos(angle);
        const double y = outer_radius * std::sin(angle);
        vertices->push_back(PointMessage({x, y, -length * 0.5}));
        vertices->push_back(PointMessage({x, y, length * 0.5}));
    }
}

std::string LogicalName(const std::string& model_name, const std::string& managed_prefix) {
    if (managed_prefix.empty() || model_name.compare(0, managed_prefix.size(), managed_prefix) != 0) {
        return "";
    }
    return model_name.substr(managed_prefix.size());
}

std::string ModelNameFromScopedCollision(const std::string& collision_name) {
    const std::string::size_type separator = collision_name.find("::");
    if (separator == std::string::npos) {
        return collision_name;
    }
    return collision_name.substr(0, separator);
}

std::string StandardGeometryType(const ConvexPart& part) {
    switch (part.shape) {
    case ConvexPart::SHAPE_BOX:
        return "cube";
    case ConvexPart::SHAPE_SPHERE:
        return "sphere";
    case ConvexPart::SHAPE_CYLINDER:
        return "cylinder";
    case ConvexPart::SHAPE_CONVEX_MESH: {
        std::string type = "convex_mesh:" + part.mesh_uri;
        if (!part.mesh_submesh.empty()) {
            type += "#submesh=" + part.mesh_submesh;
        }
        if (part.mesh_center_submesh) {
            type += "#centered";
        }
        return type;
    }
    default:
        return "";
    }
}

std::string VPolytopeGeometryTypeAlias(const ConvexPart& part) {
    if (part.shape != ConvexPart::SHAPE_CONVEX_MESH || part.mesh_uri.empty() || !part.mesh_submesh.empty() ||
        part.mesh_center_submesh) {
        return "";
    }

    // The GeometryLibrary contract names a V-polytope template after its
    // source template (for example "v_polytope:ugv_hexagon_1"). Gazebo
    // collision meshes carry the same stable name in the asset filename. Keep
    // the URI type canonical for obstacle instances, and publish this semantic
    // alias so non-obstacle consumers can share the exact collision-derived
    // support points.
    std::string filename = part.mesh_uri;
    const std::string::size_type suffix = filename.find_first_of("?#");
    if (suffix != std::string::npos) {
        filename.erase(suffix);
    }
    const std::string::size_type slash = filename.find_last_of("/\\");
    if (slash != std::string::npos) {
        filename.erase(0, slash + 1);
    }
    const std::string::size_type extension = filename.find_last_of('.');
    if (extension != std::string::npos && extension != 0) {
        filename.erase(extension);
    }
    return filename.empty() ? "" : "v_polytope:" + filename;
}

xgc2_geometry_msgs::GeometryTemplate StandardGeometryTemplate(const ConvexPart& part) {
    xgc2_geometry_msgs::GeometryTemplate geometry_template;
    geometry_template.type = StandardGeometryType(part);
    geometry_template.resolution = 0;
    if (part.shape == ConvexPart::SHAPE_BOX) {
        for (const auto& vertex : part.conservative_vertices) {
            geometry_msgs::Point point;
            point.x = vertex.x / part.size.x;
            point.y = vertex.y / part.size.y;
            point.z = vertex.z / part.size.z;
            geometry_template.support_points.push_back(point);
        }
    } else if (part.shape == ConvexPart::SHAPE_CONVEX_MESH) {
        for (const auto& vertex : part.conservative_vertices) {
            geometry_msgs::Point point;
            point.x = vertex.x / part.mesh_scale.x;
            point.y = vertex.y / part.mesh_scale.y;
            point.z = vertex.z / part.mesh_scale.z;
            geometry_template.support_points.push_back(point);
        }
    }
    return geometry_template;
}

geometry_msgs::Vector3 StandardInstanceScale(const ConvexPart& part) {
    switch (part.shape) {
    case ConvexPart::SHAPE_BOX:
        return part.size;
    case ConvexPart::SHAPE_SPHERE:
        return VectorMessage({part.radius, part.radius, part.radius});
    case ConvexPart::SHAPE_CYLINDER:
        return VectorMessage({part.radius, part.radius, part.length});
    case ConvexPart::SHAPE_CONVEX_MESH:
        return part.mesh_scale;
    default:
        return geometry_msgs::Vector3{};
    }
}

bool PoseNearlyEqual(const ignition::math::Pose3d& left, const ignition::math::Pose3d& right) {
    if ((left.Pos() - right.Pos()).Length() > kExternalPositionTolerance) {
        return false;
    }
    const double dot = std::abs(left.Rot().W() * right.Rot().W() + left.Rot().X() * right.Rot().X() +
                                left.Rot().Y() * right.Rot().Y() + left.Rot().Z() * right.Rot().Z());
    return std::abs(1.0 - dot) <= kExternalOrientationTolerance;
}

std::string ConfigureFingerprint(const xgc2_gazebo_scene::ConfigureMotions::Request& request) {
    std::ostringstream stream;
    stream.precision(17);
    stream << request.expected_scene_revision;
    for (const auto& motion : request.motions) {
        stream << '|' << motion.name << '|' << motion.mode << '|' << motion.twist.linear.x << '|'
               << motion.twist.linear.y << '|' << motion.twist.linear.z << '|' << motion.twist.angular.x << '|'
               << motion.twist.angular.y << '|' << motion.twist.angular.z << '|' << motion.speed;
        for (const auto& waypoint : motion.waypoints) {
            stream << '|' << waypoint.x << '|' << waypoint.y << '|' << waypoint.z;
        }
    }
    return stream.str();
}

std::string StopFingerprint(const xgc2_gazebo_scene::StopMotions::Request& request) {
    std::ostringstream stream;
    stream << request.expected_scene_revision;
    for (const auto& name : request.names) {
        stream << '|' << name;
    }
    return stream.str();
}

MotionConfiguration ConvertMotion(const MotionSpec& message, std::string* error) {
    MotionConfiguration configuration;
    if (!ParseMotionMode(message.mode, &configuration.mode)) {
        *error = "unsupported motion mode for " + message.name + ": " + message.mode;
        return configuration;
    }
    configuration.linear_velocity.Set(message.twist.linear.x, message.twist.linear.y, message.twist.linear.z);
    configuration.angular_velocity.Set(message.twist.angular.x, message.twist.angular.y, message.twist.angular.z);
    configuration.speed = message.speed;
    for (const auto& waypoint : message.waypoints) {
        configuration.waypoints.emplace_back(waypoint.x, waypoint.y, waypoint.z);
    }
    return configuration;
}

} // namespace

class GazeboSceneSystemPlugin final : public gazebo::SystemPlugin {
  public:
    GazeboSceneSystemPlugin() = default;

    ~GazeboSceneSystemPlugin() override {
        if (spinner_) {
            spinner_->stop();
        }
        contact_subscriber_.reset();
        gazebo_transport_node_.reset();
        world_reset_connection_.reset();
        update_connection_.reset();
        world_created_connection_.reset();
        node_.reset();
    }

    void Load(int /*argc*/, char** /*argv*/) override {
        world_created_connection_ = gazebo::event::Events::ConnectWorldCreated(
            std::bind(&GazeboSceneSystemPlugin::OnWorldCreated, this, std::placeholders::_1));
    }

  private:
    struct ManagedObstacle {
        gazebo::physics::ModelPtr model;
        uint64_t generation = 0;
        ObstacleDefinition definition;
        ignition::math::Pose3d observed_pose = ignition::math::Pose3d::Zero;
        MotionController controller;
        bool controlled = false;
        bool has_commanded_pose = false;
        ignition::math::Pose3d commanded_pose = ignition::math::Pose3d::Zero;
        uint64_t motion_revision = 0;
    };

    struct CachedConfigure {
        std::string fingerprint;
        ConfigureMotions::Response response;
    };

    struct CachedStop {
        std::string fingerprint;
        StopMotions::Response response;
    };

    void OnWorldCreated(const std::string& world_name) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (world_) {
            ROS_ERROR("XGC Gazebo Scene supports one world per gzserver process");
            return;
        }
        world_ = gazebo::physics::get_world(world_name);
        if (!world_) {
            gzerr << "XGC Gazebo Scene could not resolve world " << world_name << "\n";
            return;
        }
        if (!ros::isInitialized()) {
            gzerr << "XGC Gazebo Scene requires gazebo_ros_api_plugin to load first\n";
            world_.reset();
            return;
        }

        scene_epoch_ = world_name + ":" + std::to_string(ros::WallTime::now().toNSec());
        node_ = std::make_unique<ros::NodeHandle>("xgc2_gazebo_scene");
        node_->param<std::string>("managed_obstacle_prefix", managed_obstacle_prefix_, kDefaultManagedPrefix);
        if (managed_obstacle_prefix_.empty()) {
            ROS_ERROR("XGC Gazebo Scene managed_obstacle_prefix must not be empty; using %s", kDefaultManagedPrefix);
            managed_obstacle_prefix_ = kDefaultManagedPrefix;
        }
        node_->param("physical_contacts/enabled", physical_contact_monitor_enabled_, true);
        if (!node_->getParam("physical_contacts/tracked_model_prefixes", tracked_model_prefixes_)) {
            tracked_model_prefixes_.clear();
        }
        geometry_publisher_ = node_->advertise<ObstacleDefinitionArray>(kGeometryTopic, 1, true);
        state_publisher_ = node_->advertise<ObstacleStateArray>(kStateTopic, 1, true);
        geometry_library_publisher_ =
            node_->advertise<xgc2_geometry_msgs::GeometryLibrary>(kGeometryLibraryTopic, 1, true);
        instances_publisher_ = node_->advertise<xgc2_geometry_msgs::ConvexBodyArray>(kInstancesTopic, 1, true);
        if (physical_contact_monitor_enabled_) {
            physical_collision_publisher_ = node_->advertise<std_msgs::Bool>(kPhysicalCollisionTopic, 1, true);
            physical_collision_detail_publisher_ =
                node_->advertise<std_msgs::String>(kPhysicalCollisionDetailTopic, 1, true);
            PublishPhysicalCollision(false, "");
            // A Gazebo transport subscriber makes ContactManager retain the
            // engine contacts without forcing every consumer to ingest the
            // high-volume raw wheel/ground stream.
            gazebo_transport_node_.reset(new gazebo::transport::Node());
            gazebo_transport_node_->Init(world_name);
            contact_subscriber_ =
                gazebo_transport_node_->Subscribe("~/physics/contacts", &GazeboSceneSystemPlugin::OnContacts, this);
            world_reset_connection_ =
                gazebo::event::Events::ConnectWorldReset(std::bind(&GazeboSceneSystemPlugin::OnWorldReset, this));
        }
        configure_service_ =
            node_->advertiseService(kConfigureService, &GazeboSceneSystemPlugin::ConfigureMotionsCallback, this);
        stop_service_ = node_->advertiseService(kStopService, &GazeboSceneSystemPlugin::StopMotionsCallback, this);
        spinner_ = std::make_unique<ros::AsyncSpinner>(1);
        spinner_->start();
        update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&GazeboSceneSystemPlugin::OnUpdate, this, std::placeholders::_1));
        ROS_INFO("XGC Gazebo Scene attached to world %s", world_name.c_str());
    }

    void PublishPhysicalCollision(bool collision, const std::string& detail) {
        // Publish detail first so a simultaneously subscribed regression
        // oracle can include it in a true verdict. Publishing an empty detail
        // also clears the old latched reason after a world reset.
        std_msgs::String message;
        message.data = detail;
        physical_collision_detail_publisher_.publish(message);
        std_msgs::Bool status;
        status.data = collision;
        physical_collision_publisher_.publish(status);
    }

    void OnWorldReset() {
        std::lock_guard<std::mutex> lock(mutex_);
        physical_collision_detected_ = false;
        PublishPhysicalCollision(false, "");
    }

    void OnContacts(ConstContactsPtr& contacts) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (physical_collision_detected_ || !contacts) {
            return;
        }

        for (int index = 0; index < contacts->contact_size(); ++index) {
            const gazebo::msgs::Contact& contact = contacts->contact(index);
            const std::string first_name = ModelNameFromScopedCollision(contact.collision1());
            const std::string second_name = ModelNameFromScopedCollision(contact.collision2());
            const auto first_found = contact_models_.find(first_name);
            const auto second_found = contact_models_.find(second_name);
            if (first_found == contact_models_.end() || second_found == contact_models_.end()) {
                continue;
            }
            const ContactModelDescriptor& first = first_found->second;
            const ContactModelDescriptor& second = second_found->second;
            if (!IsForbiddenPhysicalContact(first, second, tracked_model_prefixes_)) {
                continue;
            }

            double maximum_depth = 0.0;
            for (int point = 0; point < contact.depth_size(); ++point) {
                maximum_depth = std::max(maximum_depth, contact.depth(point));
            }
            std::ostringstream detail;
            detail << "sim_time=" << contacts->time().sec() << "." << std::setfill('0') << std::setw(9)
                   << contacts->time().nsec() << std::setfill(' ') << " model1=" << first.name
                   << " collision1=" << contact.collision1() << " model2=" << second.name
                   << " collision2=" << contact.collision2() << " points=" << contact.position_size()
                   << " max_depth=" << maximum_depth;
            physical_collision_detected_ = true;
            PublishPhysicalCollision(true, detail.str());
            ROS_ERROR("XGC Gazebo Scene detected forbidden physical contact: %s", detail.str().c_str());
            return;
        }
    }

    void RefreshContactModels() {
        contact_models_.clear();
        for (const auto& model : world_->Models()) {
            ContactModelDescriptor descriptor;
            descriptor.name = model->GetName();
            descriptor.is_static = model->IsStatic();
            descriptor.is_managed_obstacle = !LogicalName(descriptor.name, managed_obstacle_prefix_).empty();
            contact_models_.emplace(descriptor.name, std::move(descriptor));
        }
    }

    bool BuildDefinition(const std::string& logical_name, uint64_t generation, const gazebo::physics::ModelPtr& model,
                         ObstacleDefinition* definition, std::string* error) {
        *definition = ObstacleDefinition{};
        error->clear();
        definition->name = logical_name;
        definition->model_name = model->GetName();
        definition->generation = generation;

        for (const auto& link : model->GetLinks()) {
            for (const auto& collision : link->GetCollisions()) {
                ConvexPart part;
                part.part_id = link->GetName() + "/" + collision->GetName();
                part.local_pose = PoseMessage(link->RelativePose() * collision->RelativePose());
                const gazebo::physics::ShapePtr shape = collision->GetShape();
                if (const auto box = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(shape)) {
                    part.shape = ConvexPart::SHAPE_BOX;
                    const ignition::math::Vector3d size = box->Size();
                    part.size.x = size.X();
                    part.size.y = size.Y();
                    part.size.z = size.Z();
                    AppendBoxVertices(size, &part.conservative_vertices);
                } else if (const auto sphere = boost::dynamic_pointer_cast<gazebo::physics::SphereShape>(shape)) {
                    part.shape = ConvexPart::SHAPE_SPHERE;
                    part.radius = sphere->GetRadius();
                    AppendSphereVertices(part.radius, &part.conservative_vertices);
                } else if (const auto cylinder = boost::dynamic_pointer_cast<gazebo::physics::CylinderShape>(shape)) {
                    part.shape = ConvexPart::SHAPE_CYLINDER;
                    part.radius = cylinder->GetRadius();
                    part.length = cylinder->GetLength();
                    AppendCylinderVertices(part.radius, part.length, &part.conservative_vertices);
                } else if (const auto mesh = boost::dynamic_pointer_cast<gazebo::physics::MeshShape>(shape)) {
                    gazebo::msgs::Geometry geometry_message;
                    mesh->FillMsg(geometry_message);
                    const gazebo::msgs::MeshGeom& mesh_message = geometry_message.mesh();
                    const std::string submesh_name = mesh_message.has_submesh() ? mesh_message.submesh() : "";
                    const bool center_submesh = mesh_message.has_center_submesh() && mesh_message.center_submesh();
                    ignition::math::Vector3d collision_scale = mesh->Scale();
                    const sdf::ElementPtr collision_sdf = collision->GetSDF();
                    if (collision_sdf && collision_sdf->HasElement("geometry")) {
                        const sdf::ElementPtr geometry_sdf = collision_sdf->GetElement("geometry");
                        if (geometry_sdf->HasElement("mesh")) {
                            const sdf::ElementPtr mesh_sdf = geometry_sdf->GetElement("mesh");
                            if (mesh_sdf->HasElement("scale")) {
                                collision_scale = mesh_sdf->Get<ignition::math::Vector3d>("scale");
                            }
                        }
                    }
                    ConvexMeshGeometry mesh_geometry;
                    std::string mesh_error;
                    if (!LoadConvexMeshGeometry(mesh->GetMeshURI(), collision_scale, submesh_name, center_submesh,
                                                &mesh_geometry, &mesh_error)) {
                        *error = "collision " + part.part_id + " is not a publishable convex mesh: " + mesh_error;
                        return false;
                    }
                    part.shape = ConvexPart::SHAPE_CONVEX_MESH;
                    part.mesh_uri = mesh_geometry.uri;
                    part.mesh_submesh = submesh_name;
                    part.mesh_center_submesh = center_submesh;
                    part.mesh_scale.x = mesh_geometry.scale.X();
                    part.mesh_scale.y = mesh_geometry.scale.Y();
                    part.mesh_scale.z = mesh_geometry.scale.Z();
                    for (const auto& vertex : mesh_geometry.vertices) {
                        part.conservative_vertices.push_back(PointMessage(vertex));
                    }
                } else {
                    *error = "collision " + part.part_id + " has an unsupported shape";
                    return false;
                }
                definition->parts.push_back(std::move(part));
            }
        }
        if (definition->parts.empty()) {
            *error = "managed obstacle contains no collision geometry";
            return false;
        }
        return true;
    }

    bool DiscoverObstacles(double simulation_time) {
        std::map<std::string, gazebo::physics::ModelPtr> current;
        for (const auto& model : world_->Models()) {
            const std::string logical_name = LogicalName(model->GetName(), managed_obstacle_prefix_);
            if (!logical_name.empty()) {
                current.emplace(logical_name, model);
            }
        }

        bool changed = false;
        for (auto iterator = obstacles_.begin(); iterator != obstacles_.end();) {
            const auto found = current.find(iterator->first);
            if (found == current.end() || found->second != iterator->second.model) {
                iterator = obstacles_.erase(iterator);
                ++scene_revision_;
                changed = true;
            } else {
                ++iterator;
            }
        }
        for (const auto& item : current) {
            if (obstacles_.find(item.first) != obstacles_.end()) {
                continue;
            }
            ManagedObstacle obstacle;
            obstacle.model = item.second;
            obstacle.generation = generation_counters_[item.first] + 1;
            obstacle.observed_pose = item.second->WorldPose();
            std::string error;
            MotionConfiguration hold;
            if (!obstacle.controller.Configure(hold, obstacle.observed_pose, simulation_time, &error)) {
                ROS_ERROR("Cannot initialize obstacle controller: %s", error.c_str());
            }
            if (!BuildDefinition(item.first, obstacle.generation, item.second, &obstacle.definition, &error)) {
                ROS_ERROR_THROTTLE(5.0, "Managed obstacle %s was rejected: %s", item.second->GetName().c_str(),
                                   error.c_str());
                continue;
            }
            generation_counters_[item.first] = obstacle.generation;
            obstacles_.emplace(item.first, std::move(obstacle));
            ++scene_revision_;
            changed = true;
        }
        return changed;
    }

    void OnUpdate(const gazebo::common::UpdateInfo& info) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!world_) {
            return;
        }
        const double simulation_time = info.simTime.Double();
        RefreshContactModels();
        const bool geometry_changed = DiscoverObstacles(simulation_time);

        for (auto& item : obstacles_) {
            ManagedObstacle& obstacle = item.second;
            const ignition::math::Pose3d current_pose = obstacle.model->WorldPose();
            if (obstacle.controlled && obstacle.has_commanded_pose &&
                !PoseNearlyEqual(current_pose, obstacle.commanded_pose)) {
                obstacle.controlled = false;
                obstacle.has_commanded_pose = false;
                obstacle.observed_pose = current_pose;
                ++obstacle.motion_revision;
                ++scene_revision_;
                ROS_INFO("External pose update took control of managed obstacle %s", item.first.c_str());
            }
            if (obstacle.controlled) {
                const MotionSample sample = obstacle.controller.Sample(simulation_time);
                // Drive pose kinematically each step. Zero residual twist so Gazebo
                // does not integrate past the commanded pose between updates and
                // falsely trip the external-control detector.
                obstacle.model->SetWorldPose(sample.pose);
                obstacle.model->SetWorldTwist(ignition::math::Vector3d::Zero, ignition::math::Vector3d::Zero);
                obstacle.commanded_pose = obstacle.model->WorldPose();
                obstacle.has_commanded_pose = true;
            }
            obstacle.observed_pose = obstacle.model->WorldPose();
        }

        if (geometry_changed || !geometry_published_) {
            PublishGeometry(info.simTime);
        }
        if (last_publish_time_ < 0.0 || simulation_time + 1e-9 >= last_publish_time_ + kPublishPeriod ||
            simulation_time < last_publish_time_) {
            PublishState(info.simTime);
            last_publish_time_ = simulation_time;
        }
    }

    void PublishGeometry(const gazebo::common::Time& simulation_time) {
        ObstacleDefinitionArray message;
        message.header.stamp = ros::Time(simulation_time.sec, simulation_time.nsec);
        message.header.frame_id = "world";
        message.scene_epoch = scene_epoch_;
        message.scene_revision = scene_revision_;
        for (const auto& item : obstacles_) {
            message.obstacles.push_back(item.second.definition);
        }
        geometry_publisher_.publish(message);

        xgc2_geometry_msgs::GeometryLibrary library;
        library.header = message.header;
        std::map<std::string, xgc2_geometry_msgs::GeometryTemplate> templates;
        std::map<std::string, std::string> v_polytope_alias_sources;
        std::set<std::string> ambiguous_v_polytope_aliases;
        for (const auto& item : obstacles_) {
            for (const auto& part : item.second.definition.parts) {
                const std::string type = StandardGeometryType(part);
                if (!type.empty()) {
                    const xgc2_geometry_msgs::GeometryTemplate standard_template = StandardGeometryTemplate(part);
                    templates.emplace(type, standard_template);

                    const std::string v_polytope_alias = VPolytopeGeometryTypeAlias(part);
                    if (!v_polytope_alias.empty() && ambiguous_v_polytope_aliases.count(v_polytope_alias) == 0) {
                        const auto alias_source = v_polytope_alias_sources.emplace(v_polytope_alias, type);
                        if (!alias_source.second && alias_source.first->second != type) {
                            templates.erase(v_polytope_alias);
                            ambiguous_v_polytope_aliases.insert(v_polytope_alias);
                            ROS_ERROR("Not publishing ambiguous geometry alias '%s': "
                                      "both '%s' and '%s' use that mesh filename",
                                      v_polytope_alias.c_str(), alias_source.first->second.c_str(), type.c_str());
                            continue;
                        }
                        xgc2_geometry_msgs::GeometryTemplate alias_template = standard_template;
                        alias_template.type = v_polytope_alias;
                        templates.emplace(v_polytope_alias, std::move(alias_template));
                    }
                }
            }
        }
        for (const auto& item : templates) {
            library.templates.push_back(item.second);
        }
        geometry_library_publisher_.publish(library);
        geometry_published_ = true;
    }

    void PublishState(const gazebo::common::Time& simulation_time) {
        ObstacleStateArray message;
        message.header.stamp = ros::Time(simulation_time.sec, simulation_time.nsec);
        message.header.frame_id = "world";
        message.scene_epoch = scene_epoch_;
        message.scene_revision = scene_revision_;
        for (const auto& item : obstacles_) {
            const ManagedObstacle& obstacle = item.second;
            ObstacleState state;
            state.name = item.first;
            state.model_name = obstacle.model->GetName();
            state.generation = obstacle.generation;
            state.pose = PoseMessage(obstacle.observed_pose);
            if (obstacle.controlled) {
                const MotionSample sample = obstacle.controller.Sample(simulation_time.Double());
                state.twist = TwistMessage(sample.linear_velocity, sample.angular_velocity);
                state.motion_mode = obstacle.controller.modeName();
            } else {
                state.twist = TwistMessage(obstacle.model->WorldLinearVel(), obstacle.model->WorldAngularVel());
                state.motion_mode = "uncontrolled";
            }
            state.motion_revision = obstacle.motion_revision;
            message.obstacles.push_back(std::move(state));
        }
        state_publisher_.publish(message);

        xgc2_geometry_msgs::ConvexBodyArray instances;
        instances.header = message.header;
        std::int32_t instance_id = 1;
        for (const auto& item : obstacles_) {
            const ManagedObstacle& obstacle = item.second;
            ignition::math::Vector3d linear_velocity;
            ignition::math::Vector3d angular_velocity;
            if (obstacle.controlled) {
                const MotionSample sample = obstacle.controller.Sample(simulation_time.Double());
                linear_velocity = sample.linear_velocity;
                angular_velocity = sample.angular_velocity;
            } else {
                linear_velocity = obstacle.model->WorldLinearVel();
                angular_velocity = obstacle.model->WorldAngularVel();
            }
            const bool is_static =
                obstacle.model->IsStatic() && (!obstacle.controlled || obstacle.controller.modeName() == "hold");
            const bool single_part = obstacle.definition.parts.size() == 1;
            for (const auto& part : obstacle.definition.parts) {
                const ignition::math::Pose3d local_pose = IgnitionPose(part.local_pose);
                const ignition::math::Pose3d world_pose = obstacle.observed_pose * local_pose;
                const ignition::math::Vector3d offset = obstacle.observed_pose.Rot().RotateVector(local_pose.Pos());
                xgc2_geometry_msgs::ConvexBodyInstance instance;
                instance.id = instance_id++;
                instance.name = single_part ? item.first : item.first + "/" + part.part_id;
                instance.geometry_type = StandardGeometryType(part);
                instance.pose = PoseMessage(world_pose);
                instance.scale = StandardInstanceScale(part);
                instance.is_static = is_static;
                instance.velocity = TwistMessage(linear_velocity + angular_velocity.Cross(offset), angular_velocity);
                instances.instances.push_back(std::move(instance));
            }
        }
        instances_publisher_.publish(instances);
    }

    bool ConfigureMotionsCallback(ConfigureMotions::Request& request, ConfigureMotions::Response& response) {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::string fingerprint = ConfigureFingerprint(request);
        const auto cached = configured_commands_.find(request.command_id);
        if (cached != configured_commands_.end()) {
            if (cached->second.fingerprint != fingerprint) {
                response.success = false;
                response.message = "command_id was reused with different parameters";
                response.scene_revision = scene_revision_;
                return true;
            }
            response = cached->second.response;
            return true;
        }
        if (request.command_id.empty() || request.command_id.size() > 128) {
            response.success = false;
            response.message = "command_id must contain 1 to 128 characters";
            response.scene_revision = scene_revision_;
            return true;
        }
        if (request.expected_scene_revision != 0 && request.expected_scene_revision != scene_revision_) {
            response.success = false;
            response.message = "scene revision conflict";
            response.scene_revision = scene_revision_;
            return true;
        }
        if (request.motions.empty()) {
            response.success = false;
            response.message = "at least one motion is required";
            response.scene_revision = scene_revision_;
            return true;
        }

        std::set<std::string> names;
        std::vector<std::pair<std::string, MotionController>> prepared;
        prepared.reserve(request.motions.size());
        for (const auto& motion : request.motions) {
            if (!names.insert(motion.name).second) {
                response.success = false;
                response.message = "duplicate obstacle name: " + motion.name;
                response.scene_revision = scene_revision_;
                return true;
            }
            const auto obstacle = obstacles_.find(motion.name);
            if (obstacle == obstacles_.end()) {
                response.success = false;
                response.message = "managed obstacle does not exist: " + motion.name;
                response.scene_revision = scene_revision_;
                return true;
            }
            if (IsSceneRuntimeModel(obstacle->second.model->GetName())) {
                response.success = false;
                response.message = "motion belongs to the scene runtime: " + motion.name;
                response.scene_revision = scene_revision_;
                return true;
            }
            std::string error;
            const MotionConfiguration configuration = ConvertMotion(motion, &error);
            if (!error.empty()) {
                response.success = false;
                response.message = error;
                response.scene_revision = scene_revision_;
                return true;
            }
            MotionController controller;
            if (!controller.Configure(configuration, obstacle->second.observed_pose, world_->SimTime().Double(),
                                      &error)) {
                response.success = false;
                response.message = error;
                response.scene_revision = scene_revision_;
                return true;
            }
            prepared.emplace_back(motion.name, std::move(controller));
        }

        ++scene_revision_;
        for (auto& item : prepared) {
            ManagedObstacle& obstacle = obstacles_.at(item.first);
            obstacle.controller = std::move(item.second);
            obstacle.controlled = true;
            obstacle.has_commanded_pose = false;
            ++obstacle.motion_revision;
            response.motion_revisions.push_back(obstacle.motion_revision);
        }
        response.success = true;
        response.message = "configured " + std::to_string(prepared.size()) + " obstacle motions";
        response.scene_revision = scene_revision_;
        configured_commands_.emplace(request.command_id, CachedConfigure{fingerprint, response});
        return true;
    }

    bool StopMotionsCallback(StopMotions::Request& request, StopMotions::Response& response) {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::string fingerprint = StopFingerprint(request);
        const auto cached = stopped_commands_.find(request.command_id);
        if (cached != stopped_commands_.end()) {
            if (cached->second.fingerprint != fingerprint) {
                response.success = false;
                response.message = "command_id was reused with different parameters";
                response.scene_revision = scene_revision_;
                return true;
            }
            response = cached->second.response;
            return true;
        }
        if (request.command_id.empty() || request.command_id.size() > 128) {
            response.success = false;
            response.message = "command_id must contain 1 to 128 characters";
            response.scene_revision = scene_revision_;
            return true;
        }
        if (request.expected_scene_revision != 0 && request.expected_scene_revision != scene_revision_) {
            response.success = false;
            response.message = "scene revision conflict";
            response.scene_revision = scene_revision_;
            return true;
        }

        std::set<std::string> names(request.names.begin(), request.names.end());
        if (names.empty()) {
            for (const auto& obstacle : obstacles_) {
                if (!IsSceneRuntimeModel(obstacle.second.model->GetName()))
                    names.insert(obstacle.first);
            }
        }
        for (const auto& name : names) {
            if (obstacles_.find(name) == obstacles_.end()) {
                response.success = false;
                response.message = "managed obstacle does not exist: " + name;
                response.scene_revision = scene_revision_;
                return true;
            }
        }

        for (const auto& name : names) {
            if (IsSceneRuntimeModel(obstacles_.at(name).model->GetName())) {
                response.success = false;
                response.message = "motion belongs to the scene runtime: " + name;
                response.scene_revision = scene_revision_;
                return true;
            }
        }

        ++scene_revision_;
        for (const auto& name : names) {
            ManagedObstacle& obstacle = obstacles_.at(name);
            MotionConfiguration hold;
            std::string error;
            if (!obstacle.controller.Configure(hold, obstacle.observed_pose, world_->SimTime().Double(), &error)) {
                response.success = false;
                response.message = error;
                response.scene_revision = scene_revision_;
                return true;
            }
            obstacle.controlled = true;
            obstacle.has_commanded_pose = false;
            ++obstacle.motion_revision;
            response.motion_revisions.push_back(obstacle.motion_revision);
        }
        response.success = true;
        response.message = "stopped " + std::to_string(names.size()) + " obstacle motions";
        response.scene_revision = scene_revision_;
        stopped_commands_.emplace(request.command_id, CachedStop{fingerprint, response});
        return true;
    }

    std::mutex mutex_;
    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr world_created_connection_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::event::ConnectionPtr world_reset_connection_;
    gazebo::transport::NodePtr gazebo_transport_node_;
    gazebo::transport::SubscriberPtr contact_subscriber_;
    std::unique_ptr<ros::NodeHandle> node_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
    ros::Publisher geometry_publisher_;
    ros::Publisher state_publisher_;
    ros::Publisher geometry_library_publisher_;
    ros::Publisher instances_publisher_;
    ros::Publisher physical_collision_publisher_;
    ros::Publisher physical_collision_detail_publisher_;
    ros::ServiceServer configure_service_;
    ros::ServiceServer stop_service_;
    std::map<std::string, ManagedObstacle> obstacles_;
    std::map<std::string, uint64_t> generation_counters_;
    std::map<std::string, CachedConfigure> configured_commands_;
    std::map<std::string, CachedStop> stopped_commands_;
    std::map<std::string, ContactModelDescriptor> contact_models_;
    std::string scene_epoch_;
    std::string managed_obstacle_prefix_ = kDefaultManagedPrefix;
    std::vector<std::string> tracked_model_prefixes_;
    uint64_t scene_revision_ = 0;
    double last_publish_time_ = -1.0;
    bool geometry_published_ = false;
    bool physical_contact_monitor_enabled_ = true;
    bool physical_collision_detected_ = false;
};

GZ_REGISTER_SYSTEM_PLUGIN(GazeboSceneSystemPlugin)

} // namespace xgc2_gazebo_scene
