#include <boost/weak_ptr.hpp>
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

#include "xgc2_gazebo_scene/convex_mesh_geometry.hpp"
#include "xgc2_gazebo_scene/model_snapshot.hpp"
#include "xgc2_gazebo_scene/obstacle_messages.hpp"
#include "xgc2_gazebo_scene/physical_contact_filter.hpp"
#include "xgc2_gazebo_scene/scene_ownership.hpp"
#include "xgc2_geometry_msgs/ConvexBodyArray.h"
#include "xgc2_geometry_msgs/ConvexBodyInstance.h"
#include "xgc2_geometry_msgs/GeometryLibrary.h"
#include "xgc2_geometry_msgs/GeometryTemplate.h"

namespace xgc2_gazebo_scene {
namespace {

constexpr char kDefaultManagedPrefix[] = "xgc2_obstacle_";
constexpr char kGeometryLibraryTopic[] = "/xgc2/simulation/obstacles/geometry_library";
constexpr char kInstancesTopic[] = "/xgc2/simulation/obstacles/instances";
constexpr char kPhysicalCollisionTopic[] = "/xgc2/simulation/physical_collision";
constexpr char kPhysicalCollisionDetailTopic[] = "/xgc2/simulation/physical_collision_detail";
constexpr double kPublishPeriod = 1.0 / 30.0;

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

std::string VPolytopeGeometryTypeAlias(const std::string& mesh_uri) {
    // The GeometryLibrary contract names a V-polytope template after its
    // source template (for example "v_polytope:ugv_hexagon_1"). Gazebo
    // collision meshes carry the same stable name in the asset filename. Keep
    // the URI type canonical for obstacle instances, and publish this semantic
    // alias so non-obstacle consumers can share the exact collision-derived
    // support points.
    std::string filename = mesh_uri;
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

} // namespace

class GazeboSceneSystemPlugin final : public gazebo::SystemPlugin {
  public:
    GazeboSceneSystemPlugin() = default;

    ~GazeboSceneSystemPlugin() override {
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
        std::vector<xgc2_geometry_msgs::ConvexBodyInstance> parts;
        std::vector<std::pair<xgc2_geometry_msgs::GeometryTemplate, std::string>> geometry_templates;
        ignition::math::Pose3d observed_pose = ignition::math::Pose3d::Zero;
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
            gzerr << "XGC Gazebo Scene requires the native ROS user-data plugin to load first\n";
            world_.reset();
            return;
        }

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

    void RefreshContactModels(const gazebo::physics::Model_V& models) {
        contact_models_.clear();
        for (const auto& model : models) {
            ContactModelDescriptor descriptor;
            descriptor.name = model->GetName();
            descriptor.is_static = model->IsStatic();
            descriptor.is_managed_obstacle = !LogicalName(descriptor.name, managed_obstacle_prefix_).empty();
            contact_models_.emplace(descriptor.name, std::move(descriptor));
        }
    }

    bool BuildGeometry(ManagedObstacle* obstacle, std::string* error) {
        error->clear();

        for (const auto& link : obstacle->model->GetLinks()) {
            for (const auto& collision : link->GetCollisions()) {
                xgc2_geometry_msgs::ConvexBodyInstance part;
                xgc2_geometry_msgs::GeometryTemplate geometry_template;
                std::string v_polytope_alias;
                part.name = link->GetName() + "/" + collision->GetName();
                part.pose = PoseMessage(link->RelativePose() * collision->RelativePose());
                const gazebo::physics::ShapePtr shape = collision->GetShape();
                if (const auto box = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(shape)) {
                    part.geometry_type = "cube";
                    part.scale = VectorMessage(box->Size());
                    AppendBoxVertices(ignition::math::Vector3d::One, &geometry_template.support_points);
                } else if (const auto sphere = boost::dynamic_pointer_cast<gazebo::physics::SphereShape>(shape)) {
                    part.geometry_type = "sphere";
                    const double radius = sphere->GetRadius();
                    part.scale = VectorMessage({radius, radius, radius});
                } else if (const auto cylinder = boost::dynamic_pointer_cast<gazebo::physics::CylinderShape>(shape)) {
                    part.geometry_type = "cylinder";
                    part.scale = VectorMessage({cylinder->GetRadius(), cylinder->GetRadius(), cylinder->GetLength()});
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
                        *error = "collision " + part.name + " is not a publishable convex mesh: " + mesh_error;
                        return false;
                    }
                    part.geometry_type = "convex_mesh:" + mesh_geometry.uri;
                    if (!submesh_name.empty()) {
                        part.geometry_type += "#submesh=" + submesh_name;
                    }
                    if (center_submesh) {
                        part.geometry_type += "#centered";
                    }
                    if (submesh_name.empty() && !center_submesh) {
                        v_polytope_alias = VPolytopeGeometryTypeAlias(mesh_geometry.uri);
                    }
                    part.scale = VectorMessage(mesh_geometry.scale);
                    for (const auto& vertex : mesh_geometry.vertices) {
                        geometry_template.support_points.push_back(PointMessage(
                            {vertex.X() / part.scale.x, vertex.Y() / part.scale.y, vertex.Z() / part.scale.z}));
                    }
                } else {
                    *error = "collision " + part.name + " has an unsupported shape";
                    return false;
                }
                geometry_template.type = part.geometry_type;
                geometry_template.resolution = 0;
                obstacle->parts.push_back(std::move(part));
                obstacle->geometry_templates.emplace_back(std::move(geometry_template), std::move(v_polytope_alias));
            }
        }
        if (obstacle->parts.empty()) {
            *error = "managed obstacle contains no collision geometry";
            return false;
        }
        return true;
    }

    bool DiscoverObstacles(const gazebo::physics::Model_V& models) {
        rediscover_ = false;
        std::map<std::string, gazebo::physics::ModelPtr> current;
        for (const auto& model : models) {
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
            obstacle.observed_pose = item.second->WorldPose();
            std::string error;
            if (!BuildGeometry(&obstacle, &error)) {
                ROS_ERROR_THROTTLE(5.0, "Managed obstacle %s was rejected: %s", item.second->GetName().c_str(),
                                   error.c_str());
                // Retried, and reported, on every update while it is present.
                rediscover_ = true;
                continue;
            }
            obstacles_.emplace(item.first, std::move(obstacle));
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
        // The contact table and the managed obstacle set are functions of the
        // world's model list (objects, names, static flags). Rebuilding them
        // copies every name into fresh maps, on every update, for every model
        // in the world; when the list is unchanged the rebuild yields the same
        // state, so it runs only when the list changed or a managed model is
        // still rejected.
        const gazebo::physics::Model_V models = world_->Models();
        bool geometry_changed = false;
        if (rediscover_ || !model_snapshot_.Matches(models)) {
            RefreshContactModels(models);
            geometry_changed = DiscoverObstacles(models);
            obstacle_messages_stale_ = obstacle_messages_stale_ || geometry_changed;
            model_snapshot_.Update(models);
        }

        for (auto& item : obstacles_) {
            ManagedObstacle& obstacle = item.second;
            obstacle.observed_pose = obstacle.model->WorldPose();
        }

        if (geometry_changed || !geometry_published_) {
            PublishGeometry(info.simTime);
        }
        if (last_publish_time_ < 0.0 || simulation_time + 1e-9 >= last_publish_time_ + kPublishPeriod ||
            simulation_time < last_publish_time_) {
            PublishInstances(info.simTime);
            last_publish_time_ = simulation_time;
        }
    }

    void PublishGeometry(const gazebo::common::Time& simulation_time) {
        xgc2_geometry_msgs::GeometryLibrary library;
        library.header.stamp = ros::Time(simulation_time.sec, simulation_time.nsec);
        library.header.frame_id = "world";
        std::map<std::string, xgc2_geometry_msgs::GeometryTemplate> templates;
        std::map<std::string, std::string> v_polytope_alias_sources;
        std::set<std::string> ambiguous_v_polytope_aliases;
        for (const auto& item : obstacles_) {
            for (const auto& geometry : item.second.geometry_templates) {
                const auto& standard_template = geometry.first;
                const std::string& type = standard_template.type;
                if (!type.empty()) {
                    templates.emplace(type, standard_template);

                    const std::string& v_polytope_alias = geometry.second;
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

    void PublishInstances(const gazebo::common::Time& simulation_time) {
        // Discovery marks the messages stale whenever the set changes; the count
        // check keeps Set() inside the messages should that ever be missed.
        if (obstacle_messages_stale_ || obstacle_messages_.size() != obstacles_.size()) {
            std::vector<ObstacleMessages::Obstacle> fixed;
            fixed.reserve(obstacles_.size());
            for (const auto& item : obstacles_) {
                fixed.push_back({item.first, &item.second.parts});
            }
            obstacle_messages_.Reset(fixed);
            obstacle_messages_stale_ = false;
        }
        obstacle_messages_.Begin(ros::Time(simulation_time.sec, simulation_time.nsec));
        ObstacleDynamics dynamics;
        std::size_t index = 0;
        for (const auto& item : obstacles_) {
            const ManagedObstacle& obstacle = item.second;
            dynamics.pose = obstacle.observed_pose;
            dynamics.linear_velocity = obstacle.model->WorldLinearVel();
            dynamics.angular_velocity = obstacle.model->WorldAngularVel();
            dynamics.is_static = obstacle.model->IsStatic();
            obstacle_messages_.Set(index++, dynamics);
        }
        instances_publisher_.publish(obstacle_messages_.instances());
    }

    std::mutex mutex_;
    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr world_created_connection_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::event::ConnectionPtr world_reset_connection_;
    gazebo::transport::NodePtr gazebo_transport_node_;
    gazebo::transport::SubscriberPtr contact_subscriber_;
    std::unique_ptr<ros::NodeHandle> node_;
    ros::Publisher geometry_library_publisher_;
    ros::Publisher instances_publisher_;
    ros::Publisher physical_collision_publisher_;
    ros::Publisher physical_collision_detail_publisher_;
    std::map<std::string, ManagedObstacle> obstacles_;
    std::map<std::string, ContactModelDescriptor> contact_models_;
    ObstacleMessages obstacle_messages_;
    bool obstacle_messages_stale_ = true;
    ModelSnapshot<gazebo::physics::ModelPtr, boost::weak_ptr<gazebo::physics::Model>> model_snapshot_;
    bool rediscover_ = true;
    std::string managed_obstacle_prefix_ = kDefaultManagedPrefix;
    std::vector<std::string> tracked_model_prefixes_;
    double last_publish_time_ = -1.0;
    bool geometry_published_ = false;
    bool physical_contact_monitor_enabled_ = true;
    bool physical_collision_detected_ = false;
};

GZ_REGISTER_SYSTEM_PLUGIN(GazeboSceneSystemPlugin)

} // namespace xgc2_gazebo_scene
