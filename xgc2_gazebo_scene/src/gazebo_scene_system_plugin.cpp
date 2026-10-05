#include <boost/weak_ptr.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Exception.hh>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/property_tree/json_parser.hpp>
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
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
#include "xgc2_gazebo_scene/model_snapshot.hpp"
#include "xgc2_gazebo_scene/motion_controller.hpp"
#include "xgc2_gazebo_scene/obstacle_messages.hpp"
#include "xgc2_gazebo_scene/physical_contact_filter.hpp"
#include "xgc2_gazebo_scene/scene_ownership.hpp"
#include "xgc2_gazebo_scene/world_sensor_inputs.hpp"
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

// Main-thread cold failure only. The stock ROS API plugin starts a thread
// that waits for services not advertised until WorldCreated. Its destructor
// joins that thread; NodeHandle::shutdown alone does not cancel the global
// ros::ok() wait. Cancel this server's ROS runtime before the original common
// exception/Fini route. Healthy start and runtime sensor fatal use no such call.
[[noreturn]] void FailSensorColdStart(const std::string& message) {
    if (ros::isStarted()) ros::shutdown();
    gzthrow(message);
}

} // namespace

class GazeboSceneSystemPlugin final : public gazebo::SystemPlugin {
  public:
    GazeboSceneSystemPlugin() = default;

    ~GazeboSceneSystemPlugin() override {
        // No callback may borrow the host after the cold join below. Cancel a
        // fatal connection wait even when transport was stopped before plugins.
        sensor_teardown_.store(true, std::memory_order_release);
        if (sensor_system_) sensor_system_->fence();
        world_reset_connection_.reset();
        update_connection_.reset();
        world_created_connection_.reset();
        contact_subscriber_.reset();
        { std::lock_guard<std::mutex> lock(mutex_); } // finish an entered update before cold join
        for (auto& input : sensor_pose_subscribers_) input.shutdown();
        for (auto& input : sensor_cloud_subscribers_) input.shutdown();
        sensor_geometry_queue_.disable();
        if (spinner_) spinner_->stop();
        if (sensor_system_) sensor_system_->stop(); // cold thread, never fatal caller
        sensor_system_.reset();
        sensor_geometry_node_.reset();
        sensor_control_publisher_.reset();
        gazebo_transport_node_.reset();
        node_.reset();
    }

    void Load(int /*argc*/, char** /*argv*/) override {
        // Existing supervised launcher owns this one immutable private artifact.
        // Absence retains the original legitimate no-attachment Scene behavior.
        const char* path = std::getenv(kWorldSensorInputsEnvironment);
        if (path && *path) {
            try {
                if (*path != '/') throw std::invalid_argument("sensor input file must be absolute");
                std::ifstream file(path);
                if (!file) throw std::runtime_error("cannot open frozen World lidar inputs");
                boost::property_tree::ptree input;
                boost::property_tree::read_json(file, input);
                sensor_inputs_ = std::make_unique<WorldSensorInputs>(ParseWorldSensorInputs(input));
            } catch (const std::exception& error) {
                // Server LoadImpl catches gazebo::common::Exception and routes
                // main through Fini/return-1. std exceptions are not that route.
                FailSensorColdStart(std::string("XGC required sensor cold configuration: ") + error.what());
            }
        }
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
            if (sensor_inputs_) FailSensorColdStart("required World sensors cannot attach to a second World");
            ROS_ERROR("XGC Gazebo Scene supports one world per gzserver process");
            return;
        }
        world_ = gazebo::physics::get_world(world_name);
        if (!world_) {
            if (sensor_inputs_) FailSensorColdStart("required sensor World could not be resolved");
            gzerr << "XGC Gazebo Scene could not resolve world " << world_name << "\n";
            return;
        }
        if (!ros::isInitialized()) {
            if (sensor_inputs_) FailSensorColdStart("required World sensors need gazebo_ros_api_plugin first");
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
        if (sensor_inputs_) {
            try { ConfigureWorldSensors(world_name); }
            catch (const std::exception& error) {
                FailSensorColdStart(std::string("XGC required sensor cold initialization: ") + error.what());
            }
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
            if (!gazebo_transport_node_) {
                gazebo_transport_node_.reset(new gazebo::transport::Node());
                gazebo_transport_node_->Init(world_name);
            }
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


    void ConfigureWorldSensors(const std::string& world_name) {
        const auto& inputs = *sensor_inputs_;
        sensor_sources_.resize(inputs.poses.size());
        sensor_pose_subscribers_.resize(inputs.poses.size());
        sensor_cloud_subscribers_.resize(inputs.clouds.size());
        sensor_cloud_loaded_.assign(inputs.clouds.size(), false);
        auto initial_geometry = std::make_shared<xgc2_world_lidar::WorldSensorGeometry>();
        initial_geometry->shared.resize(inputs.configuration.shared.size());
        sensor_geometry_ = std::move(initial_geometry);
        sensor_outputs_.reserve(inputs.outputs.size());
        for (const auto& topic : inputs.outputs)
            sensor_outputs_.push_back(node_->advertise<sensor_msgs::PointCloud2>(topic, 10));
        if (!gazebo_transport_node_) {
            gazebo_transport_node_.reset(new gazebo::transport::Node());
            gazebo_transport_node_->Init(world_name);
        }
        sensor_control_publisher_ = gazebo_transport_node_->Advertise<gazebo::msgs::ServerControl>(
            "/gazebo/server/control", 1);
        sensor_geometry_node_ = std::make_unique<ros::NodeHandle>(*node_);
        sensor_geometry_node_->setCallbackQueue(&sensor_geometry_queue_);
        for (std::size_t i = 0; i < inputs.poses.size(); ++i) {
            const auto& input = inputs.poses[i];
            if (input.type == "nav_msgs/Odometry")
                sensor_pose_subscribers_[i] = node_->subscribe<nav_msgs::Odometry>(
                    input.topic, 50, [this, i](const nav_msgs::Odometry::ConstPtr& pose) {
                        AcceptSensorPose(i, pose->pose.pose, pose->header.stamp);
                    });
            else
                sensor_pose_subscribers_[i] = node_->subscribe<geometry_msgs::PoseStamped>(
                    input.topic, 50, [this, i](const geometry_msgs::PoseStamped::ConstPtr& pose) {
                        AcceptSensorPose(i, pose->pose, pose->header.stamp);
                    });
        }
        for (std::size_t i = 0; i < inputs.clouds.size(); ++i)
            sensor_cloud_subscribers_[i] = sensor_geometry_node_->subscribe<sensor_msgs::PointCloud2>(
                inputs.clouds[i].topic, 1,
                [this, i](const sensor_msgs::PointCloud2::ConstPtr& message) {
                    // This queue is only serviced by the sensor caller, never
                    // the control spinner or Gazebo's physics update thread.
                    if (sensor_cloud_loaded_[i]) return;
                    const auto& binding = sensor_inputs_->clouds[i];
                    if (!binding.gpu_frame.empty() && message->header.frame_id != binding.gpu_frame)
                        throw std::invalid_argument("GPU static cloud does not use its declared world frame");
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::fromROSMsg(*message, *cloud);
                    // Immutable next pin; never edit the in-flight geometry.
                    auto next = std::make_shared<xgc2_world_lidar::WorldSensorGeometry>(*sensor_geometry_);
                    for (const auto group : binding.groups) next->shared[group] = cloud;
                    ++next->version;
                    sensor_geometry_ = std::move(next);
                    sensor_cloud_loaded_[i] = true;
                    sensor_cloud_subscribers_[i].shutdown(); // original first-cloud policy
                });
        xgc2_world_lidar::WorldSensorCallbacks callbacks;
        callbacks.now = [] { return ros::Time::now(); }; // original ROS domain, not Grid/World time
        callbacks.process_inputs = [this] { sensor_geometry_queue_.callAvailable(ros::WallDuration(0)); };
        callbacks.capture = [this](xgc2_world_lidar::WorldSensorAcquisition& frame) {
            { std::lock_guard<std::mutex> lock(sensor_input_mutex_);
              for (std::size_t i = 0; i < sensor_sources_.size(); ++i) frame.sources[i] = sensor_sources_[i]; }
            frame.geometry = sensor_geometry_; // same caller as geometry callback, const strong pin
            frame.world_commit = sensor_world_commit_.load(std::memory_order_relaxed);
        };
        callbacks.publish = [this](std::size_t i, xgc2_world_lidar::SensorOutputKind,
                                   const sensor_msgs::PointCloud2& cloud,
                                   const xgc2_world_lidar::SensorAcquisition&) {
            if (!sensor_teardown_.load(std::memory_order_acquire)) sensor_outputs_[i].publish(cloud);
        };
        callbacks.fatal = [this](std::exception_ptr error) { ReportSensorFatal(error); };
        sensor_system_ = std::make_unique<xgc2_world_lidar::WorldSensorSystem>(
            inputs.configuration, std::move(callbacks));
        // Requires the matching cold-start postcondition library: static
        // pool/layout/clock anchor only. No map, source, timer or first-step wait.
        sensor_system_->start();
    }

    void AcceptSensorPose(std::size_t i, const geometry_msgs::Pose& pose, const ros::Time& stamp) {
        if (sensor_teardown_.load(std::memory_order_acquire)) return;
        { std::lock_guard<std::mutex> lock(sensor_input_mutex_);
          auto& source = sensor_sources_[i];
          source.position = {pose.position.x, pose.position.y, pose.position.z};
          source.orientation = {pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
          source.source_stamp = stamp; source.ready = true; ++source.source_version;
          source.world_commit = sensor_world_commit_.load(std::memory_order_relaxed); }
        // Original accepted source, no new finite/normalization/Pairer/truth substitution.
        // The original ROS clock/due loop and actual World update supply wakeups;
        // do not borrow a partially constructed sensor owner from a ROS callback.
    }

    void ReportSensorFatal(std::exception_ptr error) {
        // WorldSensorSystem has already fenced and destroyed scan/GPU/pool
        // resources. Do not acquire the host mutex or stop/join this caller.
        std::string failure = "unknown World sensor backend failure";
        try { std::rethrow_exception(error); }
        catch (const std::exception& value) { failure = value.what(); }
        catch (...) {}
        ROS_FATAL("XGC Gazebo World sensor failed: %s", failure.c_str());
        // Original Server control subscriber appears AFTER WorldCreated. Keep
        // native publisher/Node alive; never send early and assume latching.
        while (!sensor_teardown_.load(std::memory_order_acquire)) {
            if (sensor_control_publisher_->WaitForConnection(gazebo::common::Time(0, 10000000))) {
                if (sensor_teardown_.load(std::memory_order_acquire)) return;
                gazebo::msgs::ServerControl stop;
                stop.set_stop(true);
                sensor_control_publisher_->Publish(stop, true);
                // Connection and local send are NOT execution/identity ACK.
                // Server main consumes its existing queue and does shutdown.
                return;
            }
        }
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

    bool DiscoverObstacles(const gazebo::physics::Model_V& models, double simulation_time) {
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
                // Retried, and reported, on every update while it is present.
                rediscover_ = true;
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
        if (sensor_teardown_.load(std::memory_order_acquire)) return;
        sensor_world_commit_.fetch_add(1, std::memory_order_relaxed);
        if (sensor_system_) sensor_system_->notify();
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
            geometry_changed = DiscoverObstacles(models, simulation_time);
            obstacle_messages_stale_ = obstacle_messages_stale_ || geometry_changed;
            model_snapshot_.Update(models);
        }

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
        // Discovery marks the messages stale whenever the set changes; the count
        // check keeps Set() inside the messages should that ever be missed.
        if (obstacle_messages_stale_ || obstacle_messages_.size() != obstacles_.size()) {
            std::vector<ObstacleMessages::Obstacle> fixed;
            fixed.reserve(obstacles_.size());
            for (const auto& item : obstacles_) {
                fixed.push_back(
                    {item.first, item.second.model->GetName(), item.second.generation, &item.second.definition});
            }
            obstacle_messages_.Reset(fixed);
            obstacle_messages_stale_ = false;
        }
        obstacle_messages_.Begin(ros::Time(simulation_time.sec, simulation_time.nsec), scene_epoch_, scene_revision_);
        ObstacleDynamics dynamics;
        std::size_t index = 0;
        for (const auto& item : obstacles_) {
            SampleObstacle(item.second, simulation_time.Double(), &dynamics);
            obstacle_messages_.Set(index++, dynamics);
        }
        state_publisher_.publish(obstacle_messages_.state());
        instances_publisher_.publish(obstacle_messages_.instances());
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
    std::unique_ptr<WorldSensorInputs> sensor_inputs_;
    std::unique_ptr<xgc2_world_lidar::WorldSensorSystem> sensor_system_;
    std::atomic<bool> sensor_teardown_{false};
    std::atomic<uint64_t> sensor_world_commit_{0};
    std::mutex sensor_input_mutex_;
    std::vector<xgc2_world_lidar::SensorAcquisition> sensor_sources_;
    std::shared_ptr<const xgc2_world_lidar::WorldSensorGeometry> sensor_geometry_;
    std::vector<ros::Subscriber> sensor_pose_subscribers_, sensor_cloud_subscribers_;
    std::vector<bool> sensor_cloud_loaded_; // sensor caller only
    std::vector<ros::Publisher> sensor_outputs_;
    ros::CallbackQueue sensor_geometry_queue_;
    std::unique_ptr<ros::NodeHandle> sensor_geometry_node_;
    gazebo::transport::PublisherPtr sensor_control_publisher_;

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
    ObstacleMessages obstacle_messages_;
    bool obstacle_messages_stale_ = true;
    ModelSnapshot<gazebo::physics::ModelPtr, boost::weak_ptr<gazebo::physics::Model>> model_snapshot_;
    bool rediscover_ = true;
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
