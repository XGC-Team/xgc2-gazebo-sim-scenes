#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/BoxShape.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/WorldState.hh>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
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
#include "xgc2_gazebo_scene/motion_controller.hpp"

namespace xgc2_gazebo_scene {
namespace {

constexpr char kManagedPrefix[] = "xgc2_obstacle_";
constexpr char kGeometryTopic[] = "/xgc2/simulation/obstacles/geometry";
constexpr char kStateTopic[] = "/xgc2/simulation/obstacles/state";
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

std::string LogicalName(const std::string& model_name) {
    if (model_name.compare(0, sizeof(kManagedPrefix) - 1, kManagedPrefix) != 0) {
        return "";
    }
    return model_name.substr(sizeof(kManagedPrefix) - 1);
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
        geometry_publisher_ = node_->advertise<ObstacleDefinitionArray>(kGeometryTopic, 1, true);
        state_publisher_ = node_->advertise<ObstacleStateArray>(kStateTopic, 1, true);
        configure_service_ =
            node_->advertiseService(kConfigureService, &GazeboSceneSystemPlugin::ConfigureMotionsCallback, this);
        stop_service_ = node_->advertiseService(kStopService, &GazeboSceneSystemPlugin::StopMotionsCallback, this);
        spinner_ = std::make_unique<ros::AsyncSpinner>(1);
        spinner_->start();
        update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&GazeboSceneSystemPlugin::OnUpdate, this, std::placeholders::_1));
        ROS_INFO("XGC Gazebo Scene attached to world %s", world_name.c_str());
    }

    ObstacleDefinition BuildDefinition(const std::string& logical_name, uint64_t generation,
                                       const gazebo::physics::ModelPtr& model) {
        ObstacleDefinition definition;
        definition.name = logical_name;
        definition.model_name = model->GetName();
        definition.generation = generation;

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
                } else {
                    ROS_WARN("Managed obstacle %s collision %s has an unsupported shape; "
                             "it is omitted from planning geometry",
                             model->GetName().c_str(), collision->GetName().c_str());
                    continue;
                }
                definition.parts.push_back(std::move(part));
            }
        }
        return definition;
    }

    bool DiscoverObstacles(double simulation_time) {
        std::map<std::string, gazebo::physics::ModelPtr> current;
        for (const auto& model : world_->Models()) {
            const std::string logical_name = LogicalName(model->GetName());
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
            obstacle.generation = ++generation_counters_[item.first];
            obstacle.observed_pose = item.second->WorldPose();
            std::string error;
            MotionConfiguration hold;
            if (!obstacle.controller.Configure(hold, obstacle.observed_pose, simulation_time, &error)) {
                ROS_ERROR("Cannot initialize obstacle controller: %s", error.c_str());
            }
            obstacle.definition = BuildDefinition(item.first, obstacle.generation, item.second);
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

        if (geometry_changed) {
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
    std::unique_ptr<ros::NodeHandle> node_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
    ros::Publisher geometry_publisher_;
    ros::Publisher state_publisher_;
    ros::ServiceServer configure_service_;
    ros::ServiceServer stop_service_;
    std::map<std::string, ManagedObstacle> obstacles_;
    std::map<std::string, uint64_t> generation_counters_;
    std::map<std::string, CachedConfigure> configured_commands_;
    std::map<std::string, CachedStop> stopped_commands_;
    std::string scene_epoch_;
    uint64_t scene_revision_ = 0;
    double last_publish_time_ = -1.0;
};

GZ_REGISTER_SYSTEM_PLUGIN(GazeboSceneSystemPlugin)

} // namespace xgc2_gazebo_scene
