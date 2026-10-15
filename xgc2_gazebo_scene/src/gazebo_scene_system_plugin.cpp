#include "xgc2_gazebo_scene/ros_master_registration.hpp"
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

#include "xgc2_gazebo_scene/model_snapshot.hpp"
#include "xgc2_gazebo_scene/physical_contact_filter.hpp"

namespace xgc2_gazebo_scene {
namespace {

constexpr char kDefaultManagedPrefix[] = "xgc2_obstacle_";
constexpr char kPhysicalCollisionTopic[] = "/xgc2/simulation/physical_collision";
constexpr char kPhysicalCollisionDetailTopic[] = "/xgc2/simulation/physical_collision_detail";

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

} // namespace

class GazeboSceneSystemPlugin final : public gazebo::SystemPlugin {
  public:
    GazeboSceneSystemPlugin() = default;

    ~GazeboSceneSystemPlugin() override {
        registration_.reset();
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

        registration_ = std::make_unique<RosMasterRegistration>([this, world_name] {
            AttachRos(world_name);
        });
    }

    void AttachRos(const std::string& world_name) {
        std::lock_guard<std::mutex> lock(mutex_);
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
        if (physical_contact_monitor_enabled_) {
            physical_collision_publisher_ = node_->advertise<std_msgs::Bool>(kPhysicalCollisionTopic, 1, true);
            physical_collision_detail_publisher_ =
                node_->advertise<std_msgs::String>(kPhysicalCollisionDetailTopic, 1, true);
            if (!physical_collision_publisher_ || !physical_collision_detail_publisher_)
                throw std::runtime_error("scene ROS contact registration failed");
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

    void OnUpdate(const gazebo::common::UpdateInfo& /*info*/) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!world_)
            return;
        const auto models = world_->Models();
        if (!model_snapshot_.Matches(models)) {
            RefreshContactModels(models);
            model_snapshot_.Update(models);
        }
    }

    std::mutex mutex_;
    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr world_created_connection_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::event::ConnectionPtr world_reset_connection_;
    gazebo::transport::NodePtr gazebo_transport_node_;
    gazebo::transport::SubscriberPtr contact_subscriber_;
    std::unique_ptr<ros::NodeHandle> node_;
    std::unique_ptr<RosMasterRegistration> registration_;
    ros::Publisher physical_collision_publisher_;
    ros::Publisher physical_collision_detail_publisher_;
    std::map<std::string, ContactModelDescriptor> contact_models_;
    ModelSnapshot<gazebo::physics::ModelPtr, boost::weak_ptr<gazebo::physics::Model>> model_snapshot_;
    std::string managed_obstacle_prefix_ = kDefaultManagedPrefix;
    std::vector<std::string> tracked_model_prefixes_;
    bool physical_contact_monitor_enabled_ = true;
    bool physical_collision_detected_ = false;
};

GZ_REGISTER_SYSTEM_PLUGIN(GazeboSceneSystemPlugin)

} // namespace xgc2_gazebo_scene
