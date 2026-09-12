#include "xgc2_gazebo_scene/scene_model.hpp"
#include "xgc2_gazebo_scene/scene_ownership.hpp"

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <xgc2_geometry_msgs/ApplyScene.h>
#include <xgc2_geometry_msgs/SceneConsumerStatus.h>
#include <xgc2_geometry_msgs/SceneState.h>

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace xgc2_gazebo_scene {
namespace {

std::string SnapshotIdentity(xgc2_geometry_msgs::SceneSnapshot scene) {
    scene.header.stamp = ros::Time(0);
    scene.header.seq = 0;
    std::string bytes(ros::serialization::serializationLength(scene), '\0');
    ros::serialization::OStream stream(reinterpret_cast<std::uint8_t*>(&bytes[0]), bytes.size());
    ros::serialization::serialize(stream, scene);
    return bytes;
}

bool FiniteTwist(const geometry_msgs::Twist& twist) {
    return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) && std::isfinite(twist.linear.z) &&
           std::isfinite(twist.angular.x) && std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

} // namespace

// The scene runtime owns authoring and motion. This adapter only materializes
// validated snapshots and follows computed state; it never loads algorithm YAML.
class SceneAuthoringWorldPlugin final : public gazebo::WorldPlugin {
  public:
    ~SceneAuthoringWorldPlugin() override {
        stopping_ = true;
        if (node_)
            node_->shutdown();
        if (spinner_)
            spinner_->stop();
        if (!mesh_directory_.empty()) {
            std::error_code error;
            std::filesystem::remove_all(mesh_directory_, error);
        }
    }

    void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override {
        world_ = std::move(world);
        const std::string ns =
            sdf->HasElement("scene_namespace") ? sdf->Get<std::string>("scene_namespace") : "/xgc/scene";
        if (sdf->HasElement("apply_timeout"))
            timeout_ = sdf->Get<double>("apply_timeout");
        if (!std::isfinite(timeout_) || timeout_ <= 0.0 || timeout_ > 60.0) {
            gzerr << "Scene adapter apply_timeout must be in (0, 60] seconds\n";
            return;
        }
        char directory[] = "/tmp/xgc2-scene-meshes-XXXXXX";
        char* created = mkdtemp(directory);
        if (!created) {
            gzerr << "Scene adapter cannot create private mesh directory\n";
            return;
        }
        mesh_directory_ = created;
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "xgc2_scene_gazebo", ros::init_options::NoSigintHandler);
        }
        node_ = std::make_unique<ros::NodeHandle>(ns);
        node_->setCallbackQueue(&queue_);
        transport_.reset(new gazebo::transport::Node());
        transport_->Init(world_->Name());
        visual_ = transport_->Advertise<gazebo::msgs::Visual>("~/visual");
        status_ = node_->advertise<xgc2_geometry_msgs::SceneConsumerStatus>("consumer_status", 10, true);
        generation_ = 1;
        apply_ = node_->advertiseService("gazebo/apply", &SceneAuthoringWorldPlugin::Apply, this);
        state_ = node_->subscribe("state", 1, &SceneAuthoringWorldPlugin::State, this);
        heartbeat_ = node_->createWallTimer(ros::WallDuration(1.0), &SceneAuthoringWorldPlugin::Heartbeat, this);
        // All adapter state and world mutations are serialized. Gazebo's factory
        // inserts at its own safe boundary; RemoveModel waits for the physics
        // mutex. These native APIs also work while simulation time is paused.
        spinner_ = std::make_unique<ros::AsyncSpinner>(1, &queue_);
        spinner_->start();
        ROS_INFO_STREAM("Editable scene adapter available at " << node_->resolveName("gazebo/apply"));
    }

  private:
    void Publish(const std::string& epoch, std::uint64_t revision, bool applied, const std::string& message) {
        xgc2_geometry_msgs::SceneConsumerStatus status;
        status.header.stamp = ros::Time::now();
        status.header.frame_id = "world";
        status.epoch = epoch;
        status.revision = revision;
        status.consumer = "gazebo";
        status.generation = generation_;
        status.applied = applied;
        status.operational = applied;
        status.capability = applied ? "ok" : "";
        status.success = applied;
        status.message = message;
        status_.publish(status);
    }

    bool Wait(const ros::WallTime& deadline, const std::function<bool()>& ready) {
        while (!stopping_ && ros::ok()) {
            if (ready())
                return true;
            if (ros::WallTime::now() >= deadline)
                return false;
            ros::WallDuration(0.01).sleep();
        }
        return false;
    }

    bool Apply(xgc2_geometry_msgs::ApplyScene::Request& request, xgc2_geometry_msgs::ApplyScene::Response& response) {
        const auto& scene = request.scene;
        response.epoch = epoch_;
        response.applied_revision = revision_;
        const auto reject = [&](const std::string& error) {
            response.success = false;
            response.message = error;
            Publish(scene.epoch, scene.revision, false, error);
            return true;
        };
        std::map<std::string, SceneModel> desired;
        std::string error;
        if (!CompileScene(scene, mesh_directory_, &desired, &error))
            return reject(error);
        const std::string identity = SnapshotIdentity(scene);
        if (retired_epochs_.count(scene.epoch))
            return reject("retired scene epoch");
        if (scene.epoch == epoch_) {
            if (scene.revision < revision_)
                return reject("stale scene revision");
            if (scene.revision == revision_ && identity != snapshot_identity_)
                return reject("scene revision reused for different content");
        }
        const auto deadline = ros::WallTime::now() + ros::WallDuration(timeout_);
        // A timed-out factory request can still create its model later. Drain
        // those requests before applying a corrective snapshot; otherwise a
        // successful Clear could be followed by a late, unacknowledged obstacle.
        if (!Wait(deadline, [&] {
                for (auto it = pending_factory_.begin(); it != pending_factory_.end();) {
                    if (world_->ModelByName(*it))
                        it = pending_factory_.erase(it);
                    else
                        ++it;
                }
                return pending_factory_.empty();
            })) {
            last_error_ = "previous Gazebo factory requests are still incomplete; retry synchronization";
            return reject(last_error_);
        }
        if (identity == snapshot_identity_ && geometry_consistent_) {
            bool intact = true;
            for (const auto& entry : desired) {
                intact =
                    intact && VerifySceneModel(world_->ModelByName(entry.second.name), entry.second, false, &error);
            }
            if (intact) {
                // An RPC retry must not reset a moving obstacle to its initial pose.
                response.success = true;
                response.message = "scene revision is already applied";
                last_error_.clear();
                Publish(epoch_, revision_, true, response.message);
                return true;
            }
        }
        // Never adopt or overwrite a model created by another world component,
        // even if it happens to use our naming prefix.
        for (const auto& entry : desired) {
            if (world_->ModelByName(entry.second.name) && !owned_.count(entry.second.name)) {
                return reject("model name is owned outside this scene adapter: " + entry.second.name);
            }
        }

        std::set<std::string> desired_names;
        std::set<std::string> remove;
        std::map<std::string, ignition::math::Pose3d> current_poses;
        for (const auto& entry : desired)
            desired_names.insert(entry.second.name);
        for (const auto& name : owned_)
            if (!desired_names.count(name))
                remove.insert(name);
        for (const auto& entry : desired) {
            const auto model = world_->ModelByName(entry.second.name);
            const auto previous = models_.find(entry.first);
            current_poses[entry.first] = entry.second.pose;
            if (geometry_consistent_ && scene.epoch == epoch_ && model && previous != models_.end() &&
                previous->second.pose == entry.second.pose) {
                // Editing another object, or resizing this moving object, must
                // not teleport an unchanged initial definition back to time zero.
                current_poses[entry.first] = model->WorldPose();
            }
            if (model && (previous == models_.end() || previous->second.body != entry.second.body ||
                          !VerifySceneModel(model, entry.second, false, &error))) {
                remove.insert(entry.second.name);
            }
        }

        // From this point a failure may have partially changed the world. Keep
        // the old applied revision, publish failure, and require a full repair
        // snapshot before following motion again. Never claim atomic rollback.
        geometry_consistent_ = false;
        const auto failed = [&](const std::string& reason) {
            last_error_ = reason;
            return reject(reason);
        };
        try {
            std::set<std::string> retiring;
            for (const auto& name : remove) {
                if (const auto model = world_->ModelByName(name))
                    retiring.insert(RemoveOwnedModel(model));
            }
            // Renaming frees the original name immediately. Waiting only for
            // that name would acknowledge a delete while the retired body and
            // its collision/visual are still in the world.
            if (!Wait(deadline, [&] { return SceneBodiesGone(remove) && SceneBodiesGone(retiring) && !HasRetiredSceneModels(); }))
                return failed("timed out waiting for removed scene collisions");

            for (const auto& entry : desired) {
                const auto& definition = entry.second;
                const auto model = world_->ModelByName(definition.name);
                if (model) {
                    model->SetWorldPose(current_poses.at(entry.first));
                } else {
                    owned_.insert(definition.name);
                    pending_factory_.insert(definition.name);
                    world_->InsertModelString(definition.sdf);
                }
            }
            if (!desired.empty() && ros::WallTime::now() >= deadline)
                return failed("scene application timed out before collision verification");
            std::set<std::string> parameterized;
            if (!Wait(deadline, [&] {
                    for (const auto& entry : desired) {
                        const auto model = world_->ModelByName(entry.second.name);
                        if (!parameterized.count(entry.first)) {
                            if (!ApplySceneModelParameters(model, entry.second, current_poses.at(entry.first))) {
                                VerifySceneModel(model, entry.second, false, &error);
                                if (error.empty())
                                    error = entry.first + ": factory model is not initialized";
                                return false;
                            }
                            parameterized.insert(entry.first);
                            pending_factory_.erase(entry.second.name);
                        }
                        auto expected = entry.second;
                        expected.pose = current_poses.at(entry.first);
                        if (!VerifySceneModel(model, expected, true, &error))
                            return false;
                    }
                    for (const auto& name : owned_)
                        if (!desired_names.count(name) && world_->ModelByName(name))
                            return false;
                    return SceneBodiesGone(retiring) && !HasRetiredSceneModels();
                }))
                return failed("scene application did not complete: " + error);
        } catch (const std::exception& exception) {
            return failed(std::string("Gazebo scene application failed: ") + exception.what());
        }

        if (!epoch_.empty() && epoch_ != scene.epoch)
            retired_epochs_.insert(epoch_);
        epoch_ = scene.epoch;
        revision_ = scene.revision;
        snapshot_identity_ = identity;
        models_ = std::move(desired);
        owned_ = std::move(desired_names);
        geometry_consistent_ = true;
        last_error_.clear();
        response.success = true;
        response.epoch = epoch_;
        response.applied_revision = revision_;
        response.message = "scene collision and visual definitions applied";
        Publish(epoch_, revision_, true, response.message);
        return true;
    }

    void State(const xgc2_geometry_msgs::SceneState::ConstPtr& state) {
        if (!geometry_consistent_ || state->epoch != epoch_ || state->revision != revision_)
            return;
        std::set<std::string> ids;
        bool valid = state->header.frame_id == "world" && std::isfinite(state->scene_time) &&
                     state->scene_time >= 0.0 && state->obstacles.size() == models_.size();
        for (const auto& obstacle : state->obstacles) {
            valid = valid && models_.count(obstacle.id) && ids.insert(obstacle.id).second &&
                    ValidateScenePose(obstacle.pose) && FiniteTwist(obstacle.twist);
        }
        if (!valid) {
            last_error_ = "rejected malformed scene state";
            Publish(epoch_, revision_, false, last_error_);
            return;
        }
        // Verify every target before moving any target. The scene runtime sends
        // current world poses for all objects, including held static obstacles.
        for (const auto& obstacle : state->obstacles) {
            if (!world_->ModelByName(models_.at(obstacle.id).name)) {
                geometry_consistent_ = false;
                last_error_ = "scene model disappeared: " + obstacle.id;
                Publish(epoch_, revision_, false, last_error_);
                return;
            }
        }
        for (const auto& obstacle : state->obstacles) {
            auto model = world_->ModelByName(models_.at(obstacle.id).name);
            model->SetWorldPose(ScenePose(obstacle.pose));
        }
        last_error_.clear();
    }

    bool SceneBodiesGone(const std::set<std::string>& names) const {
        for (const auto& name : names) {
            if (world_->ModelByName(name))
                return false;
        }
        return true;
    }

    bool HasRetiredSceneModels() const {
        for (const auto& model : world_->Models()) {
            if (model && IsRetiredSceneModel(model->GetName()))
                return true;
        }
        return false;
    }

    void PublishVisualDelete(const gazebo::physics::ModelPtr& model) {
        gazebo::msgs::Visual removal;
        removal.set_name(model->GetScopedName());
        removal.set_id(model->GetId());
        removal.set_parent_name(world_->Name());
        removal.set_delete_me(true);
        visual_->Publish(removal, true);
    }

    std::string RemoveOwnedModel(const gazebo::physics::ModelPtr& model) {
        // Entity::Fini publishes an asynchronous entity_delete request. If its
        // old name is immediately reused, that request can delete the replacement
        // model on the next server tick. Retire the old entity under a unique
        // internal name first, including cached descendant scopes.
        std::string retired;
        {
            boost::recursive_mutex::scoped_lock lock(*world_->Physics()->GetPhysicsUpdateMutex());
            PublishVisualDelete(model);
            retired = world_->UniqueModelName(std::string(kSceneRetiredModelPrefix) + std::to_string(model->GetId()));
            model->SetName(retired);
            std::function<void(const gazebo::physics::BasePtr&)> refresh = [&](const gazebo::physics::BasePtr& parent) {
                for (unsigned i = 0; i < parent->GetChildCount(); ++i) {
                    auto child = parent->GetChild(i);
                    const auto child_sdf = child->GetSDF();
                    if (child_sdf && child_sdf->HasAttribute("name"))
                        child->SetName(child->GetName());
                    refresh(child);
                }
            };
            refresh(model);
            PublishVisualDelete(model);
        }
        // RemoveModel waits for the physics lock. Holding it here would stall
        // the update thread that actually retires the collision.
        world_->RemoveModel(retired);
        return retired;
    }

    void Heartbeat(const ros::WallTimerEvent&) {
        if (epoch_.empty()) {
            Publish("", 0, false, "waiting for an initial scene snapshot");
            return;
        }
        if (geometry_consistent_) {
            for (const auto& entry : models_) {
                std::string error;
                if (!VerifySceneModel(world_->ModelByName(entry.second.name), entry.second, false, &error)) {
                    geometry_consistent_ = false;
                    last_error_ = error;
                    break;
                }
            }
        }
        Publish(epoch_, revision_, geometry_consistent_ && last_error_.empty(), last_error_);
    }

    gazebo::physics::WorldPtr world_;
    gazebo::transport::NodePtr transport_;
    gazebo::transport::PublisherPtr visual_;
    ros::CallbackQueue queue_;
    std::unique_ptr<ros::NodeHandle> node_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
    ros::ServiceServer apply_;
    ros::Subscriber state_;
    ros::Publisher status_;
    ros::WallTimer heartbeat_;
    std::atomic<bool> stopping_{false};
    std::string mesh_directory_;
    double timeout_ = 5.0;
    std::string epoch_;
    std::uint64_t revision_ = 0;
    std::string snapshot_identity_;
    std::set<std::string> retired_epochs_;
    std::set<std::string> owned_;
    std::set<std::string> pending_factory_;
    std::map<std::string, SceneModel> models_;
    bool geometry_consistent_ = false;
    std::string last_error_;
    std::uint32_t generation_ = 0;
};

GZ_REGISTER_WORLD_PLUGIN(SceneAuthoringWorldPlugin)

} // namespace xgc2_gazebo_scene
