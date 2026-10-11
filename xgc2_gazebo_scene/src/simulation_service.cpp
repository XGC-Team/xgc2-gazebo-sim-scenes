#include "xgc2_gazebo_scene/simulation_service.hpp"
#include "chassis_hold_host.hpp"
#include "xgc2_gazebo_scene/native_entity_motion.hpp"
#include "xgc2_gazebo_scene/native_scene_controller.hpp"
#include "xgc2_gazebo_scene/native_sensor_controller.hpp"
#include "xgc2_gazebo_scene/world_startup_binding.hpp"
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <map>
#include <mutex>
#include <sdf/sdf.hh>
#include <sys/stat.h>
#include <thread>
#include <xgc2/xrpc/http.hpp>

namespace xgc2_gazebo_scene {
namespace detail {
class WorldStartupState {
  public:
    gazebo::physics::WorldPtr world;
    std::function<void(gazebo::physics::WorldPtr)> attach;
    std::mutex mutex;
    bool closed = false, started = false;
    void Invoke() {
        std::lock_guard<std::mutex> lock(mutex);
        if (!closed && !started) {
            started = true;
            attach(world);
        }
    }
};
} // namespace detail
namespace {
using Clock = std::chrono::steady_clock;
using xgc2::xrpc::HttpReply;
using xgc2::xrpc::HttpRequest;
using xgc2::xrpc::HttpResponse;
constexpr std::size_t kQueue = 32, kEntities = 256, kReceipts = 128, kWaiters = 32;
constexpr std::size_t kArtifactBytes = 65536;
// Bounds of the world's http.v1 server (the SDK reads no environment).
xgc2::xrpc::HttpLimits WorldLimits() {
    using std::chrono::milliseconds;
    xgc2::xrpc::HttpLimits limits;
    limits.connections = 32;
    limits.header_bytes = 8192;
    limits.request_bytes = 131072;
    limits.response_bytes = 262144;
    limits.inflight = 32;
    limits.request_timeout = milliseconds(60000);
    limits.idle_timeout = milliseconds(5000);
    limits.header_timeout = milliseconds(5000);
    limits.shutdown_timeout = milliseconds(1000);
    return limits;
}
std::mutex registry_mutex;
std::map<gazebo::physics::World*, SimulationService*> registry;
std::vector<std::weak_ptr<detail::WorldStartupState>> startup_bindings;
struct DomainError : std::runtime_error {
    int status;
    std::string code;
    DomainError(int status_, std::string code_, const std::string& detail)
        : std::runtime_error(detail), status(status_), code(std::move(code_)) {}
};
[[noreturn]] void Invalid(const std::string& detail) {
    throw DomainError(400, "invalid_argument", detail);
}
void Fields(const Json::Value& v, std::initializer_list<const char*> allowed) {
    if (!v.isObject())
        Invalid("expected object");
    for (const auto& key : v.getMemberNames()) {
        bool known = false;
        for (const auto* name : allowed)
            known |= key == name;
        if (!known)
            Invalid("unsupported field: " + key);
    }
}
std::string String(const Json::Value& v) {
    if (!v.isString() || v.asString().empty())
        Invalid("expected nonempty string");
    return v.asString();
}
std::string Id(const Json::Value& v) {
    auto id = String(v);
    if (id.size() > 256)
        Invalid("entity ID exceeds 256 bytes");
    for (unsigned char c : id)
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-' ||
              c == '.' || c == ':'))
            Invalid("entity ID must be URL-safe ASCII");
    return id;
}
std::uint64_t UInt(const Json::Value& v, std::uint64_t maximum) {
    if ((v.type() != Json::intValue && v.type() != Json::uintValue) || !v.isUInt64() || v.asUInt64() > maximum)
        Invalid("expected bounded unsigned integer");
    return v.asUInt64();
}
double Number(const Json::Value& v) {
    if (!v.isNumeric() || v.isBool() || !std::isfinite(v.asDouble()))
        Invalid("expected finite number");
    return v.asDouble();
}
ignition::math::Vector3d Vector(const Json::Value& v) {
    if (!v.isArray() || v.size() != 3)
        Invalid("vector must have three components");
    return {Number(v[0]), Number(v[1]), Number(v[2])};
}
ignition::math::Pose3d Pose(const Json::Value& v) {
    Fields(v, {"position", "orientation"});
    auto p = Vector(v["position"]);
    const auto& q = v["orientation"];
    if (!q.isArray() || q.size() != 4)
        Invalid("orientation must be xyzw");
    ignition::math::Quaterniond r(Number(q[3]), Number(q[0]), Number(q[1]), Number(q[2]));
    double norm = r.W() * r.W() + r.X() * r.X() + r.Y() * r.Y() + r.Z() * r.Z();
    if (std::abs(norm - 1) > 1e-6)
        Invalid("orientation must be a unit quaternion");
    return {p, r};
}
Json::Value Encode(const ignition::math::Vector3d& v) {
    Json::Value out(Json::arrayValue);
    out.append(v.X());
    out.append(v.Y());
    out.append(v.Z());
    return out;
}
Json::Value Encode(const ignition::math::Pose3d& p) {
    Json::Value out;
    out["position"] = Encode(p.Pos());
    auto& q = out["orientation"] = Json::Value(Json::arrayValue);
    q.append(p.Rot().X());
    q.append(p.Rot().Y());
    q.append(p.Rot().Z());
    q.append(p.Rot().W());
    return out;
}
Json::Value Parse(const std::string& body) {
    Json::CharReaderBuilder b;
    b["rejectDupKeys"] = true;
    b["failIfExtra"] = true;
    b["allowComments"] = false;
    b["allowSpecialFloats"] = false;
    b["stackLimit"] = 32;
    b["collectComments"] = false;
    Json::Value v;
    std::string error;
    auto reader = std::unique_ptr<Json::CharReader>(b.newCharReader());
    if (!reader->parse(body.data(), body.data() + body.size(), &v, &error))
        Invalid("invalid JSON document");
    return v;
}
HttpResponse Response(const Json::Value& v, int status = 200) {
    Json::StreamWriterBuilder b;
    b["indentation"] = "";
    return {status, {{"Content-Type", "application/json"}}, Json::writeString(b, v), true};
}
struct NativeState {
    std::string id, role;
    std::uint64_t generation = 0;
    ignition::math::Pose3d pose;
    ignition::math::Vector3d linear, angular;
    bool enabled = true;
    std::string specification;
    bool native_ready = true, scene_owned = false;
};
Json::Value Encode(const NativeState& s) {
    Json::Value v;
    v["ref"]["id"] = s.id;
    v["ref"]["generation"] = Json::UInt64(s.generation);
    v["role"] = s.role;
    v["lifecycle"] = s.native_ready ? "ready" : "failed";
    v["owner"] = s.scene_owned ? "scene" : "entity";
    v["state"]["pose"] = Encode(s.pose);
    v["state"]["twist"]["linear"] = Encode(s.linear);
    v["state"]["twist"]["angular"] = Encode(s.angular);
    v["state"]["enabled"] = s.enabled;
    if (!s.specification.empty())
        v["specification"] = Parse(s.specification);
    return v;
}
enum class Kind {
    QueryWorld,
    QueryEntities,
    QueryEntity,
    Create,
    Remove,
    SetState,
    ResetEntity,
    Pause,
    Resume,
    Step,
    ResetWorld,
    QueryScene,
    Scene,
    Motion,
    Sensor,
    Extension
};
struct Command {
    Kind kind = Kind::QueryWorld;
    std::string id, role, native_name;
    std::string specification;
    std::uint64_t generation = 0;
    ignition::math::Pose3d pose;
    ignition::math::Vector3d linear, angular;
    bool has_pose = false, has_twist = false, has_enabled = false, enabled = true;
    bool reset_time = false;
    bool chassis = false; // the asset carries a chassis plugin: HOLD binds it under the entity ID
    unsigned steps = 0;
    std::vector<std::pair<std::string, std::uint64_t>> refs;
    sdf::SDFPtr artifact;
    std::shared_ptr<const PreparedSceneCommand> scene;
    std::shared_ptr<const PreparedEntityMotion> motion;
    std::shared_ptr<const PreparedSensorCommand> sensor;
    std::shared_ptr<NativeWorldExtension> extension_owner;
    std::shared_ptr<const PreparedWorldExtension> extension;
    Clock::time_point expires;
};
struct NativeResult {
    std::vector<NativeState> entities;
    std::int64_t nanoseconds = 0;
    std::uint64_t epoch = 0;
    bool paused = false;
    double step_size = 0;
    int status = 200;
    std::string code, message;
    std::string extension_result;
    bool effects_started = false;
};
struct Entity {
    std::string id, role, native_name;
    std::uint64_t generation = 0;
    gazebo::physics::ModelPtr model;
    ignition::math::Pose3d initial;
    bool occupied = false, enabled = true;
    std::string scene_body;
    std::string specification;
    bool native_ready = false;
};
} // namespace

class SimulationService::Impl {
  public:
    Impl(gazebo::physics::WorldPtr world, std::string path, std::string target, const std::string& root,
         const std::vector<std::string>& required_components, std::string configuration_revision)
        : world_(std::move(world)), path_(std::move(path)), target_(std::move(target)),
          resource_root_(std::filesystem::canonical(root)), instance_(xgc2::xrpc::new_instance_id()),
          configuration_revision_(std::move(configuration_revision)) {
        if (path_.empty() || target_.empty() || !std::filesystem::is_directory(resource_root_))
            Invalid("explicit socket, target and resource directory required");
        const auto initial_time = world_->SimTime();
        native_clock_ns_.store(std::int64_t(initial_time.sec) * 1000000000LL + initial_time.nsec);
        if (required_components.size() > 16)
            Invalid("native component bound exceeded");
        for (const auto& id : required_components) {
            Id(id);
            for (const auto& existing : components_)
                if (existing.id == id)
                    Invalid("duplicate native component");
            components_.push_back({id, std::make_shared<detail::NativeComponentState>()});
        }
        limits_ = WorldLimits();
        // One HOLD host per world: the domain shares the instance of the http.v1 server.
        chassis_ = detail::ChassisHoldHost::Create(world_, instance_);
        server_ = std::make_unique<xgc2::xrpc::HttpServer>(
            xgc2::xrpc::UnixOptions{path_, 0600, xgc2::xrpc::ExistingPath::ReclaimUnreachable},
            [this](HttpRequest r, HttpReply reply) {
                Handle(std::move(r), std::move(reply));
            },
            limits_, xgc2::xrpc::HttpIdentity{instance_, {"/v1/describe"}});
        server_->set_wakeup_handler([this] {
            Drain();
        });
        for (auto& slot : slots_)
            slot.result.entities.reserve(kEntities);
        mesh_root_ = std::filesystem::path(path_).parent_path() / ("scene-meshes-" + instance_);
        if (!std::filesystem::create_directory(mesh_root_) || chmod(mesh_root_.c_str(), 0700) != 0)
            throw std::runtime_error("cannot allocate granted transient scene artifact directory");
        scene_ = std::make_unique<NativeSceneController>(
            world_, mesh_root_.string(),
            [this](const auto& desired, auto deadline) {
                return ApplyScene(desired, deadline);
            },
            [this] {
                return native_clock_ns_.load();
            });
        motion_ = std::make_unique<NativeEntityMotion>(
            world_,
            [this](const std::string& id, std::uint64_t generation) {
                auto& entity = Find(id, generation);
                if (entity.role != "obstacle" || !entity.scene_body.empty())
                    throw DomainError(409, "conflict", "motion requires an independently owned obstacle EntityRef");
                return entity.model;
            },
            [this] {
                return native_clock_ns_.load();
            });
        sensors_ =
            std::make_unique<NativeSensorController>(world_, [this](const std::string& id, std::uint64_t generation) {
                return Find(id, generation).model;
            });
        update_gate_ = std::make_shared<UpdateGate>();
        update_gate_->owner.store(this);
        auto gate = update_gate_;
        update_ = gazebo::event::Events::ConnectWorldUpdateBegin([gate](const gazebo::common::UpdateInfo& info) {
            ++gate->readers;
            if (auto* owner = gate->owner.load()) {
                owner->native_clock_ns_.store(std::int64_t(info.simTime.sec) * 1000000000LL + info.simTime.nsec);
                // First among the update callbacks: the control steps of the chassis models that connect
                // later see the zero of this tick.
                owner->chassis_->Tick();
                const auto serial = owner->scene_->serial();
                owner->scene_->Update(info.simTime.Double());
                owner->motion_->Update(info.simTime.Double());
                if (owner->scene_observers_active_.load() && owner->scene_->serial() != serial &&
                    !owner->scene_snapshot_requested_.exchange(true))
                    owner->queue_cv_.notify_one();
            }
            --gate->readers;
        });
        time_reset_ = gazebo::event::Events::ConnectTimeReset([gate] {
            ++gate->readers;
            if (auto* owner = gate->owner.load()) {
                // The native reset callback runs on the writer, after ResetTime's
                // assignment. The physics mutex does not guard Gazebo's sec/nsec pair.
                const auto now = owner->world_->SimTime();
                owner->native_clock_ns_.store(std::int64_t(now.sec) * 1000000000LL + now.nsec);
            }
            --gate->readers;
        });
    }
    ~Impl() { Stop(); }
    void Start() {
        chassis_->Start();
        worker_ = std::thread([this] {
            Work();
        });
        io_ = std::thread([this] {
            while (!stopping_) {
                server_->poll(std::chrono::milliseconds(25));
                Drain(); // bounded waiter expiry and finite receipt retention
            }
        });
    }
    void Stop() {
        if (stopping_.exchange(true))
            return;
        if (update_gate_) {
            update_gate_->owner.store(nullptr);
            update_.reset();
            time_reset_.reset();
            while (update_gate_->readers.load())
                std::this_thread::yield();
        }
        // Pending engine factories remain owned until the world owner shuts down.
        // Join domain work before releasing the SDK's endpoint lease.
        sensors_->Stop();
        {
            std::lock_guard<std::mutex> lock(extension_mutex_);
            for (const auto& extension : extensions_)
                extension.second->Stop();
        }
        queue_cv_.notify_all();
        ack_cv_.notify_all();
        if (worker_.joinable())
            worker_.join();
        server_->request_stop();
        if (io_.joinable())
            io_.join();
        // Completed reply copies also own SDK admission/endpoint leases. Release
        // them only after domain work and both fixed executors are quiescent.
        for (auto& slot : slots_) {
            slot.query_reply = {};
            slot.waiters.clear();
        }
        health_observers_.clear();
        scene_observers_.clear();
        // Pending engages hold replies too: answer them before the server goes.
        chassis_->Stop();
        server_.reset();
        scene_.reset();
        motion_.reset();
        sensors_.reset();
        if (!mesh_root_.empty()) {
            std::error_code error;
            std::filesystem::remove_all(mesh_root_, error);
        }
    }
    void Acknowledge(gazebo::physics::ModelPtr model) {
        std::lock_guard<std::mutex> lock(ack_mutex_);
        if (pending_name_ == model->GetName()) {
            pending_model_ = std::move(model);
            ack_cv_.notify_one();
        }
    }
    void MarkNativeReady() noexcept { native_ready_.store(true); }
    std::shared_ptr<detail::NativeComponentState> AttachComponent(const std::string& id) {
        for (const auto& component : components_)
            if (component.id == id) {
                if (component.status->attached.exchange(true))
                    throw std::runtime_error("native component already has an owner");
                component.status->state.store(1);
                component.status->revision.fetch_add(1);
                return component.status;
            }
        throw std::invalid_argument("native component is not granted by this world");
    }
    void RegisterExtension(const std::string& id, std::shared_ptr<NativeWorldExtension> adapter) {
        if (!adapter || id == "scene" || id == "entities")
            throw std::invalid_argument("invalid extension owner");
        bool granted = false;
        for (const auto& component : components_)
            granted |= component.id == id;
        if (!granted)
            throw std::invalid_argument("extension component is not granted by this world");
        std::lock_guard<std::mutex> lock(extension_mutex_);
        if (stopping_ || extensions_.size() >= 16 || extensions_.count(id))
            throw std::runtime_error("extension registration unavailable");
        extensions_.emplace(id, std::move(adapter));
    }
    void UnregisterExtension(const std::string& id, const std::shared_ptr<NativeWorldExtension>& adapter) {
        std::lock_guard<std::mutex> lock(extension_mutex_);
        const auto found = extensions_.find(id);
        if (found != extensions_.end() && found->second == adapter)
            extensions_.erase(found);
    }

  private:
    std::shared_ptr<NativeWorldExtension> SelectExtension(const std::string& target) {
        std::lock_guard<std::mutex> lock(extension_mutex_);
        for (const auto& [id, owner] : extensions_) {
            const auto prefix = "/v1/extensions/" + id;
            if (target == prefix || target.starts_with(prefix + "/"))
                return owner;
        }
        return {};
    }
    struct Component {
        std::string id;
        std::shared_ptr<detail::NativeComponentState> status;
    };
    struct Observer {
        HttpReply reply;
        Clock::time_point deadline;
        std::uint64_t after;
    };
    struct Waiter {
        HttpReply reply;
        Clock::time_point deadline;
    };
    struct UpdateGate {
        std::atomic<Impl*> owner{nullptr};
        std::atomic<unsigned> readers{0};
    };
    struct Slot {
        Command command;
        NativeResult result;
        std::string operation_id, fingerprint, state, kind;
        HttpReply query_reply;
        std::vector<Waiter> waiters;
        std::atomic<int> phase{0}; // free, queued, executing, native completion, retained
        std::atomic<bool> cancel{false};
        bool mutation = false;
        Clock::time_point terminal_at;
    };
    Json::Value Health() const {
        Json::Value v;
        bool ready = native_ready_.load(), failed = false;
        std::uint64_t revision = 1 + std::uint64_t(ready) + std::uint64_t(stopping_.load());
        v["components"] = Json::Value(Json::arrayValue);
        for (const auto& component : components_) {
            const auto state = component.status->state.load();
            revision += component.status->revision.load();
            ready &= state == 2;
            failed |= state == 3;
            Json::Value entry;
            entry["id"] = component.id;
            entry["state"] = state == 2 ? "ready" : state == 3 ? "failed" : state == 1 ? "starting" : "missing";
            v["components"].append(entry);
        }
        v["lifecycle"] = stopping_ ? "stopping" : failed ? "failed" : ready ? "ready" : "starting";
        v["revision"] = Json::UInt64(revision);
        v["instance_id"] = instance_;
        v["world_initialized"] = native_ready_.load();
        v["source"] = "native world and component owners";
        v["pending_commands"] = Json::UInt(pending_.load());
        return v;
    }
    Json::Value Description() {
        Json::Value v;
        auto& ref = v["service_ref"];
        ref["target_id"] = target_;
        ref["service"] = "xgc2.simulation";
        ref["api_version"] = "v1";
        ref["instance_id"] = instance_;
        ref["profile"] = "http.v1";
        ref["endpoint"]["kind"] = "unix";
        ref["endpoint"]["address"] = path_;
        v["describe_path"] = "/v1/describe";
        v["engine"] = "gazebo-classic-11";
        v["configuration"]["revision"] =
            configuration_revision_.empty() ? Json::Value(Json::nullValue) : Json::Value(configuration_revision_);
        v["configuration"]["resource_root"] = resource_root_.string();
        v["health"]["query"] = "/v1/health";
        v["health"]["observe"] = "/v1/health/observe";
        for (const auto& component : components_)
            v["health"]["required_components"].append(component.id);
        v["extensions"]["scene"]["query"] = "/v1/extensions/scene";
        v["extensions"]["scene"]["observe"] = "/v1/extensions/scene/observe";
        {
            std::lock_guard<std::mutex> lock(extension_mutex_);
            for (const auto& [id, owner] : extensions_)
                v["extensions"][id] = owner->Describe();
        }
        for (auto name : {"entities.create", "entities.remove", "entities.set_state", "entities.reset", "world.pause",
                          "world.resume", "world.step", "world.reset.entities", "world.reset.all_entities",
                          "world.reset_time", "health.observe"})
            v["capabilities"].append(name);
        v["capabilities"].append("xgc2.scene.v1");
        v["capabilities"].append("xgc2.entity_motion.v1");
        // These capabilities are retained only with the native Init/Fini fixture.
        for (const auto* name : {"sensors.describe", "sensors.create", "sensors.configure", "sensors.remove"})
            v["capabilities"].append(name);
        for (const auto* kind : {"camera", "imu", "lidar"})
            v["sensor_kinds"].append(kind);
        v["limits"]["sensors"] = 32;
        v["world_frame"]["id"] = "world";
        v["world_frame"]["axes"] = "right-handed z-up";
        v["artifact_types"].append("application/sdf+xml");
        v["artifact_types"].append("application/urdf+xml");
        v["limits"]["entities"] = Json::UInt(kEntities);
        v["limits"]["pending_commands"] = Json::UInt(kQueue);
        v["limits"]["operation_receipts"] = Json::UInt(kReceipts);
        v["limits"]["operation_waiters"] = Json::UInt(kWaiters);
        v["limits"]["operation_timeout_ms"] = 60000;
        v["limits"]["receipt_retention_ms"] = 300000;
        v["limits"]["artifact_bytes"] = Json::UInt(kArtifactBytes);
        v["limits"]["request_bytes"] = Json::UInt64(limits_.request_bytes);
        v["limits"]["response_bytes"] = Json::UInt64(limits_.response_bytes);
        chassis_->DescribeFacts(v["facts"]);
        v["storage"]["durable_writes"] = Json::Value(Json::arrayValue);
        v["storage"]["operation_receipts"] = "instance-scoped memory";
        v["storage"]["scene_meshes"]["location"] = mesh_root_.string();
        v["storage"]["scene_meshes"]["lifetime"] = "world-scoped transient artifacts";
        return v;
    }
    Command Prepare(const HttpRequest& r) {
        Command c;
        auto b = r.body.empty() ? Json::Value(Json::objectValue) : Parse(r.body);
        c.extension_owner = SelectExtension(r.target);
        if (c.extension_owner) {
            if (r.method == "GET" && !r.body.empty())
                Invalid("GET body is unsupported");
            c.kind = Kind::Extension;
            if (r.method == "GET")
                c.expires = r.deadline;
            else {
                const auto budget = UInt(b["operation_timeout_ms"], 60000);
                if (!budget)
                    Invalid("operation_timeout_ms must be positive");
                c.expires = Clock::now() + std::chrono::milliseconds(budget);
            }
            c.extension = c.extension_owner->Prepare(r.method, r.target, b);
            return c;
        }
        if (r.method == "GET") {
            if (!r.body.empty())
                Invalid("GET body is unsupported");
            if (r.target == "/v1/sensors" || r.target.starts_with("/v1/sensors/")) {
                c.kind = Kind::Sensor;
                c.sensor = sensors_->Prepare(r.method, r.target, b);
                c.expires = r.deadline;
                return c;
            }
            if (r.target == "/v1/world")
                c.kind = Kind::QueryWorld;
            else if (r.target == "/v1/entities")
                c.kind = Kind::QueryEntities;
            else if (r.target == "/v1/extensions/scene")
                c.kind = Kind::QueryScene;
            else if (r.target.starts_with("/v1/entities/")) {
                c.kind = Kind::QueryEntity;
                c.id = Id(r.target.substr(13));
            } else
                throw DomainError(404, "not_found", "route not found");
            c.expires = r.deadline;
            return c;
        }
        auto budget = UInt(b["operation_timeout_ms"], 60000);
        if (!budget)
            Invalid("operation_timeout_ms must be positive");
        c.expires = Clock::now() + std::chrono::milliseconds(budget);
        if (r.target == "/v1/sensors" || r.target.starts_with("/v1/sensors/")) {
            c.kind = Kind::Sensor;
            c.sensor = sensors_->Prepare(r.method, r.target, b);
            return c;
        }
        if (r.method == "POST" &&
            (r.target == "/v1/extensions/scene/apply" || r.target == "/v1/extensions/scene/motion")) {
            c.kind = Kind::Scene;
            c.scene = scene_->Prepare(r.target, r.body);
            return c;
        }
        if (r.method == "POST" && r.target == "/v1/extensions/entities/motion") {
            c.kind = Kind::Motion;
            c.motion = motion_->Prepare(r.body);
            return c;
        }
        if (r.target == "/v1/entities" && r.method == "POST") {
            Fields(b, {"entity", "operation_timeout_ms"});
            const auto& e = b["entity"];
            Fields(e, {"id", "role", "asset", "pose", "parameters", "sensors"});
            c.kind = Kind::Create;
            c.id = e.isMember("id") ? Id(e["id"]) : "entity-" + xgc2::xrpc::new_instance_id();
            c.role = String(e["role"]);
            if (c.role != "robot" && c.role != "obstacle" && c.role != "object" && c.role != "sensor")
                Invalid("unknown entity role");
            std::string ros_namespace;
            if (e.isMember("parameters")) {
                Fields(e["parameters"], {"ros_namespace"});
                if (e["parameters"].isMember("ros_namespace")) {
                    ros_namespace = String(e["parameters"]["ros_namespace"]);
                    if (ros_namespace.front() != '/' || ros_namespace.size() > 128 ||
                        ros_namespace.find("..") != std::string::npos)
                        Invalid("ros_namespace must be an absolute bounded ROS namespace");
                }
            }
            if (e.isMember("sensors") && !e["sensors"].isArray())
                Invalid("entity sensors must be an array");
            if (e.isMember("sensors") && !e["sensors"].empty())
                throw DomainError(422, "unsupported",
                                  "create sensor attachments through the advertised sensor collection");
            c.pose = Pose(e["pose"]);
            Json::StreamWriterBuilder writer;
            writer["indentation"] = "";
            c.specification = Json::writeString(writer, e);
            const auto& a = e["asset"];
            Fields(a, {"id", "revision", "realization"});
            String(a["id"]);
            if (a.isMember("revision"))
                String(a["revision"]);
            const auto& f = a["realization"];
            Fields(f, {"media_type", "content", "uri"});
            auto media_type = String(f["media_type"]);
            if (media_type != "application/sdf+xml" && media_type != "application/urdf+xml")
                throw DomainError(422, "unsupported", "artifact type is unsupported");
            if (f.isMember("content") == f.isMember("uri"))
                Invalid("exactly one artifact content or uri required");
            std::string xml;
            if (f.isMember("content"))
                xml = String(f["content"]);
            else {
                auto relative = String(f["uri"]);
                if (std::filesystem::path(relative).is_absolute())
                    Invalid("artifact URI must be resource-root relative");
                auto path = std::filesystem::canonical(resource_root_ / relative);
                auto rel = path.lexically_relative(resource_root_);
                if (rel.empty() || *rel.begin() == ".." || !std::filesystem::is_regular_file(path))
                    Invalid("artifact URI escapes granted resource root");
                if (std::filesystem::file_size(path) > kArtifactBytes)
                    Invalid("artifact exceeds byte limit");
                std::ifstream in(path);
                xml.assign(std::istreambuf_iterator<char>(in), {});
            }
            if (xml.size() > kArtifactBytes)
                Invalid("artifact exceeds byte limit");
            c.artifact.reset(new sdf::SDF());
            sdf::init(c.artifact);
            if (!sdf::readString(xml, c.artifact) || !c.artifact->Root()->HasElement("model"))
                Invalid("artifact must contain one valid SDF model");
            auto m = c.artifact->Root()->GetElement("model");
            if (m->GetNextElement("model"))
                Invalid("artifact must contain exactly one model");
            if (m->HasElement("plugin")) {
                for (auto p = m->GetElement("plugin"); p; p = p->GetNextElement("plugin")) {
                    if (p->HasElement("chassisRobotId")) {
                        if (p->Get<std::string>("chassisRobotId") != c.id)
                            Invalid("chassisRobotId must match the public entity ID");
                        if (!xgc2::chassis_hold::valid_robot_id(c.id))
                            Invalid("a chassis entity ID must be 1-128 characters of [A-Za-z0-9._:-]");
                        c.chassis = true;
                    }
                }
            }
            if (!ros_namespace.empty()) {
                bool applied = false;
                if (m->HasElement("plugin")) {
                    for (auto p = m->GetElement("plugin"); p; p = p->GetNextElement("plugin")) {
                        if (p->HasElement("robotNamespace")) {
                            p->GetElement("robotNamespace")->Set(ros_namespace);
                            applied = true;
                        }
                    }
                }
                if (!applied)
                    throw DomainError(422, "unsupported", "asset has no advertised robotNamespace slot");
            }
            c.native_name = "xgc2_rpc_" + instance_ + "_" + std::to_string(++native_sequence_);
            m->GetAttribute("name")->Set(c.native_name);
            m->GetElement("pose")->Set(c.pose);
            auto plugin = m->AddElement("plugin");
            plugin->GetAttribute("name")->Set("xgc2_rpc_completion");
            plugin->GetAttribute("filename")->Set("libxgc2_simulation_entity_ack.so");
            auto public_id = sdf::ElementPtr(new sdf::Element());
            public_id->SetName("public_entity_id");
            public_id->AddValue("string", c.id, true);
            plugin->InsertElement(public_id);
            if (!ros_namespace.empty()) {
                auto public_namespace = sdf::ElementPtr(new sdf::Element());
                public_namespace->SetName("ros_namespace");
                public_namespace->AddValue("string", ros_namespace, true);
                plugin->InsertElement(public_namespace);
            }
            return c;
        }
        if (r.target.starts_with("/v1/entities/")) {
            auto path = r.target.substr(13);
            auto slash = path.find('/');
            c.id = Id(path.substr(0, slash));
            c.generation = UInt(b["generation"], 9007199254740991ULL);
            if (!c.generation)
                Invalid("generation must be positive");
            if (r.method == "DELETE" && slash == std::string::npos) {
                Fields(b, {"generation", "operation_timeout_ms"});
                c.kind = Kind::Remove;
            } else if (r.method == "POST" && slash != std::string::npos && path.substr(slash) == "/state") {
                Fields(b, {"generation", "state", "operation_timeout_ms"});
                c.kind = Kind::SetState;
                PrepareState(c, b["state"]);
            } else if (r.method == "POST" && slash != std::string::npos && path.substr(slash) == "/reset") {
                Fields(b, {"generation", "state", "operation_timeout_ms"});
                c.kind = Kind::ResetEntity;
                if (b.isMember("state"))
                    PrepareState(c, b["state"]);
            } else
                throw DomainError(422, "unsupported", "entity capability is unsupported");
            return c;
        }
        if (r.method != "POST")
            throw DomainError(404, "not_found", "route not found");
        if (r.target == "/v1/world/pause" || r.target == "/v1/world/resume") {
            Fields(b, {"operation_timeout_ms"});
            c.kind = r.target.ends_with("pause") ? Kind::Pause : Kind::Resume;
        } else if (r.target == "/v1/world/step") {
            Fields(b, {"steps", "operation_timeout_ms"});
            c.kind = Kind::Step;
            c.steps = UInt(b["steps"], 10000);
            if (!c.steps)
                Invalid("steps must be positive");
        } else if (r.target == "/v1/world/reset") {
            Fields(b, {"scope", "entities", "reset_time", "operation_timeout_ms"});
            c.kind = Kind::ResetWorld;
            if (!b["reset_time"].isBool())
                Invalid("reset_time must be explicit boolean");
            c.reset_time = b["reset_time"].asBool();
            auto scope = String(b["scope"]);
            if (scope == "entities") {
                const auto& refs = b["entities"];
                if (!refs.isArray() || refs.empty() || refs.size() > kEntities)
                    Invalid("invalid reset entity list");
                for (const auto& ref : refs) {
                    Fields(ref, {"id", "generation"});
                    auto id = Id(ref["id"]);
                    auto gen = UInt(ref["generation"], 9007199254740991ULL);
                    if (!gen)
                        Invalid("generation must be positive");
                    for (const auto& existing : c.refs)
                        if (existing.first == id)
                            Invalid("duplicate reset entity");
                    c.refs.emplace_back(id, gen);
                }
            } else if (scope != "all_entities" || b.isMember("entities"))
                Invalid("invalid reset scope");
        } else
            throw DomainError(422, "unsupported", "world capability is unsupported");
        return c;
    }
    static void PrepareState(Command& c, const Json::Value& s) {
        Fields(s, {"pose", "twist", "enabled"});
        if (s.empty())
            Invalid("state must have at least one field");
        if (s.isMember("pose")) {
            c.pose = Pose(s["pose"]);
            c.has_pose = true;
        }
        if (s.isMember("twist")) {
            Fields(s["twist"], {"linear", "angular"});
            c.linear = Vector(s["twist"]["linear"]);
            c.angular = Vector(s["twist"]["angular"]);
            c.has_twist = true;
        }
        if (s.isMember("enabled")) {
            if (!s["enabled"].isBool())
                Invalid("enabled must be boolean");
            c.enabled = s["enabled"].asBool();
            c.has_enabled = true;
        }
    }
    Json::Value Result(const NativeResult& r) {
        if (!r.extension_result.empty())
            return Parse(r.extension_result);
        Json::Value v;
        v["time"]["epoch"] = Json::UInt64(r.epoch);
        v["time"]["nanoseconds"] = std::to_string(r.nanoseconds);
        v["paused"] = r.paused;
        v["step_size_seconds"] = r.step_size;
        v["entities"] = Json::Value(Json::arrayValue);
        for (const auto& s : r.entities)
            v["entities"].append(Encode(s));
        return v;
    }
    std::string SceneSnapshot() {
        auto snapshot = Parse(scene_->Snapshot());
        snapshot["simulation_time"]["epoch"] = Json::UInt64(epoch_);
        snapshot["simulation_time"]["nanoseconds"] = std::to_string(native_clock_ns_.load());
        return Response(snapshot).body;
    }
    Json::Value Operation(const Slot& slot) {
        Json::Value v;
        v["id"] = slot.operation_id;
        v["kind"] = slot.kind;
        v["state"] = slot.state;
        if (slot.phase.load() == 4) {
            if (!slot.result.code.empty()) {
                v["error"]["code"] = slot.result.code;
                v["error"]["message"] = slot.result.message;
            } else if (slot.state == "succeeded")
                v["result"] = Result(slot.result);
            v["effects"]["started"] = slot.result.effects_started;
            if (slot.state == "succeeded")
                v["effects"]["applied"] = true;
            else if (slot.result.effects_started)
                v["effects"]["completion"] = "partial_or_unknown";
            else
                v["effects"]["applied"] = false;
        }
        return v;
    }
    void Handle(HttpRequest r, HttpReply reply) {
        try {
            Drain();
            if (r.target == "/v1/describe" && r.method == "GET") {
                if (!r.body.empty())
                    Invalid("describe body is unsupported");
                reply.complete(Response(Description()));
                return;
            }
            if (r.target == "/v1/health" && r.method == "GET") {
                if (!r.body.empty())
                    Invalid("health body is unsupported");
                reply.complete(Response(Health()));
                return;
            }
            if (r.target == "/v1/health/observe" && r.method == "POST") {
                const auto body = Parse(r.body);
                Fields(body, {"after_revision"});
                const auto after = UInt(body["after_revision"], 9007199254740991ULL);
                auto health = Health();
                if (health["revision"].asUInt64() != after) {
                    reply.complete(Response(health));
                    return;
                }
                if (health_observers_.size() >= 16)
                    throw DomainError(429, "resource_exhausted", "health observer capacity reached");
                health_observers_.push_back({std::move(reply), r.deadline, after});
                return;
            }
            if (r.target == "/v1/extensions/scene/observe" && r.method == "POST") {
                const auto body = Parse(r.body);
                Fields(body, {"after_serial"});
                const auto after = UInt(body["after_serial"], 9007199254740991ULL);
                if (scene_observers_.size() >= 16)
                    throw DomainError(429, "resource_exhausted", "scene observer capacity reached");
                scene_observers_.push_back({std::move(reply), r.deadline, after});
                scene_observers_active_.store(true);
                scene_snapshot_requested_.store(true);
                queue_cv_.notify_one();
                return;
            }
            if (detail::ChassisHoldHost::Calls(r.target)) {
                chassis_->Handle(std::move(r), std::move(reply));
                return;
            }
            if (r.target.starts_with("/v1/operations/")) {
                auto name = r.target.substr(15);
                auto slash = name.find('/');
                auto id = name.substr(0, slash);
                Slot* found = nullptr;
                for (auto& slot : slots_)
                    if (slot.phase.load() && slot.mutation && slot.operation_id == id) {
                        found = &slot;
                        break;
                    }
                if (!found)
                    throw DomainError(404, "not_found", "operation not retained; outcome unknown");
                if (r.method == "GET" && slash == std::string::npos && r.body.empty()) {
                    reply.complete(Response(Operation(*found)));
                    return;
                }
                if (r.method != "POST" || slash == std::string::npos)
                    Invalid("invalid operation observation");
                if (!r.body.empty())
                    Fields(Parse(r.body), {});
                auto action = name.substr(slash);
                if (action == "/cancel") {
                    found->cancel.store(true);
                    queue_cv_.notify_one();
                    reply.complete(Response(Operation(*found)));
                    return;
                }
                if (action != "/wait")
                    throw DomainError(404, "not_found", "route not found");
                if (found->phase.load() == 4) {
                    reply.complete(Response(Operation(*found)));
                    return;
                }
                if (waiters_ >= kWaiters)
                    throw DomainError(429, "resource_exhausted", "operation waiter limit reached");
                found->waiters.push_back({reply, r.deadline});
                ++waiters_;
                return;
            }
            bool mutation = r.method != "GET";
            if (mutation) {
                auto fingerprint = r.method + " " + r.target + "\n" + r.body;
                for (auto& slot : slots_)
                    if (slot.phase.load() && slot.mutation && slot.operation_id == r.request_id) {
                        if (slot.fingerprint != fingerprint)
                            throw DomainError(409, "conflict", "request ID reused with different request");
                        reply.complete(Response(Operation(slot), slot.phase.load() == 4 ? 200 : 202));
                        return;
                    }
            }
            auto command = Prepare(r);
            Slot* selected = nullptr;
            std::size_t index = 0;
            for (; index < slots_.size(); ++index)
                if (slots_[index].phase.load() == 0) {
                    selected = &slots_[index];
                    break;
                }
            if (!selected)
                throw DomainError(429, "resource_exhausted", "retained operation capacity reached");
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (queue_.size() >= kQueue)
                throw DomainError(429, "resource_exhausted", "native command queue full");
            auto& slot = *selected;
            slot.command = std::move(command);
            slot.result.entities.clear();
            slot.result.code.clear();
            slot.result.message.clear();
            slot.result.extension_result.clear();
            slot.result.effects_started = false;
            slot.cancel.store(false);
            slot.mutation = mutation;
            slot.operation_id = mutation ? r.request_id : "";
            slot.fingerprint = mutation ? r.method + " " + r.target + "\n" + r.body : "";
            slot.state = "accepted";
            slot.kind = r.target;
            slot.query_reply = mutation ? HttpReply{} : reply;
            slot.phase.store(1);
            queue_.push_back(index);
            ++pending_;
            if (mutation)
                reply.complete(Response(Operation(slot), 202));
            queue_cv_.notify_one();
        } catch (const DomainError& e) {
            reply.complete(xgc2::xrpc::http_error(e.status, e.code, e.what()));
        } catch (const SensorControlError& e) {
            reply.complete(xgc2::xrpc::http_error(e.status, e.code, e.what()));
        } catch (const NativeExtensionError& e) {
            reply.complete(xgc2::xrpc::http_error(e.status, e.code, e.what()));
        } catch (const std::invalid_argument& e) {
            reply.complete(xgc2::xrpc::http_error(400, "invalid_argument", e.what()));
        } catch (const std::exception&) {
            reply.complete(xgc2::xrpc::http_error(400, "invalid_argument", "invalid resource or request"));
        }
    }
    void Drain() {
        for (auto& slot : slots_) {
            if (slot.phase.load() == 2)
                slot.state = "running";
            if (slot.phase.load() == 3) {
                slot.state =
                    slot.result.code == "cancelled" ? "cancelled" : slot.result.code.empty() ? "succeeded" : "failed";
                slot.terminal_at = Clock::now();
                slot.phase.store(4);
                if (!slot.mutation) {
                    if (slot.result.code.empty())
                        slot.query_reply.complete(Response(Result(slot.result)));
                    else
                        slot.query_reply.complete(
                            xgc2::xrpc::http_error(slot.result.status, slot.result.code, slot.result.message));
                    slot.query_reply = {};
                    slot.phase.store(0);
                }
            }
            for (auto it = slot.waiters.begin(); it != slot.waiters.end();) {
                if (slot.phase.load() == 4) {
                    it->reply.complete(Response(Operation(slot)));
                    it = slot.waiters.erase(it);
                    --waiters_;
                } else if (it->reply.cancelled() ||
                           Clock::now().time_since_epoch().count() >= it->deadline.time_since_epoch().count()) {
                    it->reply.complete(
                        xgc2::xrpc::http_error(504, "deadline_exceeded", "observation deadline elapsed"));
                    it = slot.waiters.erase(it);
                    --waiters_;
                } else
                    ++it;
            }
            if (slot.phase.load() == 4 && slot.waiters.empty() &&
                (Clock::now() - slot.terminal_at).count() >=
                    std::chrono::duration_cast<Clock::duration>(std::chrono::minutes(5)).count()) {
                slot.fingerprint.clear();
                slot.command = {};
                slot.phase.store(0);
            }
        }
        const auto health = Health();
        for (auto it = health_observers_.begin(); it != health_observers_.end();) {
            if (health["revision"].asUInt64() != it->after) {
                it->reply.complete(Response(health));
                it = health_observers_.erase(it);
            } else if (it->reply.cancelled() ||
                       Clock::now().time_since_epoch().count() >= it->deadline.time_since_epoch().count()) {
                it->reply.complete(
                    xgc2::xrpc::http_error(504, "deadline_exceeded", "health observation deadline elapsed"));
                it = health_observers_.erase(it);
            } else
                ++it;
        }
        std::string snapshot;
        if (scene_snapshot_ready_.exchange(false)) {
            std::lock_guard<std::mutex> lock(scene_snapshot_mutex_);
            snapshot = scene_snapshot_;
        }
        const auto observed = snapshot.empty() ? Json::Value{} : Parse(snapshot);
        for (auto it = scene_observers_.begin(); it != scene_observers_.end();) {
            if (!observed.isNull() && observed["serial"].asUInt64() != it->after) {
                it->reply.complete(Response(observed));
                it = scene_observers_.erase(it);
            } else if (it->reply.cancelled() ||
                       Clock::now().time_since_epoch().count() >= it->deadline.time_since_epoch().count()) {
                it->reply.complete(
                    xgc2::xrpc::http_error(504, "deadline_exceeded", "scene observation deadline elapsed"));
                it = scene_observers_.erase(it);
            } else
                ++it;
        }
        scene_observers_active_.store(!scene_observers_.empty());
    }
    Entity& Find(const std::string& id, std::uint64_t generation = 0) {
        for (auto& e : entities_)
            if (e.occupied && e.id == id) {
                if (generation && generation != e.generation)
                    throw DomainError(409, "conflict", "stale entity generation");
                if (!e.model)
                    throw DomainError(503, "unavailable", "native entity realization pending");
                return e;
            }
        throw DomainError(404, "not_found", "entity not found");
    }
    NativeState Snapshot(const Entity& e, bool include_specification = true) {
        return {e.id,
                e.role,
                e.generation,
                e.model->WorldPose(),
                e.model->WorldLinearVel(),
                e.model->WorldAngularVel(),
                e.enabled,
                include_specification ? e.specification : "",
                e.native_ready,
                !e.scene_body.empty()};
    }
    void WorldState(NativeResult& r) {
        r.nanoseconds = native_clock_ns_.load();
        r.epoch = epoch_;
        r.paused = world_->IsPaused();
        r.step_size = world_->Physics()->GetMaxStepSize();
    }
    gazebo::physics::ModelPtr Realize(const sdf::SDFPtr& artifact, const std::string& name) {
        {
            std::lock_guard<std::mutex> lock(ack_mutex_);
            pending_name_ = name;
            pending_model_.reset();
        }
        world_->InsertModelSDF(*artifact);
        std::unique_lock<std::mutex> lock(ack_mutex_);
        ack_cv_.wait(lock, [this] {
            return pending_model_ || stopping_;
        });
        if (!pending_model_)
            throw DomainError(503, "unavailable", "world ended before native factory completion");
        auto model = std::move(pending_model_);
        pending_name_.clear();
        return model;
    }
    std::map<std::string, gazebo::physics::ModelPtr> ApplyScene(const std::map<std::string, SceneModel>& desired,
                                                                Clock::time_point deadline) {
        if (desired.size() > kEntities)
            throw DomainError(429, "resource_exhausted", "scene entity bound exceeded");
        std::size_t others = 0;
        for (const auto& e : entities_)
            if (e.occupied && e.scene_body.empty())
                ++others;
        if (desired.size() + others > kEntities)
            throw DomainError(429, "resource_exhausted", "shared world entity bound exceeded");
        // Validate native artifacts before changing membership.
        std::map<std::string, sdf::SDFPtr> artifacts;
        for (const auto& [id, model] : desired) {
            auto artifact = sdf::SDFPtr(new sdf::SDF());
            sdf::init(artifact);
            if (!sdf::readString(model.sdf, artifact))
                Invalid("invalid prepared scene SDF");
            auto plugin = artifact->Root()->GetElement("model")->AddElement("plugin");
            plugin->GetAttribute("name")->Set("xgc2_rpc_completion");
            plugin->GetAttribute("filename")->Set("libxgc2_simulation_entity_ack.so");
            artifacts.emplace(id, std::move(artifact));
        }
        if (Clock::now().time_since_epoch().count() >= deadline.time_since_epoch().count())
            throw DomainError(504, "deadline_exceeded", "scene expired before native application");
        for (auto& e : entities_)
            if (e.occupied && !e.scene_body.empty()) {
                auto match = desired.find(e.id.substr(6));
                if (match == desired.end() || match->second.body != e.scene_body) {
                    world_->RemoveModel(e.model);
                    e.model.reset();
                    e.occupied = false;
                }
            }
        std::map<std::string, gazebo::physics::ModelPtr> models;
        for (const auto& [id, definition] : desired) {
            Entity* entity = nullptr;
            for (auto& e : entities_)
                if (e.occupied && e.id == "scene:" + id) {
                    entity = &e;
                    break;
                }
            if (!entity) {
                for (auto& e : entities_)
                    if (!e.occupied) {
                        entity = &e;
                        break;
                    }
                if (!entity)
                    throw DomainError(429, "resource_exhausted", "scene entity capacity reached");
                *entity = {"scene:" + id,   "obstacle", definition.name, ++generation_, {}, definition.pose, true, true,
                           definition.body, ""};
                entity->model = Realize(artifacts.at(id), definition.name);
                entity->native_ready = true;
            }
            ModelSdfParser parser;
            if (!parser.Parse(definition.sdf))
                throw DomainError(500, "internal", "scene artifact parsing failed");
            boost::recursive_mutex::scoped_lock native_lock(*world_->Physics()->GetPhysicsUpdateMutex());
            if (!ApplySceneModelParameters(entity->model, definition, definition.pose, parser))
                throw DomainError(500, "internal", "scene native parameters could not be applied");
            std::string error;
            if (!VerifySceneModel(entity->model, definition, true, &error))
                throw DomainError(500, "internal", "scene native verification failed");
            models.emplace(id, entity->model);
        }
        return models;
    }
    void Execute(Slot& slot) {
        auto& c = slot.command;
        auto& r = slot.result;
        if (slot.cancel.load())
            throw DomainError(409, "cancelled", "cancelled before native application");
        if (Clock::now().time_since_epoch().count() >= c.expires.time_since_epoch().count())
            throw DomainError(504, "deadline_exceeded", "expired before native application");
        if (c.kind == Kind::QueryScene) {
            r.extension_result = SceneSnapshot();
            return;
        }
        if (c.kind == Kind::Sensor) {
            r.extension_result = Response(sensors_->Execute(c.sensor, &r.effects_started)).body;
            return;
        }
        if (c.kind == Kind::Extension) {
            r.extension_result = Response(c.extension_owner->Execute(c.extension, &r.effects_started)).body;
            return;
        }
        if (c.kind == Kind::Scene || c.kind == Kind::Motion) {
            r.extension_result = c.kind == Kind::Scene ? scene_->Execute(c.scene, &r.effects_started)
                                                       : motion_->Execute(c.motion, &r.effects_started);
            return;
        }
        if (c.kind == Kind::Create) {
            for (const auto& e : entities_)
                if (e.occupied && e.id == c.id)
                    throw DomainError(409, "conflict", "entity ID already exists");
            Entity* entity = nullptr;
            for (auto& e : entities_)
                if (!e.occupied) {
                    entity = &e;
                    break;
                }
            if (!entity)
                throw DomainError(429, "resource_exhausted", "entity capacity reached");
            const auto generation = ++generation_;
            sensors_->PrepareParentSensors(c.id, generation, c.artifact);
            *entity = {c.id, c.role, c.native_name, generation, {}, c.pose, true, true, "", c.specification};
            r.effects_started = true;
            // Factory submission is irreversible. Expiry/disconnect does not make
            // the operation terminal; the native last-plugin ack reconciles it.
            entity->model = Realize(c.artifact, c.native_name);
            sensors_->CompleteParentSensors(entity->id, entity->generation);
            if (c.chassis && !chassis_->Bound(c.id))
                throw DomainError(503, "unavailable", "native chassis initialization did not complete");
            entity->native_ready = true;
            r.entities.push_back(Snapshot(*entity));
            WorldState(r);
            return;
        }
        if (c.kind == Kind::Step) {
            if (!world_->IsPaused())
                throw DomainError(409, "conflict", "step requires paused world");
            r.effects_started = true;
            auto before = world_->Iterations();
            world_->Step(c.steps);
            if (world_->Iterations() - before != c.steps)
                throw DomainError(503, "unavailable", "world ended during step");
            WorldState(r);
            return;
        }
        if (c.kind == Kind::ResetWorld) {
            for (const auto& ref : c.refs) {
                const auto& e = Find(ref.first, ref.second);
                if (!e.scene_body.empty())
                    throw DomainError(422, "unsupported", "partial scene reset requires scene motion reset scope");
            }
            r.effects_started = true;
            if (c.refs.empty()) {
                scene_->Reset();
                motion_->Reset();
            } else
                for (const auto& ref : c.refs)
                    motion_->Reset(ref.first, ref.second);
        }
        if (c.kind == Kind::ResetEntity) {
            const auto& e = Find(c.id, c.generation);
            if (!e.scene_body.empty())
                throw DomainError(422, "unsupported", "scene entity reset requires scene motion reset scope");
            r.effects_started = true;
            motion_->Reset(c.id, c.generation);
        }
        if (c.kind == Kind::Remove || c.kind == Kind::SetState) {
            auto& entity = Find(c.id, c.generation);
            if (!entity.scene_body.empty())
                throw DomainError(422, "unsupported", "scene membership and state are owned by the scene extension");
            if (c.kind == Kind::Remove) {
                sensors_->RemoveParent(entity.id, entity.generation, 5000, &r.effects_started);
                {
                    // The model plugin is destroyed after Gazebo has finalized its joints and links: the
                    // HOLD seat ends here, before the removal.
                    boost::recursive_mutex::scoped_lock physics(*world_->Physics()->GetPhysicsUpdateMutex());
                    r.effects_started = true;
                    chassis_->Retire(entity.id);
                }
            } else {
                r.effects_started = true;
                motion_->Reset(entity.id, entity.generation);
            }
        }
        // Gazebo native management APIs use the same physics update mutex. The
        // executor has called InitForThread; it is independent of the HTTP loop.
        // SetPaused takes World's update mutex. Never acquire it while holding
        // the physics mutex: World::Update takes those locks in the other order.
        if (c.kind == Kind::Pause || c.kind == Kind::Resume) {
            r.effects_started = true;
            world_->SetPaused(c.kind == Kind::Pause);
            WorldState(r);
            return;
        }
        bool resume_after_reset = c.kind == Kind::ResetWorld && c.reset_time && !world_->IsPaused();
        if (resume_after_reset)
            world_->SetPaused(true);
        {
            boost::recursive_mutex::scoped_lock native_lock(*world_->Physics()->GetPhysicsUpdateMutex());
            if (c.kind == Kind::QueryWorld) {
            } else if (c.kind == Kind::QueryEntities) {
                for (const auto& e : entities_)
                    if (e.occupied && e.model)
                        r.entities.push_back(Snapshot(e, false));
            } else if (c.kind == Kind::ResetWorld) {
                // Fence every selected reference before performing any reset.
                for (const auto& ref : c.refs)
                    Find(ref.first, ref.second);
                r.effects_started = true;
                for (auto& e : entities_)
                    if (e.occupied && e.model) {
                        bool selected = c.refs.empty();
                        for (const auto& ref : c.refs)
                            selected |= e.id == ref.first;
                        if (selected) {
                            if (!chassis_->ZeroOutput(e.id))
                                throw DomainError(503, "unavailable", "native chassis reset output failed");
                            e.model->Reset();
                            e.model->SetWorldPose(e.initial);
                            e.enabled = true;
                            e.model->SetEnabled(true);
                        }
                        r.entities.push_back(Snapshot(e, false));
                    }
                if (c.reset_time) {
                    world_->ResetTime();
                    ++epoch_;
                }
            } else {
                auto& e = Find(c.id, c.generation);
                if (c.kind == Kind::Remove) {
                    r.effects_started = true;
                    motion_->Remove(e.id, e.generation);
                    world_->RemoveModel(e.model);
                    e.model.reset();
                    e.occupied = false;
                } else {
                    if (c.kind == Kind::ResetEntity) {
                        r.effects_started = true;
                        if (!chassis_->ZeroOutput(e.id))
                            throw DomainError(503, "unavailable", "native chassis reset output failed");
                        e.model->Reset();
                        e.model->SetWorldPose(e.initial);
                        e.model->SetEnabled(true);
                        e.enabled = true;
                    }
                    if (c.kind == Kind::ResetEntity || c.kind == Kind::SetState) {
                        r.effects_started = true;
                        if (c.has_pose)
                            e.model->SetWorldPose(c.pose);
                        if (c.has_twist) {
                            e.model->SetLinearVel(c.linear);
                            e.model->SetAngularVel(c.angular);
                        }
                        if (c.has_enabled) {
                            e.model->SetEnabled(c.enabled);
                            e.enabled = c.enabled;
                        }
                    }
                    r.entities.push_back(Snapshot(e));
                }
            }
            WorldState(r);
        }
        if (resume_after_reset) {
            world_->SetPaused(false);
            r.paused = false;
        }
    }
    void Work() {
        world_->Physics()->InitForThread();
        while (!stopping_) {
            std::size_t index = slots_.size();
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this] {
                    return stopping_ || !queue_.empty() || scene_snapshot_requested_.load();
                });
                if (stopping_)
                    break;
                if (!queue_.empty()) {
                    index = queue_.front();
                    queue_.pop_front();
                }
            }
            if (scene_snapshot_requested_.exchange(false) && scene_observers_active_) {
                try {
                    auto snapshot = SceneSnapshot();
                    {
                        std::lock_guard<std::mutex> lock(scene_snapshot_mutex_);
                        scene_snapshot_ = std::move(snapshot);
                    }
                    scene_snapshot_ready_.store(true);
                    server_->wake();
                } catch (...) { /* Waiters retain their deadlines; no fabricated snapshot. */
                }
            }
            if (index == slots_.size())
                continue;
            auto& slot = slots_[index];
            slot.phase.store(2);
            server_->wake();
            try {
                Execute(slot);
            } catch (const DomainError& e) {
                slot.result.status = e.status;
                slot.result.code = e.code;
                slot.result.message = e.what();
            } catch (const SensorControlError& e) {
                slot.result.status = e.status;
                slot.result.code = e.code;
                slot.result.message = e.what();
            } catch (const NativeExtensionError& e) {
                slot.result.status = e.status;
                slot.result.code = e.code;
                slot.result.message = e.what();
            } catch (const std::invalid_argument& e) {
                const std::string message = e.what();
                const bool capacity =
                    message == "scene epoch capacity exceeded" || message == "entity motion capacity exceeded";
                slot.result.status = capacity ? 429 : 409;
                slot.result.code = capacity ? "resource_exhausted" : "conflict";
                slot.result.message = message;
            } catch (const std::exception&) {
                slot.result.status = 500;
                slot.result.code = "internal";
                slot.result.message = "native engine operation failed";
            }
            slot.phase.store(3);
            --pending_;
            server_->wake();
            if (scene_observers_active_)
                scene_snapshot_requested_.store(true);
        }
    }
    gazebo::physics::WorldPtr world_;
    std::string path_, target_;
    std::filesystem::path resource_root_;
    std::string instance_, configuration_revision_;
    std::unique_ptr<xgc2::xrpc::HttpServer> server_;
    xgc2::xrpc::HttpLimits limits_;
    std::shared_ptr<detail::ChassisHoldHost> chassis_;
    std::atomic<bool> native_ready_{false};
    std::vector<Component> components_;
    std::vector<Observer> health_observers_, scene_observers_;
    std::atomic<bool> scene_observers_active_{false}, scene_snapshot_requested_{false}, scene_snapshot_ready_{false};
    std::mutex scene_snapshot_mutex_;
    std::string scene_snapshot_;
    std::mutex extension_mutex_;
    std::map<std::string, std::shared_ptr<NativeWorldExtension>> extensions_;
    std::unique_ptr<NativeSceneController> scene_;
    std::unique_ptr<NativeEntityMotion> motion_;
    std::unique_ptr<NativeSensorController> sensors_;
    std::filesystem::path mesh_root_;
    std::shared_ptr<UpdateGate> update_gate_;
    gazebo::event::ConnectionPtr update_, time_reset_;
    std::atomic<std::int64_t> native_clock_ns_{0};
    std::thread io_, worker_;
    std::atomic<bool> stopping_{false};
    std::atomic<unsigned> pending_{0};
    std::mutex queue_mutex_, ack_mutex_;
    std::condition_variable queue_cv_, ack_cv_;
    std::deque<std::size_t> queue_;
    std::string pending_name_;
    gazebo::physics::ModelPtr pending_model_;
    std::array<Slot, kReceipts> slots_;
    std::array<Entity, kEntities> entities_;
    std::size_t waiters_ = 0;
    std::uint64_t epoch_ = 0, generation_ = 0, native_sequence_ = 0;
};

SimulationService::SimulationService(gazebo::physics::WorldPtr world, std::string path, std::string target,
                                     const std::string& root,
                                     const std::vector<std::string>& required_components,
                                     std::string configuration_revision) {
    auto* identity = world.get();
    impl_ = std::make_unique<Impl>(std::move(world), std::move(path), std::move(target), root, required_components,
                                   std::move(configuration_revision));
    std::vector<std::shared_ptr<detail::WorldStartupState>> attached;
    {
        std::lock_guard<std::mutex> lock(registry_mutex);
        if (registry.count(identity))
            throw std::runtime_error("world already has a simulation authority");
        registry.emplace(identity, this);
        for (const auto& pending : startup_bindings)
            if (auto state = pending.lock(); state && state->world.get() == identity)
                attached.push_back(std::move(state));
    }
    try {
        for (const auto& state : attached)
            state->Invoke();
    } catch (...) {
        std::lock_guard<std::mutex> lock(registry_mutex);
        registry.erase(identity);
        throw;
    }
}
SimulationService::~SimulationService() {
    {
        std::lock_guard<std::mutex> lock(registry_mutex);
        for (auto i = registry.begin(); i != registry.end();) {
            if (i->second == this)
                i = registry.erase(i);
            else
                ++i;
        }
    }
    Stop();
}
void SimulationService::Start() {
    impl_->Start();
}
void SimulationService::Stop() {
    impl_->Stop();
}
void SimulationService::Acknowledge(gazebo::physics::ModelPtr model) {
    impl_->Acknowledge(std::move(model));
}
void SimulationService::MarkNativeReady() noexcept {
    impl_->MarkNativeReady();
}
std::shared_ptr<detail::NativeComponentState> SimulationService::AttachComponent(const std::string& id) {
    return impl_->AttachComponent(id);
}
void SimulationService::RegisterExtension(const std::string& id, std::shared_ptr<NativeWorldExtension> adapter) {
    impl_->RegisterExtension(id, std::move(adapter));
}
void SimulationService::UnregisterExtension(const std::string& id,
                                            const std::shared_ptr<NativeWorldExtension>& adapter) {
    impl_->UnregisterExtension(id, adapter);
}
WorldExtensionBinding::WorldExtensionBinding(gazebo::physics::WorldPtr world, std::string id,
                                             std::shared_ptr<NativeWorldExtension> adapter)
    : world_(std::move(world)), id_(std::move(id)), adapter_(std::move(adapter)) {
    std::lock_guard<std::mutex> lock(registry_mutex);
    const auto found = registry.find(world_.get());
    if (found == registry.end())
        throw std::runtime_error("world has no native simulation authority");
    found->second->RegisterExtension(id_, adapter_);
}
WorldExtensionBinding::~WorldExtensionBinding() {
    adapter_->Stop();
    std::lock_guard<std::mutex> lock(registry_mutex);
    const auto found = registry.find(world_.get());
    if (found != registry.end())
        found->second->UnregisterExtension(id_, adapter_);
}
WorldStartupBinding::WorldStartupBinding(gazebo::physics::WorldPtr world,
                                         std::function<void(gazebo::physics::WorldPtr)> attach)
    : state_(std::make_shared<detail::WorldStartupState>()) {
    if (!world || !attach)
        throw std::invalid_argument("native startup attachment requires world and callback");
    state_->world = std::move(world);
    state_->attach = std::move(attach);
    bool ready;
    {
        std::lock_guard<std::mutex> lock(registry_mutex);
        for (auto it = startup_bindings.begin(); it != startup_bindings.end();) {
            if (it->expired())
                it = startup_bindings.erase(it);
            else
                ++it;
        }
        if (startup_bindings.size() >= 16)
            throw std::runtime_error("native startup attachment bound exceeded");
        startup_bindings.push_back(state_);
        ready = registry.count(state_->world.get()) != 0;
    }
    if (ready)
        state_->Invoke();
}
WorldStartupBinding::~WorldStartupBinding() {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->closed = true;
    state_->attach = {};
}
namespace detail {
std::shared_ptr<NativeComponentState> AttachNativeComponent(const gazebo::physics::WorldPtr& world,
                                                            const std::string& id) {
    std::lock_guard<std::mutex> lock(registry_mutex);
    auto found = registry.find(world.get());
    if (found == registry.end())
        throw std::runtime_error("world has no native simulation authority");
    return found->second->AttachComponent(id);
}
} // namespace detail
void AcknowledgeSimulationModel(gazebo::physics::ModelPtr model) {
    std::lock_guard<std::mutex> lock(registry_mutex);
    auto i = registry.find(model->GetWorld().get());
    if (i != registry.end())
        i->second->Acknowledge(std::move(model));
}
} // namespace xgc2_gazebo_scene
