#include "xgc2_gazebo_scene/native_scene_controller.hpp"
#include <atomic>
#include <cmath>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <json/json.h>
#include <mutex>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace xgc2_gazebo_scene {
namespace {
using JsonValue = Json::Value;

void Fields(const JsonValue& value, std::initializer_list<const char*> allowed) {
    if (!value.isObject())
        throw std::invalid_argument("scene value must be an object");
    for (const auto& name : value.getMemberNames()) {
        bool found = false;
        for (const auto* field : allowed)
            found = found || name == field;
        if (!found)
            throw std::invalid_argument("unknown scene field: " + name);
    }
}
double Number(const JsonValue& value, double low = -1e6, double high = 1e6) {
    if (!value.isNumeric() || !std::isfinite(value.asDouble()) || value.asDouble() < low || value.asDouble() > high)
        throw std::invalid_argument("scene number is outside its finite supported range");
    return value.asDouble();
}
std::uint64_t Integer(const JsonValue& value, std::uint64_t low, std::uint64_t high) {
    if ((value.type() != Json::intValue && value.type() != Json::uintValue) || !value.isUInt64() ||
        value.asUInt64() < low || value.asUInt64() > high)
        throw std::invalid_argument("scene integer is outside its supported range");
    return value.asUInt64();
}
std::string Id(const JsonValue& value) {
    static const std::regex pattern("[A-Za-z0-9][A-Za-z0-9_.-]{0,127}");
    if (!value.isString() || !std::regex_match(value.asString(), pattern))
        throw std::invalid_argument("invalid scene identifier");
    return value.asString();
}
ignition::math::Vector3d Vector(const JsonValue& value) {
    if (!value.isArray() || value.size() != 3)
        throw std::invalid_argument("three scene vector components required");
    return {Number(value[0]), Number(value[1]), Number(value[2])};
}
geometry_msgs::Pose Pose(const JsonValue& value) {
    Fields(value, {"position", "orientation"});
    const auto p = Vector(value["position"]);
    const auto& q = value["orientation"];
    if (!q.isArray() || q.size() != 4)
        throw std::invalid_argument("xyzw quaternion required");
    geometry_msgs::Pose result;
    result.position.x = p.X();
    result.position.y = p.Y();
    result.position.z = p.Z();
    const double x = Number(q[0]), y = Number(q[1]), z = Number(q[2]), w = Number(q[3]);
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (std::abs(norm - 1.0) > 1e-9)
        throw std::invalid_argument("unit scene quaternion required");
    result.orientation.x = x;
    result.orientation.y = y;
    result.orientation.z = z;
    result.orientation.w = w;
    return result;
}
std::string Encode(const JsonValue& value) {
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    return Json::writeString(builder, value);
}
JsonValue NativeVector(const ignition::math::Vector3d& vector) {
    JsonValue result(Json::arrayValue);
    for (const double component : {vector.X(), vector.Y(), vector.Z()}) {
        if (!std::isfinite(component))
            throw std::runtime_error("nonfinite native scene state");
        result.append(component);
    }
    return result;
}
JsonValue NativePose(const ignition::math::Pose3d& pose) {
    JsonValue result;
    result["position"] = NativeVector(pose.Pos());
    JsonValue orientation(Json::arrayValue);
    for (const double component : {pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W()}) {
        if (!std::isfinite(component))
            throw std::runtime_error("nonfinite native scene orientation");
        orientation.append(component);
    }
    result["orientation"] = std::move(orientation);
    return result;
}
struct Trajectory {
    std::string kind;
    ignition::math::Pose3d initial;
    ignition::math::Vector3d linear, angular, a, b;
    double radius = 0, speed = 0, phase = 0;
    void Sample(double elapsed, bool playing, ignition::math::Pose3d& pose, ignition::math::Vector3d& v,
                ignition::math::Vector3d& w) const {
        pose = initial;
        v = ignition::math::Vector3d::Zero;
        w = v;
        if (kind == "constant_twist") {
            pose.Pos() += elapsed * linear;
            const double norm = angular.Length();
            if (norm > 1e-12) {
                pose.Rot() = ignition::math::Quaterniond(angular / norm, norm * elapsed) * initial.Rot();
                pose.Rot().Normalize();
            }
            v = linear;
            w = angular;
        } else if (kind == "ping_pong") {
            const auto delta = b - a;
            const double length = delta.Length(), t = std::fmod(elapsed * speed / length, 2.0);
            pose.Pos() = a + (t < 1.0 ? t : 2.0 - t) * delta;
            v = (t < 1.0 ? 1.0 : -1.0) * speed / length * delta;
        } else if (kind == "circle") {
            const double angle = phase + elapsed * speed;
            pose.Pos().Set(a.X() + radius * std::cos(angle), a.Y() + radius * std::sin(angle), a.Z());
            v.Set(-radius * speed * std::sin(angle), radius * speed * std::cos(angle), 0);
        }
        if (!playing) {
            v = ignition::math::Vector3d::Zero;
            w = v;
        }
    }
};
Trajectory ParseMotion(const JsonValue& value, const ignition::math::Pose3d& initial) {
    if (!value.isObject() || !value["type"].isString())
        throw std::invalid_argument("scene motion type required");
    Trajectory t;
    t.initial = initial;
    t.kind = value["type"].asString();
    if (t.kind == "hold")
        Fields(value, {"type"});
    else if (t.kind == "constant_twist") {
        Fields(value, {"type", "linear", "angular"});
        t.linear = Vector(value["linear"]);
        t.angular = Vector(value["angular"]);
    } else if (t.kind == "ping_pong") {
        Fields(value, {"type", "point_a", "point_b", "speed"});
        t.a = Vector(value["point_a"]);
        t.b = Vector(value["point_b"]);
        t.speed = Number(value["speed"], 1e-6, 1000);
        if ((t.b - t.a).Length() < 1e-6 || (initial.Pos() - t.a).Length() > 1e-6)
            throw std::invalid_argument("invalid ping_pong endpoints or initial pose");
    } else if (t.kind == "circle") {
        Fields(value, {"type", "center", "radius", "angular_speed", "phase"});
        t.a = Vector(value["center"]);
        t.radius = Number(value["radius"], 1e-6, 1e4);
        t.speed = Number(value["angular_speed"], -1000, 1000);
        t.phase = Number(value["phase"]);
        ignition::math::Vector3d start(t.a.X() + t.radius * std::cos(t.phase), t.a.Y() + t.radius * std::sin(t.phase),
                                       t.a.Z());
        if ((initial.Pos() - start).Length() > 1e-6)
            throw std::invalid_argument("circle initial pose disagrees with path");
    } else
        throw std::invalid_argument("unsupported scene motion");
    return t;
}
} // namespace

struct PreparedSceneCommand {
    bool apply = false;
    std::string epoch, operation, identity;
    std::uint64_t revision = 0;
    std::chrono::milliseconds timeout{30000};
    std::map<std::string, SceneModel> models;
    std::map<std::string, Trajectory> trajectories;
    JsonValue definition;
};

class NativeSceneController::Impl {
  public:
    gazebo::physics::WorldPtr world;
    std::string mesh_root;
    NativeSceneApply apply;
    std::function<std::int64_t()> clock;
    mutable std::mutex mutex;
    std::string epoch, identity;
    std::uint64_t revision = 0;
    std::atomic<std::uint64_t> change_serial{0};
    bool playing = false, suspended = false;
    double elapsed = 0, started = 0;
    std::map<std::string, gazebo::physics::ModelPtr> models;
    std::map<std::string, Trajectory> trajectories;
    std::set<std::string> retired_epochs;
    JsonValue definition;
    double sampled_time = -1;
    double Time(double now) const { return elapsed + (playing ? std::max(0.0, now - started) : 0.0); }
    JsonValue Status(double now) const {
        JsonValue value;
        value["epoch"] = epoch;
        value["revision"] = Json::UInt64(revision);
        value["playing"] = playing;
        value["scene_time"] = Time(now);
        value["applied"] = !suspended;
        return value;
    }
    void Sample(double now) {
        const double time = Time(now);
        for (const auto& entry : trajectories) {
            const auto found = models.find(entry.first);
            if (found == models.end())
                continue;
            ignition::math::Pose3d pose;
            ignition::math::Vector3d linear, angular;
            entry.second.Sample(time, playing, pose, linear, angular);
            found->second->SetWorldPose(pose);
            found->second->SetLinearVel(linear);
            found->second->SetAngularVel(angular);
        }
        if (playing && sampled_time != time)
            change_serial.fetch_add(1, std::memory_order_release);
        sampled_time = time;
    }
};

NativeSceneController::NativeSceneController(gazebo::physics::WorldPtr world, std::string root, NativeSceneApply apply,
                                             std::function<std::int64_t()> clock)
    : impl_(std::make_unique<Impl>()) {
    if (!world || root.empty() || !apply || !clock)
        throw std::invalid_argument("native scene world, mesh root, clock and apply owner required");
    impl_->world = std::move(world);
    impl_->mesh_root = std::move(root);
    impl_->apply = std::move(apply);
    impl_->clock = std::move(clock);
    impl_->definition["obstacles"] = JsonValue(Json::arrayValue);
}
NativeSceneController::~NativeSceneController() = default;

std::shared_ptr<const PreparedSceneCommand> NativeSceneController::Prepare(const std::string& route,
                                                                           const std::string& text) {
    if (text.size() > 8 * 1024 * 1024)
        throw std::invalid_argument("scene request exceeds 8 MiB");
    Json::CharReaderBuilder builder;
    builder["rejectDupKeys"] = true;
    builder["failIfExtra"] = true;
    builder["allowComments"] = false;
    builder["strictRoot"] = true;
    builder["stackLimit"] = 64;
    JsonValue value;
    std::string error;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(text.data(), text.data() + text.size(), &value, &error))
        throw std::invalid_argument("invalid scene JSON");
    auto command = std::make_shared<PreparedSceneCommand>();
    command->apply = route == "/v1/extensions/scene/apply";
    if (!command->apply && route != "/v1/extensions/scene/motion")
        throw std::invalid_argument("unknown scene route");
    Fields(value, command->apply
                      ? std::initializer_list<const char*>{"epoch", "revision", "document", "operation_timeout_ms"}
                      : std::initializer_list<const char*>{"epoch", "revision", "operation", "operation_timeout_ms"});
    if (!value["epoch"].isString() || value["epoch"].asString().empty() || value["epoch"].asString().size() > 128)
        throw std::invalid_argument("scene epoch required");
    command->epoch = value["epoch"].asString();
    command->revision = Integer(value["revision"], 1, 9007199254740991ULL);
    command->timeout = std::chrono::milliseconds(Integer(value["operation_timeout_ms"], 1, 60000));
    if (!command->apply) {
        if (!value["operation"].isString())
            throw std::invalid_argument("scene motion operation required");
        command->operation = value["operation"].asString();
        if (command->operation != "play" && command->operation != "pause" && command->operation != "reset")
            throw std::invalid_argument("unsupported scene motion operation");
        return command;
    }
    const auto& doc = value["document"];
    Fields(doc, {"schema", "id", "frame", "obstacles"});
    if (doc["schema"] != "xgc2.scene.v1" || doc["frame"] != "world")
        throw std::invalid_argument("scene v1 in world frame required");
    xgc2_geometry_msgs::SceneSnapshot snapshot;
    snapshot.scene_id = Id(doc["id"]);
    snapshot.epoch = command->epoch;
    snapshot.revision = command->revision;
    snapshot.header.frame_id = "world";
    const auto& obstacles = doc["obstacles"];
    if (!obstacles.isArray() || obstacles.size() > 256)
        throw std::invalid_argument("scene obstacle bound exceeded");
    std::size_t parts = 0;
    command->definition["obstacles"] = JsonValue(Json::arrayValue);
    for (const auto& item : obstacles) {
        Fields(item, {"id", "name", "pose", "parts", "motion"});
        xgc2_geometry_msgs::SceneObstacle body;
        body.id = Id(item["id"]);
        if (!item["name"].isString())
            throw std::invalid_argument("scene obstacle name must be a string");
        body.name = item["name"].asString();
        if (body.name.empty() || body.name.size() > 256)
            throw std::invalid_argument("scene obstacle name required");
        body.pose = Pose(item["pose"]);
        const auto trajectory = ParseMotion(item["motion"], ScenePose(body.pose));
        if (!command->trajectories.emplace(body.id, trajectory).second)
            throw std::invalid_argument("duplicate scene obstacle id");
        body.motion_type = trajectory.kind;
        body.dynamic = trajectory.kind != "hold";
        JsonValue frozen;
        frozen["id"] = body.id;
        frozen["dynamic"] = body.dynamic;
        frozen["motion_type"] = trajectory.kind;
        frozen["pose"] = item["pose"];
        frozen["parts"] = JsonValue(Json::arrayValue);
        const auto& list = item["parts"];
        if (!list.isArray() || list.empty() || list.size() > 128 || (parts += list.size()) > 4096)
            throw std::invalid_argument("scene part bound exceeded");
        for (const auto& part : list) {
            Fields(part, {"id", "pose", "geometry", "color"});
            xgc2_geometry_msgs::ScenePart p;
            p.id = Id(part["id"]);
            p.pose = Pose(part["pose"]);
            const auto& color = part["color"];
            if (part.isMember("color") && (!color.isArray() || color.size() != 4 || Number(color[0]) != 1.0 ||
                                           Number(color[1]) != .5 || Number(color[2]) != .1 || Number(color[3]) != 1.0))
                throw std::invalid_argument("scene v1 uses the declared opaque amber part color");
            p.color.r = 1.0;
            p.color.g = .5;
            p.color.b = .1;
            p.color.a = 1.0;
            const auto& geometry = part["geometry"];
            if (!geometry.isObject() || !geometry["type"].isString())
                throw std::invalid_argument("scene geometry type required");
            p.geometry.type = geometry["type"].asString();
            if (p.geometry.type == "box") {
                Fields(geometry, {"type", "size"});
                const auto size = Vector(geometry["size"]);
                p.geometry.size.x = size.X();
                p.geometry.size.y = size.Y();
                p.geometry.size.z = size.Z();
            } else if (p.geometry.type == "sphere" || p.geometry.type == "cylinder" || p.geometry.type == "capsule") {
                Fields(geometry, p.geometry.type == "sphere"
                                     ? std::initializer_list<const char*>{"type", "radius"}
                                     : std::initializer_list<const char*>{"type", "radius", "height"});
                p.geometry.radius = Number(geometry["radius"], 1e-6, 1e4);
                if (p.geometry.type != "sphere")
                    p.geometry.height = Number(geometry["height"], p.geometry.type == "capsule" ? 0 : 1e-6, 1e4);
            } else if (p.geometry.type == "convex") {
                Fields(geometry, {"type", "vertices", "triangles"});
                const auto& vertices = geometry["vertices"];
                const auto& triangles = geometry["triangles"];
                if (!vertices.isArray() || vertices.size() < 4 || vertices.size() > 4096 || !triangles.isArray() ||
                    triangles.size() < 12 || triangles.size() > 24576 || triangles.size() % 3)
                    throw std::invalid_argument("invalid convex scene bounds");
                for (const auto& point : vertices) {
                    const auto v = Vector(point);
                    geometry_msgs::Point out;
                    out.x = v.X();
                    out.y = v.Y();
                    out.z = v.Z();
                    p.geometry.vertices.push_back(out);
                }
                for (const auto& index : triangles) {
                    p.geometry.triangles.push_back(Integer(index, 0, vertices.size() - 1));
                }
            } else
                throw std::invalid_argument("unsupported scene geometry");
            JsonValue frozen_part;
            frozen_part["id"] = p.id;
            frozen_part["pose"] = part["pose"];
            frozen_part["geometry"] = geometry;
            frozen["parts"].append(std::move(frozen_part));
            body.parts.push_back(std::move(p));
        }
        command->definition["obstacles"].append(std::move(frozen));
        snapshot.obstacles.push_back(std::move(body));
    }
    if (!CompileScene(snapshot, impl_->mesh_root, &command->models, &error))
        throw std::invalid_argument(error);
    command->identity = Encode(doc);
    return command;
}

std::string NativeSceneController::Execute(const std::shared_ptr<const PreparedSceneCommand>& command,
                                           bool* effects_started) {
    if (effects_started)
        *effects_started = false;
    if (!command)
        throw std::invalid_argument("prepared scene command required");
    auto& s = *impl_;
    const auto deadline = std::chrono::steady_clock::now() + command->timeout;
    {
        std::lock_guard<std::mutex> lock(s.mutex);
        if (s.retired_epochs.count(command->epoch))
            throw std::invalid_argument("retired scene epoch");
        if (command->apply) {
            if (command->epoch != s.epoch && s.retired_epochs.size() >= 256)
                throw std::invalid_argument("scene epoch capacity exceeded");
            if (command->epoch == s.epoch && (command->revision < s.revision ||
                                              (command->revision == s.revision && command->identity != s.identity)))
                throw std::invalid_argument("scene revision conflict");
            if (command->epoch == s.epoch && command->revision == s.revision && !s.suspended)
                return Encode(s.Status(s.clock() * 1e-9));
            // All local revision/capacity fences precede suspension. Suspending
            // native trajectory updates is itself a configuration effect.
            if (effects_started)
                *effects_started = true;
            s.suspended = true;
            s.change_serial.fetch_add(1, std::memory_order_release);
        } else {
            if (command->epoch != s.epoch || command->revision != s.revision || s.suspended)
                throw std::invalid_argument("scene motion requires the applied scene revision");
            const double now = s.clock() * 1e-9;
            if (effects_started)
                *effects_started = true;
            s.elapsed = command->operation == "reset" ? 0.0 : s.Time(now);
            s.started = now;
            s.playing = command->operation == "play";
            boost::recursive_mutex::scoped_lock physics(*s.world->Physics()->GetPhysicsUpdateMutex());
            s.Sample(now);
            s.change_serial.fetch_add(1, std::memory_order_release);
            return Encode(s.Status(now));
        }
    }
    // Never hold our RT mutex while the world owner waits for native factory ack.
    auto models = s.apply(command->models, deadline);
    std::lock_guard<std::mutex> lock(s.mutex);
    if (models.size() != command->models.size())
        throw std::runtime_error("world owner returned incomplete scene membership");
    for (const auto& model : command->models)
        if (!models.count(model.first) || !models.at(model.first))
            throw std::runtime_error("world owner omitted scene model");
    if (s.epoch != command->epoch) {
        if (!s.epoch.empty())
            s.retired_epochs.insert(s.epoch);
        s.elapsed = 0;
        s.started = s.clock() * 1e-9;
        s.playing = false;
    }
    s.epoch = command->epoch;
    s.revision = command->revision;
    s.identity = command->identity;
    s.models = std::move(models);
    s.trajectories = command->trajectories;
    s.suspended = false;
    s.definition = command->definition;
    {
        boost::recursive_mutex::scoped_lock physics(*s.world->Physics()->GetPhysicsUpdateMutex());
        s.Sample(s.clock() * 1e-9);
    }
    s.change_serial.fetch_add(1, std::memory_order_release);
    return Encode(s.Status(s.clock() * 1e-9));
}

void NativeSceneController::Update(double now) {
    auto& s = *impl_;
    std::unique_lock<std::mutex> lock(s.mutex, std::try_to_lock);
    if (!lock.owns_lock() || s.suspended)
        return;
    s.Sample(now);
}

std::string NativeSceneController::Status() const {
    const auto& s = *impl_;
    std::lock_guard<std::mutex> lock(s.mutex);
    return Encode(s.Status(s.clock() * 1e-9));
}
std::string NativeSceneController::Snapshot() const {
    const auto& s = *impl_;
    std::lock_guard<std::mutex> lock(s.mutex);
    boost::recursive_mutex::scoped_lock physics(*s.world->Physics()->GetPhysicsUpdateMutex());
    const auto nanoseconds = s.clock();
    JsonValue value = s.Status(nanoseconds * 1e-9);
    if (!std::isfinite(value["scene_time"].asDouble()))
        throw std::runtime_error("nonfinite native scene clock");
    value["frame"] = "world";
    value["stamp_ns"] = std::to_string(nanoseconds);
    value["serial"] = Json::UInt64(s.change_serial.load(std::memory_order_acquire));
    value["definition"] = s.definition;
    value["state"]["obstacles"] = JsonValue(Json::arrayValue);
    if (!s.suspended) {
        for (const auto& frozen : s.definition["obstacles"]) {
            const auto model = s.models.find(frozen["id"].asString());
            if (model == s.models.end() || !model->second)
                throw std::runtime_error("native scene roster is incomplete");
            JsonValue state;
            state["id"] = frozen["id"];
            state["pose"] = NativePose(model->second->WorldPose());
            state["twist"]["linear"] = NativeVector(model->second->WorldLinearVel());
            state["twist"]["angular"] = NativeVector(model->second->WorldAngularVel());
            value["state"]["obstacles"].append(std::move(state));
        }
    }
    return Encode(value);
}
std::uint64_t NativeSceneController::serial() const noexcept {
    return impl_->change_serial.load(std::memory_order_acquire);
}
void NativeSceneController::Reset() {
    auto& s = *impl_;
    std::lock_guard<std::mutex> lock(s.mutex);
    s.elapsed = 0;
    s.started = s.clock() * 1e-9;
    s.playing = false;
    if (!s.suspended) {
        boost::recursive_mutex::scoped_lock physics(*s.world->Physics()->GetPhysicsUpdateMutex());
        s.Sample(s.started);
    }
    s.change_serial.fetch_add(1, std::memory_order_release);
}
} // namespace xgc2_gazebo_scene
