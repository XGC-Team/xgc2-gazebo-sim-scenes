#include "xgc2_gazebo_scene/native_scene_controller.hpp"
#include <atomic>
#include <cmath>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <json/json.h>
#include <limits>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <xgc2_math/geometry/geometry_json.hpp>
#include <xgc2_math/scene/placement.hpp>

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
xgc2_math::Pose3 MathPose(const ignition::math::Pose3d& pose) {
    xgc2_math::Pose3 result;
    result.position = {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()};
    result.orientation = Eigen::Quaterniond(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());
    return result;
}
ignition::math::Pose3d IgnitionPose(const xgc2_math::Pose3& pose) {
    return {{pose.position.x(), pose.position.y(), pose.position.z()},
            {pose.orientation.w(), pose.orientation.x(), pose.orientation.y(), pose.orientation.z()}};
}
ignition::math::Vector3d IgnitionVector(const Eigen::Vector3d& value) {
    return {value.x(), value.y(), value.z()};
}
struct PlacedMotion {
    xgc2_math::Pose3 initial;
    xgc2_math::scene::Motion motion;
};
PlacedMotion ParseMotion(const JsonValue& value, const ignition::math::Pose3d& initial) {
    if (!value.isObject() || !value["type"].isString())
        throw std::invalid_argument("scene motion type required");
    PlacedMotion placed;
    placed.initial = MathPose(initial);
    const auto kind = value["type"].asString();
    if (kind == "waypoints")
        throw std::invalid_argument("unsupported scene motion");
    if (kind == "hold") {
        Fields(value, {"type"});
        placed.motion.type = xgc2_math::scene::Motion::kHold;
    } else if (kind == "constant_twist") {
        Fields(value, {"type", "linear", "angular"});
        placed.motion.type = xgc2_math::scene::Motion::kConstantTwist;
        const auto linear = Vector(value["linear"]);
        const auto angular = Vector(value["angular"]);
        placed.motion.linear = {linear.X(), linear.Y(), linear.Z()};
        placed.motion.angular = {angular.X(), angular.Y(), angular.Z()};
    } else if (kind == "ping_pong") {
        Fields(value, {"type", "point_a", "point_b", "speed"});
        placed.motion.type = xgc2_math::scene::Motion::kPingPong;
        const auto a = Vector(value["point_a"]);
        const auto b = Vector(value["point_b"]);
        placed.motion.point_a = {a.X(), a.Y(), a.Z()};
        placed.motion.point_b = {b.X(), b.Y(), b.Z()};
        placed.motion.speed = Number(value["speed"], 1e-6, 1000);
    } else if (kind == "circle") {
        Fields(value, {"type", "center", "radius", "angular_speed", "phase"});
        placed.motion.type = xgc2_math::scene::Motion::kCircle;
        const auto center = Vector(value["center"]);
        placed.motion.center = {center.X(), center.Y(), center.Z()};
        placed.motion.radius = Number(value["radius"], 1e-6, 1e4);
        placed.motion.angular_speed = Number(value["angular_speed"], -1000, 1000);
        placed.motion.phase = Number(value["phase"]);
    } else
        throw std::invalid_argument("unsupported scene motion");
    if (placed.motion.type == xgc2_math::scene::Motion::kPingPong ||
        placed.motion.type == xgc2_math::scene::Motion::kCircle) {
        const auto start = xgc2_math::scene::motionStart(placed.initial.position, placed.motion);
        if ((start - placed.initial.position).norm() > 1e-6)
            throw std::invalid_argument("scene motion initial pose disagrees with path");
    }
    return placed;
}
xgc2_geometry_msgs::ScenePart CompiledPart(const JsonValue& part, std::size_t geometry_count) {
    if (part.isMember("geometry"))
        throw std::invalid_argument("compiled part has no inline geometry");
    Fields(part, {"id", "pose", "geometryIndex", "color"});
    xgc2_geometry_msgs::ScenePart message;
    message.id = Id(part["id"]);
    message.pose = Pose(part["pose"]);
    if (geometry_count == 0 || geometry_count > std::numeric_limits<std::uint32_t>::max())
        throw std::invalid_argument("compiled geometry index out of range");
    message.geometry_index = static_cast<std::uint32_t>(Integer(part["geometryIndex"], 0, geometry_count - 1));
    const auto& color = part["color"];
    if (!color.isArray() || color.size() != 4)
        throw std::invalid_argument("compiled part color required");
    message.color.r = static_cast<float>(Number(color[0], 0, 1));
    message.color.g = static_cast<float>(Number(color[1], 0, 1));
    message.color.b = static_cast<float>(Number(color[2], 0, 1));
    message.color.a = static_cast<float>(Number(color[3], 0, 1));
    return message;
}
} // namespace

struct CompiledBuild {
    std::map<std::string, SceneModel> models;
    std::map<std::string, PlacedMotion> trajectories;
    std::optional<SceneModel> ground;
    JsonValue definition;
};

const char* MotionName(const xgc2_math::scene::Motion& motion) {
    switch (motion.type) {
    case xgc2_math::scene::Motion::kHold:
        return "hold";
    case xgc2_math::scene::Motion::kConstantTwist:
        return "constant_twist";
    case xgc2_math::scene::Motion::kPingPong:
        return "ping_pong";
    case xgc2_math::scene::Motion::kCircle:
        return "circle";
    }
    return "";
}

CompiledBuild ConsumeCompiled(const JsonValue& doc, const std::string& mesh_root, const std::string& asset_root,
                              const std::string& epoch, std::uint64_t revision) {
    Fields(doc, {"schema", "id", "frame", "geometries", "models", "obstacles", "ground"});
    if (doc["schema"] != "xgc2.scene.compiled.v1" || doc["frame"] != "world")
        throw std::invalid_argument("compiled scene in world frame required");
    xgc2_geometry_msgs::SceneSnapshot snapshot;
    snapshot.scene_id = Id(doc["id"]);
    snapshot.epoch = epoch;
    snapshot.revision = revision;
    snapshot.header.frame_id = "world";
    const auto& geometries = doc["geometries"];
    if (!geometries.isArray() || geometries.size() > 4096)
        throw std::invalid_argument("compiled geometry bound exceeded");
    for (const auto& geometry : geometries) {
        if (!geometry.isObject() || !geometry["type"].isString())
            throw std::invalid_argument("compiled geometry type required");
        const auto payload = xgc2_math::geometry::geometryFromJson(
            nlohmann::json::parse(Json::writeString(Json::StreamWriterBuilder{}, geometry)));
        xgc2_geometry_msgs::SceneGeometry message;
        message.type = payload.type;
        message.parameters = payload.parameters;
        snapshot.geometries.push_back(std::move(message));
    }
    const auto& models = doc["models"];
    if (!models.isObject())
        throw std::invalid_argument("compiled models required");
    struct ModelParts {
        std::vector<xgc2_geometry_msgs::ScenePart> parts;
        std::string resource_file;
    };
    std::map<std::string, ModelParts> table;
    for (const auto& name : models.getMemberNames()) {
        const auto& model = models[name];
        if (!model.isObject() || name.empty() || name.size() > 128)
            throw std::invalid_argument("compiled model name required");
        bool has_parts = false, has_resource = false;
        for (const auto& field : model.getMemberNames()) {
            if (field == "parts")
                has_parts = true;
            else if (field == "resource")
                has_resource = true;
            else if (field != "unparsed")
                throw std::invalid_argument("unknown compiled model field: " + field);
        }
        if (!has_parts)
            throw std::invalid_argument("compiled model parts required");
        if (model.isMember("unparsed") && !model["unparsed"].isArray())
            throw std::invalid_argument("compiled unparsed annotations must be an array");
        ModelParts parsed;
        const auto& list = model["parts"];
        if (!list.isArray() || list.empty() || list.size() > 128)
            throw std::invalid_argument("scene part bound exceeded");
        for (const auto& part : list)
            parsed.parts.push_back(CompiledPart(part, snapshot.geometries.size()));
        if (has_resource) {
            const auto& resource = model["resource"];
            Fields(resource, {"format", "file"});
            if (resource["format"] != "sdformat-model" || !resource["file"].isString())
                throw std::invalid_argument("compiled resource must be an sdformat model file");
            parsed.resource_file = resource["file"].asString();
        }
        table.emplace(name, std::move(parsed));
    }
    const auto& ground = doc["ground"];
    if (!ground.isObject() || !ground["mode"].isString())
        throw std::invalid_argument("compiled ground mode required");
    const auto mode = ground["mode"].asString();
    snapshot.ground.mode = mode;
    snapshot.ground.pose.orientation.w = 1;
    if (mode == xgc2_geometry_msgs::SceneGround::MODE_DEFAULT_PLANE) {
        Fields(ground, {"mode", "z"});
        snapshot.ground.z = Number(ground["z"]);
    } else if (mode == xgc2_geometry_msgs::SceneGround::MODE_NONE) {
        Fields(ground, {"mode"});
    } else if (mode == xgc2_geometry_msgs::SceneGround::MODE_MODEL) {
        Fields(ground, {"mode", "model", "pose"});
        if (!ground["model"].isString() || !table.count(ground["model"].asString()))
            throw std::invalid_argument("compiled ground requires a known model");
        snapshot.ground.pose = Pose(ground["pose"]);
        snapshot.ground.parts = table.at(ground["model"].asString()).parts;
    } else
        throw std::invalid_argument("unsupported ground mode");
    const auto& obstacles = doc["obstacles"];
    if (!obstacles.isArray() || obstacles.size() > 256)
        throw std::invalid_argument("scene obstacle bound exceeded");
    CompiledBuild built;
    built.definition["obstacles"] = JsonValue(Json::arrayValue);
    built.definition["ground"]["mode"] = mode;
    if (mode == xgc2_geometry_msgs::SceneGround::MODE_DEFAULT_PLANE)
        built.definition["ground"]["z"] = snapshot.ground.z;
    else if (mode == xgc2_geometry_msgs::SceneGround::MODE_MODEL) {
        built.definition["ground"]["model"] = ground["model"];
        built.definition["ground"]["pose"] = ground["pose"];
    }
    std::size_t parts = 0;
    std::map<std::string, std::string> resources;
    for (const auto& item : obstacles) {
        if (item.isMember("parts"))
            throw std::invalid_argument("compiled obstacle has no inline parts");
        Fields(item, {"id", "name", "model", "pose", "motion"});
        xgc2_geometry_msgs::SceneObstacle body;
        body.id = Id(item["id"]);
        if (!item["name"].isString() || item["name"].asString().empty() || item["name"].asString().size() > 256)
            throw std::invalid_argument("scene obstacle name required");
        body.name = item["name"].asString();
        if (!item["model"].isString() || !table.count(item["model"].asString()))
            throw std::invalid_argument("compiled obstacle requires a known model");
        body.pose = Pose(item["pose"]);
        const auto placed = ParseMotion(item["motion"], ScenePose(body.pose));
        body.motion_type = MotionName(placed.motion);
        body.dynamic = placed.motion.type != xgc2_math::scene::Motion::kHold;
        if (placed.motion.type != xgc2_math::scene::Motion::kHold &&
            !built.trajectories.emplace(body.id, placed).second)
            throw std::invalid_argument("duplicate scene obstacle id");
        const auto& parsed = table.at(item["model"].asString());
        if ((parts += parsed.parts.size()) > 4096)
            throw std::invalid_argument("scene part bound exceeded");
        body.parts = parsed.parts;
        if (!parsed.resource_file.empty())
            resources.emplace(body.id, parsed.resource_file);
        JsonValue frozen;
        frozen["id"] = body.id;
        frozen["name"] = body.name;
        frozen["model"] = item["model"];
        frozen["dynamic"] = body.dynamic;
        frozen["motion_type"] = body.motion_type;
        frozen["pose"] = item["pose"];
        built.definition["obstacles"].append(frozen);
        snapshot.obstacles.push_back(std::move(body));
    }
    std::string error;
    if (!CompileScene(snapshot, mesh_root, &built.models, &error))
        throw std::invalid_argument(error);
    for (const auto& [id, file] : resources) {
        auto& model = built.models.at(id);
        const auto collisions = std::move(model.collisions);
        const auto pose = model.pose;
        if (!AuthorResourceModel(id, pose, file, asset_root, &model, &error))
            throw std::invalid_argument(error);
        model.collisions = collisions;
    }
    if (mode == xgc2_geometry_msgs::SceneGround::MODE_DEFAULT_PLANE) {
        SceneModel plane;
        if (!MakeInstalledGroundPlane(snapshot.ground.z, &plane, &error))
            throw std::invalid_argument(error);
        built.ground = std::move(plane);
    } else if (mode == xgc2_geometry_msgs::SceneGround::MODE_MODEL) {
        const auto& parsed = table.at(ground["model"].asString());
        SceneModel ground_model;
        if (!parsed.resource_file.empty()) {
            if (!AuthorResourceModel(kConsumerGroundName, ScenePose(snapshot.ground.pose), parsed.resource_file,
                                     asset_root, &ground_model, &error) ||
                !UseConsumerGroundName(&ground_model, &error))
                throw std::invalid_argument(error);
        } else {
            xgc2_geometry_msgs::SceneSnapshot local;
            local.scene_id = snapshot.scene_id;
            local.epoch = snapshot.epoch;
            local.header.frame_id = "world";
            local.geometries = snapshot.geometries;
            local.ground.mode = xgc2_geometry_msgs::SceneGround::MODE_NONE;
            xgc2_geometry_msgs::SceneObstacle standin;
            standin.id = kConsumerGroundName;
            standin.pose = snapshot.ground.pose;
            standin.parts = snapshot.ground.parts;
            local.obstacles.push_back(standin);
            std::map<std::string, SceneModel> compiled;
            if (!CompileScene(local, mesh_root, &compiled, &error))
                throw std::invalid_argument(error);
            ground_model = std::move(compiled.at(kConsumerGroundName));
            if (!UseConsumerGroundName(&ground_model, &error))
                throw std::invalid_argument(error);
        }
        built.ground = std::move(ground_model);
    }
    return built;
}

struct PreparedSceneCommand {
    bool apply = false;
    std::string epoch, operation, identity;
    std::uint64_t revision = 0;
    std::chrono::milliseconds timeout{30000};
    std::map<std::string, SceneModel> models;
    std::map<std::string, PlacedMotion> trajectories;
    std::optional<SceneModel> ground;
    JsonValue definition;
};

class NativeSceneController::Impl {
  public:
    gazebo::physics::WorldPtr world;
    std::string mesh_root;
    std::string asset_root;
    NativeSceneApply apply;
    std::function<std::int64_t()> clock;
    mutable std::mutex mutex;
    std::string epoch, identity;
    std::uint64_t revision = 0;
    std::atomic<std::uint64_t> change_serial{0};
    bool playing = false, suspended = false;
    double elapsed = 0, started = 0;
    std::map<std::string, gazebo::physics::ModelPtr> models;
    std::map<std::string, PlacedMotion> trajectories;
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
            const auto state =
                xgc2_math::scene::evaluateMotion(entry.second.initial, entry.second.motion, time, playing);
            found->second->SetWorldPose(IgnitionPose(state.pose));
            found->second->SetLinearVel(IgnitionVector(state.linear));
            found->second->SetAngularVel(IgnitionVector(state.angular));
        }
        if (playing && sampled_time != time)
            change_serial.fetch_add(1, std::memory_order_release);
        sampled_time = time;
    }
};

NativeSceneController::NativeSceneController(gazebo::physics::WorldPtr world, std::string root, std::string asset_root,
                                             NativeSceneApply apply, std::function<std::int64_t()> clock)
    : impl_(std::make_unique<Impl>()) {
    if (!world || root.empty() || asset_root.empty() || !apply || !clock)
        throw std::invalid_argument("native scene world, mesh root, asset root, clock and apply owner required");
    impl_->world = std::move(world);
    impl_->mesh_root = std::move(root);
    impl_->asset_root = std::move(asset_root);
    impl_->apply = std::move(apply);
    impl_->clock = std::move(clock);
    impl_->definition["obstacles"] = JsonValue(Json::arrayValue);
}
NativeSceneController::~NativeSceneController() = default;

std::shared_ptr<const PreparedSceneCommand> NativeSceneController::Prepare(const std::string& route,
                                                                           const std::string& text) {
    if (text.size() > kMaxRequestBytes)
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
    auto built = ConsumeCompiled(doc, impl_->mesh_root, impl_->asset_root, command->epoch, command->revision);
    command->models = std::move(built.models);
    command->trajectories = std::move(built.trajectories);
    command->ground = std::move(built.ground);
    command->definition = std::move(built.definition);
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
    auto models = s.apply(command->models, command->ground ? &*command->ground : nullptr, deadline);
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
            value["state"]["obstacles"].append(state);
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

std::map<std::string, SceneModel> BuildCompiledSceneModels(const std::string& compiled_document,
                                                           const std::string& mesh_directory,
                                                           const std::string& asset_root) {
    if (compiled_document.size() > NativeSceneController::kMaxRequestBytes)
        throw std::invalid_argument("scene request exceeds 8 MiB");
    Json::CharReaderBuilder builder;
    builder["rejectDupKeys"] = true;
    builder["failIfExtra"] = true;
    builder["allowComments"] = false;
    builder["strictRoot"] = true;
    builder["stackLimit"] = 64;
    Json::Value document;
    std::string error;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(compiled_document.data(), compiled_document.data() + compiled_document.size(), &document,
                       &error))
        throw std::invalid_argument("invalid scene JSON");
    return ConsumeCompiled(document, mesh_directory, asset_root, "compiled", 1).models;
}

CompiledConsumerSlice InspectCompiledConsumer(const std::string& compiled_document, const std::string& mesh_directory,
                                              const std::string& asset_root) {
    if (compiled_document.size() > NativeSceneController::kMaxRequestBytes)
        throw std::invalid_argument("scene request exceeds 8 MiB");
    Json::CharReaderBuilder builder;
    builder["rejectDupKeys"] = true;
    builder["failIfExtra"] = true;
    builder["allowComments"] = false;
    builder["strictRoot"] = true;
    builder["stackLimit"] = 64;
    Json::Value document;
    std::string error;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(compiled_document.data(), compiled_document.data() + compiled_document.size(), &document,
                       &error))
        throw std::invalid_argument("invalid scene JSON");
    auto built = ConsumeCompiled(document, mesh_directory, asset_root, "compiled", 1);
    CompiledConsumerSlice slice;
    slice.obstacles = std::move(built.models);
    slice.ground = std::move(built.ground);
    slice.moving = built.trajectories.size();
    return slice;
}
} // namespace xgc2_gazebo_scene
