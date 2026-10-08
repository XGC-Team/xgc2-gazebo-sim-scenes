#include "xgc2_gazebo_scene/native_entity_motion.hpp"
#include "xgc2_gazebo_scene/motion_controller.hpp"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <json/json.h>
#include <cmath>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <vector>

namespace xgc2_gazebo_scene {
namespace {
void Fields(const Json::Value& value, std::initializer_list<const char*> allowed) {
  if (!value.isObject()) throw std::invalid_argument("entity motion object required");
  for (const auto& name : value.getMemberNames()) {
    bool found = false; for (const auto* key : allowed) found = found || key == name;
    if (!found) throw std::invalid_argument("unknown entity motion field: " + name);
  }
}
double Number(const Json::Value& value) {
  if (!value.isNumeric() || !std::isfinite(value.asDouble()) || std::abs(value.asDouble()) > 1e6)
    throw std::invalid_argument("finite entity motion number required");
  return value.asDouble();
}
ignition::math::Vector3d Vector(const Json::Value& value) {
  if (!value.isArray() || value.size() != 3) throw std::invalid_argument("entity motion vector requires three components");
  return {Number(value[0]), Number(value[1]), Number(value[2])};
}
}
struct PreparedEntityMotion {
  struct Item { std::string id; std::uint64_t generation; MotionConfiguration configuration; };
  std::vector<Item> items;
};
class NativeEntityMotion::Impl {
 public:
  struct Entry { std::uint64_t generation; gazebo::physics::ModelPtr model; MotionController controller; };
  gazebo::physics::WorldPtr world;
  NativeMotionResolve resolve;
  std::function<std::int64_t()> clock;
  std::mutex mutex;
  std::map<std::string, Entry> entries;
};
NativeEntityMotion::NativeEntityMotion(gazebo::physics::WorldPtr world, NativeMotionResolve resolve,
                                      std::function<std::int64_t()> clock)
  : impl_(std::make_unique<Impl>()) {
  if (!world || !resolve || !clock) throw std::invalid_argument("entity motion world, resolver and clock required");
  impl_->world = std::move(world); impl_->resolve = std::move(resolve);
  impl_->clock = std::move(clock);
}
NativeEntityMotion::~NativeEntityMotion() = default;
std::shared_ptr<const PreparedEntityMotion> NativeEntityMotion::Prepare(const std::string& text) {
  if (text.size() > 1024*1024) throw std::invalid_argument("entity motion body exceeds limit");
  Json::CharReaderBuilder builder; builder["rejectDupKeys"] = true; builder["failIfExtra"] = true;
  builder["allowComments"] = false; builder["stackLimit"] = 32;
  Json::Value value; std::string error; std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  if (!reader->parse(text.data(), text.data()+text.size(), &value, &error)) throw std::invalid_argument("invalid entity motion JSON");
  Fields(value, {"entities", "operation_timeout_ms"});
  const auto& timeout = value["operation_timeout_ms"];
  if ((timeout.type() != Json::intValue && timeout.type() != Json::uintValue) || !timeout.isUInt64()
      || !timeout.asUInt64() || timeout.asUInt64() > 60000) throw std::invalid_argument("integer entity motion budget required");
  const auto& items = value["entities"];
  if (!items.isArray() || items.empty() || items.size() > 256) throw std::invalid_argument("entity motion requires 1 to 256 entities");
  auto command = std::make_shared<PreparedEntityMotion>(); std::set<std::string> ids;
  for (const auto& item : items) {
    Fields(item, {"ref", "motion"}); const auto& ref = item["ref"]; Fields(ref, {"id", "generation"});
    if (!ref["id"].isString() || ref["id"].asString().empty() || ref["id"].asString().size() > 128
        || (ref["generation"].type() != Json::intValue && ref["generation"].type() != Json::uintValue)
        || !ref["generation"].isUInt64() || !ref["generation"].asUInt64()
        || ref["generation"].asUInt64() > 9007199254740991ULL) throw std::invalid_argument("EntityRef required");
    PreparedEntityMotion::Item out; out.id = ref["id"].asString(); out.generation = ref["generation"].asUInt64();
    if (!ids.insert(out.id).second) throw std::invalid_argument("duplicate entity motion target");
    const auto& motion = item["motion"];
    Fields(motion, {"type", "linear", "angular", "point_a", "point_b", "center", "angular_speed", "speed"});
    if (!motion["type"].isString() || !ParseMotionMode(motion["type"].asString(), &out.configuration.mode))
      throw std::invalid_argument("unsupported entity motion type");
    switch (out.configuration.mode) {
      case MotionMode::kHold: Fields(motion, {"type"}); break;
      case MotionMode::kConstantTwist:
        Fields(motion, {"type", "linear", "angular"});
        out.configuration.linear_velocity = Vector(motion["linear"]); out.configuration.angular_velocity = Vector(motion["angular"]); break;
      case MotionMode::kPingPong:
        Fields(motion, {"type", "point_a", "point_b", "speed"});
        out.configuration.waypoints = {Vector(motion["point_a"]), Vector(motion["point_b"])};
        out.configuration.speed = Number(motion["speed"]); break;
      case MotionMode::kCircle:
        Fields(motion, {"type", "center", "angular_speed"});
        out.configuration.waypoints = {Vector(motion["center"])}; out.configuration.speed = Number(motion["angular_speed"]); break;
    }
    command->items.push_back(std::move(out));
  }
  return command;
}
std::string NativeEntityMotion::Execute(const std::shared_ptr<const PreparedEntityMotion>& command, bool* effects_started) {
  if (effects_started) *effects_started = false;
  if (!command) throw std::invalid_argument("prepared entity motion required");
  auto& s = *impl_; std::map<std::string, Impl::Entry> prepared;
  const double now = s.clock()*1e-9;
  // Resolve and validate the entire batch before changing any controller.
  for (const auto& item : command->items) {
    auto model = s.resolve(item.id, item.generation);
    if (!model) throw std::invalid_argument("entity motion reference does not exist");
    MotionController controller; std::string error;
    if (!controller.Configure(item.configuration, model->WorldPose(), now, &error)) throw std::invalid_argument(error);
    prepared.emplace(item.id, Impl::Entry{item.generation, std::move(model), std::move(controller)});
  }
  std::lock_guard<std::mutex> lock(s.mutex);
  std::size_t additions = 0;
  for (const auto& item : prepared) additions += !s.entries.count(item.first);
  if (s.entries.size()+additions > 256) throw std::invalid_argument("entity motion capacity exceeded");
  boost::recursive_mutex::scoped_lock physics(*s.world->Physics()->GetPhysicsUpdateMutex());
  Json::Value result; result["entities"] = Json::Value(Json::arrayValue);
  if (effects_started) *effects_started = true;
  for (auto& item : prepared) {
    const auto sample = item.second.controller.Sample(now);
    item.second.model->SetWorldPose(sample.pose); item.second.model->SetLinearVel(sample.linear_velocity);
    item.second.model->SetAngularVel(sample.angular_velocity);
    Json::Value ref; ref["id"] = item.first; ref["generation"] = Json::UInt64(item.second.generation);
    result["entities"].append(ref); s.entries.insert_or_assign(item.first, std::move(item.second));
  }
  Json::StreamWriterBuilder writer; writer["indentation"] = ""; return Json::writeString(writer, result);
}
void NativeEntityMotion::Update(double now) {
  auto& s = *impl_; std::unique_lock<std::mutex> lock(s.mutex, std::try_to_lock);
  if (!lock.owns_lock()) return;
  for (const auto& item : s.entries) {
    const auto sample = item.second.controller.Sample(now);
    item.second.model->SetWorldPose(sample.pose); item.second.model->SetLinearVel(sample.linear_velocity);
    item.second.model->SetAngularVel(sample.angular_velocity);
  }
}
void NativeEntityMotion::Remove(const std::string& id, std::uint64_t generation) {
  auto& s = *impl_; std::lock_guard<std::mutex> lock(s.mutex);
  const auto found = s.entries.find(id); if (found != s.entries.end() && found->second.generation == generation) s.entries.erase(found);
}
void NativeEntityMotion::Reset() {
  auto& s = *impl_; std::lock_guard<std::mutex> lock(s.mutex); s.entries.clear();
}
void NativeEntityMotion::Reset(const std::string& id, std::uint64_t generation) {
  // A reset retires the controller of this generation. The world authority
  // restores its declared initial/explicit state; a subsequent physics tick
  // cannot overwrite it using an old trajectory or another entity's clock.
  Remove(id, generation);
}
}
