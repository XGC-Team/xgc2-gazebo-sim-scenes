#include "xgc2_gazebo_scene/native_sensor_controller.hpp"
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorFactory.hh>
#include <sdf/sdf.hh>
#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <regex>
#include <unordered_map>
#include <optional>
#include <vector>

namespace xgc2_gazebo_scene {
namespace {
using Clock = std::chrono::steady_clock;
struct AckState {
  std::mutex mutex;
  std::condition_variable changed;
  bool initialized = false, released = false;
  std::weak_ptr<gazebo::sensors::Sensor> sensor;
};
std::mutex registry_mutex;
std::unordered_map<std::string, std::weak_ptr<AckState>> registry;
std::atomic<std::uint64_t> next_token{1};
[[noreturn]] void Invalid(const std::string& message) { throw SensorControlError(400,"invalid_argument",message); }
void Fields(const Json::Value& value, std::initializer_list<const char*> allowed) {
  if (!value.isObject()) Invalid("document must be an object");
  for (const auto& name : value.getMemberNames()) {
    bool accepted=false; for (auto field:allowed) accepted |= name==field;
    if (!accepted) Invalid("unknown field: "+name);
  }
}
std::uint64_t UInt(const Json::Value& value) {
  if ((value.type()!=Json::intValue && value.type()!=Json::uintValue) || !value.isUInt64() || value.asUInt64() > 9007199254740991ULL) Invalid("integer must be unsigned and exactly representable");
  return value.asUInt64();
}
std::string Id(const Json::Value& value) {
  static const std::regex pattern("^[A-Za-z0-9._-]{1,128}$");
  if (!value.isString() || !std::regex_match(value.asString(),pattern)) Invalid("invalid bounded identifier");
  return value.asString();
}
double Number(const Json::Value& value) {
  if (!value.isNumeric() || !std::isfinite(value.asDouble())) Invalid("number must be finite");
  return value.asDouble();
}
ignition::math::Pose3d Pose(const Json::Value& value) {
  Fields(value,{"position","orientation"});
  const auto& p=value["position"]; const auto& q=value["orientation"];
  if (!p.isArray() || p.size()!=3 || !q.isArray() || q.size()!=4) Invalid("pose requires position[3] and orientation[4]");
  ignition::math::Vector3d position(Number(p[0]),Number(p[1]),Number(p[2]));
  ignition::math::Quaterniond rotation(Number(q[3]),Number(q[0]),Number(q[1]),Number(q[2]));
  const double norm=rotation.W()*rotation.W()+rotation.X()*rotation.X()+rotation.Y()*rotation.Y()+rotation.Z()*rotation.Z();
  if (std::abs(norm-1.)>1e-6) Invalid("orientation quaternion must have unit norm");
  return {position,rotation};
}
Json::Value PoseJson(const ignition::math::Pose3d& pose) {
  Json::Value result; for (auto v:{pose.Pos().X(),pose.Pos().Y(),pose.Pos().Z()}) result["position"].append(v);
  for (auto v:{pose.Rot().X(),pose.Rot().Y(),pose.Rot().Z(),pose.Rot().W()}) result["orientation"].append(v);
  return result;
}
std::shared_ptr<AckState> FindAck(const std::string& token) {
  std::lock_guard<std::mutex> lock(registry_mutex);
  auto it=registry.find(token); return it==registry.end()?nullptr:it->second.lock();
}
}
SensorControlError::SensorControlError(int status_,std::string code_,std::string message)
  : std::runtime_error(std::move(message)),status(status_),code(std::move(code_)) {}
void AcknowledgeSimulationSensor(const std::string& token,gazebo::sensors::SensorPtr sensor) {
  if (auto ack=FindAck(token)) {
    { std::lock_guard<std::mutex> lock(ack->mutex); ack->sensor=sensor; ack->initialized=true; }
    ack->changed.notify_all();
  }
}
void ReleaseSimulationSensor(const std::string& token) {
  if (auto ack=FindAck(token)) {
    { std::lock_guard<std::mutex> lock(ack->mutex); ack->sensor.reset(); ack->released=true; }
    ack->changed.notify_all();
  }
}
struct PreparedSensorCommand {
  enum class Kind { List, Describe, Create, Configure, Remove } kind;
  std::string id,parent,link,type;
  std::uint64_t generation=0,parent_generation=0,expected_revision=0;
  std::uint32_t timeout_ms=5000;
  Clock::time_point admission_deadline=Clock::now()+std::chrono::seconds(5);
  sdf::ElementPtr realization;
  std::optional<bool> active;
  std::optional<double> rate;
  std::optional<ignition::math::Pose3d> pose;
};
class NativeSensorController::Impl {
 public:
  struct Record {
    std::string id,parent,link,type,native_name,token;
    std::uint64_t generation=0,parent_generation=0,desired_revision=1,applied_revision=1;
    bool occupied=false;
    Json::Value desired,applied;
    std::shared_ptr<AckState> ack;
  };
  Impl(gazebo::physics::WorldPtr world,NativeSensorResolve resolve,std::string plugin)
      :world_(std::move(world)),resolve_(std::move(resolve)),plugin_(std::move(plugin)) {}
  ~Impl() {
    std::lock_guard<std::mutex> lock(registry_mutex);
    for (const auto& record:records_) if (record.occupied) registry.erase(record.token);
  }
  std::shared_ptr<const PreparedSensorCommand> Prepare(const std::string& method,const std::string& route,const Json::Value& body) {
    auto command=std::make_shared<PreparedSensorCommand>();
    const std::string prefix="/v1/sensors";
    if (route==prefix && method=="GET") { Fields(body,{}); command->kind=PreparedSensorCommand::Kind::List; return command; }
    if (route==prefix && method=="POST") {
      Fields(body,{"sensor","operation_timeout_ms"}); command->kind=PreparedSensorCommand::Kind::Create;
      const auto& sensor=body["sensor"]; Fields(sensor,{"id","parent","realization"}); command->id=Id(sensor["id"]);
      const auto& parent=sensor["parent"]; Fields(parent,{"id","generation","link"});
      if (!parent["id"].isString() || parent["id"].asString().empty() || parent["id"].asString().size()>128 ||
          !std::regex_match(parent["id"].asString(),std::regex("^[A-Za-z0-9._:-]+$"))) Invalid("invalid parent entity identifier");
      command->parent=parent["id"].asString(); command->parent_generation=UInt(parent["generation"]); command->link=Id(parent["link"]);
      if (!command->parent_generation) Invalid("parent generation must be positive");
      const auto& realization=sensor["realization"]; Fields(realization,{"media_type","content"});
      if (realization["media_type"]!="application/sdf+xml" || !realization["content"].isString()) Invalid("sensor requires inline application/sdf+xml realization");
      const auto document=realization["content"].asString();
      if (document.empty() || document.size()>32768 || document.find("<!")!=std::string::npos) Invalid("sensor SDF must be bounded and have no declarations");
      auto sdf_document=std::make_shared<sdf::SDF>(); sdf::init(sdf_document);
      const auto wrapped="<sdf version='1.6'><model name='realization'><link name='attachment'>"+document+"</link></model></sdf>";
      if (!sdf::readString(wrapped,sdf_document) || !sdf_document->Root()->HasElement("model")) Invalid("realization must contain one sensor XML element");
      auto link=sdf_document->Root()->GetElement("model")->GetElement("link");
      if (!link->HasElement("sensor")) Invalid("realization must contain one sensor XML element");
      auto sensor_element=link->GetElement("sensor");
      if (sensor_element->GetNextElement("sensor")) Invalid("one sensor is required");
      command->realization=sensor_element->Clone();
      command->type=command->realization->Get<std::string>("type");
      if (command->type!="camera" && command->type!="imu" && command->type!="ray") Invalid("supported sensor types are camera, imu and ray");
    } else {
      if (route.compare(0,prefix.size()+1,prefix+"/")!=0) throw SensorControlError(404,"not_found","unknown sensor route");
      auto suffix=route.substr(prefix.size()+1); const auto slash=suffix.find('/');
      command->id=Id(Json::Value(suffix.substr(0,slash)));
      if (slash==std::string::npos && method=="GET") { Fields(body,{}); command->kind=PreparedSensorCommand::Kind::Describe; }
      else if (slash==std::string::npos && method=="DELETE") {
        Fields(body,{"generation","operation_timeout_ms"}); command->kind=PreparedSensorCommand::Kind::Remove; command->generation=UInt(body["generation"]);
      } else if (slash!=std::string::npos && suffix.substr(slash)=="/config" && method=="PATCH") {
        Fields(body,{"generation","expected_revision","persist","config","operation_timeout_ms"}); command->kind=PreparedSensorCommand::Kind::Configure;
        command->generation=UInt(body["generation"]); command->expected_revision=UInt(body["expected_revision"]);
        if (body.isMember("persist") && (!body["persist"].isBool() || body["persist"].asBool())) Invalid("sensor configuration supports persist:false only");
        const auto& config=body["config"]; Fields(config,{"active","update_rate_hz","pose"}); if (config.empty()) Invalid("config must be nonempty");
        if (config.isMember("active")) { if (!config["active"].isBool()) Invalid("active must be boolean"); command->active=config["active"].asBool(); }
        if (config.isMember("update_rate_hz")) { auto rate=Number(config["update_rate_hz"]); if (rate<0 || rate>1000) Invalid("update_rate_hz must be 0 through 1000"); command->rate=rate; }
        if (config.isMember("pose")) command->pose=Pose(config["pose"]);
      } else throw SensorControlError(404,"not_found","unknown sensor route");
      if ((command->kind==PreparedSensorCommand::Kind::Configure || command->kind==PreparedSensorCommand::Kind::Remove) && !command->generation) Invalid("sensor generation must be positive");
    }
    if (body.isMember("operation_timeout_ms")) {
      auto timeout=UInt(body["operation_timeout_ms"]); if (!timeout || timeout>30000) Invalid("operation_timeout_ms must be 1 through 30000"); command->timeout_ms=timeout;
    }
    command->admission_deadline=Clock::now()+std::chrono::milliseconds(command->timeout_ms);
    return command;
  }
  Record& Find(const std::string& id,std::uint64_t generation=0) {
    for (auto& record:records_) if (record.occupied && record.id==id) {
      if (generation && record.generation!=generation) throw SensorControlError(409,"conflict","stale sensor generation");
      return record;
    }
    throw SensorControlError(404,"not_found","sensor is not registered");
  }
  void Wait(const std::shared_ptr<AckState>& ack,bool removal,Clock::time_point) {
    std::unique_lock<std::mutex> lock(ack->mutex);
    ack->changed.wait(lock,[&]{return stopping_.load() || (removal?ack->released:(ack->initialized || ack->released));});
    if (stopping_.load()) throw SensorControlError(503,"unavailable","world stopped during native sensor effect");
    if (!removal && ack->released) throw SensorControlError(503,"unavailable","sensor released during initialization");
  }
  gazebo::sensors::SensorPtr Sensor(Record& record) {
    std::lock_guard<std::mutex> lock(record.ack->mutex);
    auto sensor=record.ack->sensor.lock(); if (!sensor || record.ack->released) throw SensorControlError(503,"unavailable","native sensor is unavailable"); return sensor;
  }
  Json::Value Config(gazebo::sensors::SensorPtr sensor) {
    Json::Value result; result["active"]=sensor->IsActive(); result["update_rate_hz"]=sensor->UpdateRate(); result["pose"]=PoseJson(sensor->Pose()); return result;
  }
  Json::Value Describe(Record& record) {
    Json::Value result; result["ref"]["id"]=record.id; result["ref"]["generation"]=Json::UInt64(record.generation);
    result["parent"]["id"]=record.parent; result["parent"]["generation"]=Json::UInt64(record.parent_generation); result["parent"]["link"]=record.link;
    result["kind"]=record.type=="ray"?"lidar":record.type;
    result["native_type"]=record.type; result["native_name"]=record.native_name;
    result["desired"]["revision"]=Json::UInt64(record.desired_revision); result["desired"]["config"]=record.desired;
    result["applied"]["revision"]=Json::UInt64(record.applied_revision); result["applied"]["config"]=record.applied; result["persisted"]=Json::nullValue;
    { std::lock_guard<std::mutex> lock(record.ack->mutex); result["native_initialized"]=record.ack->initialized; result["native_released"]=record.ack->released; }
    for (auto field:{"active","update_rate_hz","pose"}) result["configuration_schema"]["live_fields"].append(field);
    result["configuration_schema"]["persist_supported"]=false;
    return result;
  }
  void AppendAck(sdf::ElementPtr realization,const std::string& token) {
    auto plugin=realization->AddElement("plugin");
    plugin->GetAttribute("name")->SetFromString("xgc_sensor_ack_"+token);
    plugin->GetAttribute("filename")->SetFromString(plugin_);
    auto value=std::make_shared<sdf::Element>();value->SetName("controller_token");value->AddValue("string",token,true);plugin->InsertElement(value);
  }
  void PrepareParentSensors(const std::string& id,std::uint64_t generation,const sdf::SDFPtr& artifact) {
    if(stopping_.load())throw SensorControlError(503,"unavailable","world sensor controller stopped");
    if(!artifact || !artifact->Root() || !artifact->Root()->HasElement("model"))Invalid("parent artifact must contain a model");
    const auto staged_root=artifact->Root()->Clone();
    struct Authored {sdf::ElementPtr sensor;std::string link;};std::vector<Authored> sensors;
    const auto walk=[&](auto&& self,sdf::ElementPtr model,const std::string& prefix)->void {
      if(model->HasElement("link"))for(auto link=model->GetElement("link");link;link=link->GetNextElement("link")) {
        const auto name=prefix+link->Get<std::string>("name");
        if(link->HasElement("sensor"))for(auto sensor=link->GetElement("sensor");sensor;sensor=sensor->GetNextElement("sensor")) {
          sensors.push_back({sensor,name});if(sensors.size()>records_.size())throw SensorControlError(429,"resource_exhausted","parent sensor capacity exceeds 32");
        }
      }
      if(model->HasElement("model"))for(auto child=model->GetElement("model");child;child=child->GetNextElement("model"))self(self,child,prefix+child->Get<std::string>("name")+"::");
    };
    walk(walk,staged_root->GetElement("model"),"");
    std::vector<std::string> registered_types;
    gazebo::sensors::SensorFactory::GetSensorTypes(registered_types);
    for (const auto& authored : sensors) {
      const auto type = authored.sensor->Get<std::string>("type");
      if (std::find(registered_types.begin(), registered_types.end(), type) == registered_types.end())
        Invalid("parent sensor type is not registered by the native SensorFactory: " + type);
    }
    std::size_t free=0;for(const auto& record:records_)if(!record.occupied)++free;
    if(sensors.size()>free)throw SensorControlError(429,"resource_exhausted","shared managed sensor capacity is 32");
    // Clone, plugin append and all allocations precede the commit. A validation
    // or allocation failure cannot alter the authored artifact or occupy slots.
    std::vector<Record> staged_records;staged_records.reserve(sensors.size());
    std::unordered_map<std::string,std::weak_ptr<AckState>> staged_registry;
    for(const auto& authored:sensors) {
      const auto sequence=next_token.fetch_add(1);const auto token="sensor-"+std::to_string(sequence);
      auto ack=std::make_shared<AckState>();
      AppendAck(authored.sensor,token);
      staged_registry.emplace(token,ack);
      staged_records.push_back(Record{"attached-"+std::to_string(sequence),id,authored.link,authored.sensor->Get<std::string>("type"),"",token,++generation_,generation,1,0,true,{}, {},ack});
    }
    {std::lock_guard<std::mutex> lock(registry_mutex);registry.reserve(registry.size()+staged_registry.size());registry.merge(staged_registry);}
    for(auto& staged:staged_records)for(auto& record:records_)if(!record.occupied){record=std::move(staged);break;}
    artifact->Root(staged_root);
  }
  Json::Value CompleteParentSensors(const std::string& id,std::uint64_t generation) {
    Json::Value result;result["sensors"]=Json::arrayValue;
    for(auto& record:records_)if(record.occupied&&record.parent==id&&record.parent_generation==generation) {
      Wait(record.ack,false,{});auto sensor=Sensor(record);record.native_name=sensor->ScopedName();
      record.desired=record.applied=Config(sensor);record.applied_revision=record.desired_revision;
      result["sensors"].append(Describe(record));
    }
    result["native_initialized"]=true;return result;
  }
  void Remove(Record& record,Clock::time_point deadline,bool* effects_started=nullptr) {
    if(record.native_name.empty()) {
      Wait(record.ack,false,deadline);auto sensor=Sensor(record);record.native_name=sensor->ScopedName();
    }
    if(effects_started) *effects_started=true;
    gazebo::sensors::SensorManager::Instance()->RemoveSensor(record.native_name);
    Wait(record.ack,true,deadline);
    { std::lock_guard<std::mutex> lock(registry_mutex); registry.erase(record.token); }
    record=Record{};
  }
  Json::Value Execute(const PreparedSensorCommand& command,bool* effects_started) {
    if (stopping_.load()) throw SensorControlError(503,"unavailable","world sensor controller stopped");
    if(Clock::now()>=command.admission_deadline) throw SensorControlError(504,"deadline_exceeded","sensor admission budget expired before effects");
    const auto deadline=Clock::now()+std::chrono::milliseconds(command.timeout_ms);
    Json::Value result; result["sensors"]=Json::arrayValue;
    if (command.kind==PreparedSensorCommand::Kind::List) {
      for (auto& record:records_) if (record.occupied) result["sensors"].append(Describe(record));
      return result;
    }
    if (command.kind==PreparedSensorCommand::Kind::Create) {
      for (auto& record:records_) if (record.occupied && record.id==command.id) throw SensorControlError(409,"conflict","sensor id already exists");
      Record* record=nullptr; for (auto& candidate:records_) if (!candidate.occupied) {record=&candidate;break;}
      if (!record) throw SensorControlError(429,"resource_exhausted","managed sensor capacity is 32");
      auto model=resolve_(command.parent,command.parent_generation);
      auto link=model?model->GetLink(command.link):gazebo::physics::LinkPtr{};
      if (!link) throw SensorControlError(404,"not_found","parent link not found");
      auto realization=command.realization->Clone(); realization->GetAttribute("name")->SetFromString(command.id);
      auto token="sensor-"+std::to_string(next_token.fetch_add(1)); auto ack=std::make_shared<AckState>();
      {std::lock_guard<std::mutex> lock(registry_mutex);registry[token]=ack;}
      AppendAck(realization,token);
      *record=Record{command.id,command.parent,command.link,command.type,"",token,++generation_,command.parent_generation,1,1,true,{}, {},ack};
      try {
        if(effects_started) *effects_started=true;
        record->native_name=gazebo::sensors::SensorManager::Instance()->CreateSensor(realization,world_->Name(),link->GetScopedName(),link->GetId());
        if (record->native_name.empty()) throw SensorControlError(503,"unavailable","native sensor creation rejected");
        link.reset(); model.reset(); Wait(ack,false,deadline);
        auto sensor=Sensor(*record); record->desired=record->applied=Config(sensor); result["sensors"].append(Describe(*record));
      } catch (...) {
        // Keep an occupied tombstone if release cannot be confirmed; a later
        // explicit DELETE can reconcile it without creating a duplicate.
        if (!record->native_name.empty()) gazebo::sensors::SensorManager::Instance()->RemoveSensor(record->native_name);
        else {std::lock_guard<std::mutex> lock(registry_mutex);registry.erase(token); *record=Record{};}
        throw;
      }
    } else {
      auto& record=Find(command.id,command.generation);
      if (command.kind==PreparedSensorCommand::Kind::Remove) {
        Json::Value ref; ref["id"]=record.id;ref["generation"]=Json::UInt64(record.generation);Remove(record,deadline,effects_started);result["removed"].append(ref);
      } else if (command.kind==PreparedSensorCommand::Kind::Configure) {
        if (record.desired_revision!=command.expected_revision) throw SensorControlError(409,"conflict","sensor configuration revision differs");
        resolve_(record.parent,record.parent_generation);
        auto sensor=Sensor(record); auto desired=record.desired;
        if(command.active) desired["active"]=*command.active;
        if(command.rate) desired["update_rate_hz"]=*command.rate;
        if(command.pose) desired["pose"]=PoseJson(*command.pose);
        record.desired=desired; ++record.desired_revision;
        {
          boost::recursive_mutex::scoped_lock lock(*world_->Physics()->GetPhysicsUpdateMutex());
          if(effects_started) *effects_started=true;
          if(command.pose) sensor->SetPose(*command.pose);
          if(command.rate) sensor->SetUpdateRate(*command.rate);
          if(command.active) sensor->SetActive(*command.active);
          record.applied=Config(sensor); record.applied_revision=record.desired_revision;
        }
        result["sensors"].append(Describe(record));
      } else result["sensors"].append(Describe(record));
    }
    result["effects"]["native_initialized"]=command.kind==PreparedSensorCommand::Kind::Create;
    result["effects"]["native_release_confirmed"]=command.kind==PreparedSensorCommand::Kind::Remove;
    result["effects"]["applied"]=command.kind!=PreparedSensorCommand::Kind::Describe;
    return result;
  }
  Json::Value RemoveParent(const std::string& id,std::uint64_t generation,std::uint32_t timeout,bool* effects_started) {
    const auto deadline=Clock::now()+std::chrono::milliseconds(timeout); Json::Value result;result["removed"]=Json::arrayValue;
    for(auto& record:records_) if(record.occupied && record.parent==id && record.parent_generation==generation) {
      Json::Value ref;ref["id"]=record.id;ref["generation"]=Json::UInt64(record.generation);Remove(record,deadline,effects_started);result["removed"].append(ref);
    }
    result["native_release_confirmed"]=true;return result;
  }
  void Stop() {
    stopping_.store(true);
    std::lock_guard<std::mutex> lock(registry_mutex);
    for (auto& item:registry) if(auto ack=item.second.lock()) {
      std::lock_guard<std::mutex> ack_lock(ack->mutex);
      ack->changed.notify_all();
    }
  }
 private:
  gazebo::physics::WorldPtr world_;
  NativeSensorResolve resolve_;
  std::string plugin_;
  std::array<Record,32> records_;
  std::uint64_t generation_=0;
  std::atomic<bool> stopping_{false};
};
NativeSensorController::NativeSensorController(gazebo::physics::WorldPtr world,NativeSensorResolve resolve,std::string plugin)
    :impl_(std::make_unique<Impl>(std::move(world),std::move(resolve),std::move(plugin))) {}
NativeSensorController::~NativeSensorController()=default;
std::shared_ptr<const PreparedSensorCommand> NativeSensorController::Prepare(const std::string& method,const std::string& route,const Json::Value& body){return impl_->Prepare(method,route,body);}
Json::Value NativeSensorController::Execute(const std::shared_ptr<const PreparedSensorCommand>& command,bool* effects_started){if(!command)Invalid("sensor command is missing");return impl_->Execute(*command,effects_started);}
Json::Value NativeSensorController::RemoveParent(const std::string& id,std::uint64_t generation,std::uint32_t timeout,bool* effects_started){return impl_->RemoveParent(id,generation,timeout,effects_started);}
void NativeSensorController::Stop(){impl_->Stop();}
void NativeSensorController::PrepareParentSensors(const std::string& id,std::uint64_t generation,const sdf::SDFPtr& artifact){impl_->PrepareParentSensors(id,generation,artifact);}
Json::Value NativeSensorController::CompleteParentSensors(const std::string& id,std::uint64_t generation){return impl_->CompleteParentSensors(id,generation);}
}
