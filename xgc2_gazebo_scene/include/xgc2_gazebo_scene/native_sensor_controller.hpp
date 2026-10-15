#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <json/json.h>
#include <memory>
#include <sdf/sdf.hh>
#include <stdexcept>
#include <string>

namespace xgc2_gazebo_scene {
struct PreparedSensorCommand;
using NativeSensorResolve = std::function<gazebo::physics::ModelPtr(const std::string&, std::uint64_t)>;
class SensorControlError : public std::runtime_error {
  public:
    SensorControlError(int status, std::string code, const std::string& message);
    const int status;
    const std::string code;
};
// World lifetime owner. Prepare runs on IO; Execute/RemoveParent run on the
// single initialized management worker. No sensor ownership is kept across
// removal: the final SensorPlugin destructor acknowledges native release.
class NativeSensorController {
  public:
    NativeSensorController(gazebo::physics::WorldPtr world, NativeSensorResolve resolve,
                           std::string ack_plugin = "libxgc2_simulation_sensor_ack.so",
                           std::function<void()> completion_changed = {});
    ~NativeSensorController();
    std::shared_ptr<const PreparedSensorCommand> Prepare(const std::string& method, const std::string& route,
                                                         const Json::Value& body);
    Json::Value Execute(const std::shared_ptr<const PreparedSensorCommand>& command, bool* effects_started = nullptr,
                        const std::atomic<bool>* cancelled = nullptr);
    // Register every authored sensor before inserting its parent model. The
    // original sensor name and preceding plugins remain unchanged. Completion
    // stays pending until the final SensorPlugin Init after the model factory ack.
    void PrepareParentSensors(const std::string& id, std::uint64_t generation, const sdf::SDFPtr& model_artifact);
    // Nonblocking, worker-only completion probe; native ACKs wake the worker.
    bool CompleteParentSensors(const std::string& id, std::uint64_t generation, Json::Value* result);
    // Call before deleting the parent model, outside the physics update mutex.
    // Success confirms each managed sensor's native destructor callback.
    Json::Value RemoveParent(const std::string& id, std::uint64_t generation, std::chrono::steady_clock::time_point deadline,
                             bool* effects_started = nullptr, const std::atomic<bool>* cancelled = nullptr);
    // World Stop calls this before joining the management worker. Native effects
    // retain their reservation if their deadline expires. Stop wakes every native
    // wait without waiting for the worker.
    void Stop();
    void NotifyCancellation();

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
// Ack plugin links the same shared scene domain library as the world service.
void AcknowledgeSimulationSensor(const std::string& token, const gazebo::sensors::SensorPtr& sensor);
void ReleaseSimulationSensor(const std::string& token);
} // namespace xgc2_gazebo_scene
