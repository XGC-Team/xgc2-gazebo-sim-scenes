#pragma once
#include "xgc2_gazebo_scene/native_component_binding.hpp"
#include "xgc2_gazebo_scene/native_world_extension.hpp"
#include <gazebo/physics/PhysicsTypes.hh>
#include <json/json.h>
#include <memory>
#include <string>
#include <vector>

namespace xgc2_gazebo_scene {
// One fixed management executor and one XRPC IO owner per live world.
// Engine update callbacks only publish native events; they never parse JSON.
class SimulationService {
  public:
    SimulationService(gazebo::physics::WorldPtr world, std::string socket_path, std::string target_id,
                      const std::string& resource_root, std::vector<std::string> chassis_robot_ids = {},
                      const std::vector<std::string>& required_components = {},
                      std::string configuration_revision = {});
    ~SimulationService();
    SimulationService(const SimulationService&) = delete;
    SimulationService& operator=(const SimulationService&) = delete;
    void Start();
    void Stop();
    void Acknowledge(gazebo::physics::ModelPtr model);
    void MarkNativeReady() noexcept;
    std::shared_ptr<detail::NativeComponentState> AttachComponent(const std::string& id);
    void RegisterExtension(const std::string& id, std::shared_ptr<NativeWorldExtension> adapter);
    void UnregisterExtension(const std::string& id, const std::shared_ptr<NativeWorldExtension>& adapter);

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
// The last root model plugin signals native model initialization. Gazebo may
// reject another plugin without rejecting the model; required owned controllers
// and sensors therefore supply separate native readiness acknowledgements.
void AcknowledgeSimulationModel(gazebo::physics::ModelPtr model);
} // namespace xgc2_gazebo_scene
