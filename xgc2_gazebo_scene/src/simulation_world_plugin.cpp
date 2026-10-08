#include "xgc2_gazebo_scene/simulation_service.hpp"
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>
#include <memory>
#include <stdexcept>

namespace xgc2_gazebo_scene {
class SimulationWorldPlugin final : public gazebo::WorldPlugin {
 public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr config) override {
    // No developer HOME, local identity, or implicit provider startup.
    for (const auto* key : {"socket_path", "target_id", "resource_root"})
      if (!config->HasElement(key))
        throw std::runtime_error(std::string("simulation world requires ") + key);
    std::vector<std::string> chassis_ids;
    if (config->HasElement("chassis_robot_id"))
      for (auto id = config->GetElement("chassis_robot_id"); id; id = id->GetNextElement("chassis_robot_id"))
        chassis_ids.push_back(id->Get<std::string>());
    std::vector<std::string> required_components;
    if (config->HasElement("required_component"))
      for (auto id = config->GetElement("required_component"); id; id = id->GetNextElement("required_component"))
        required_components.push_back(id->Get<std::string>());
    service_ = std::make_unique<SimulationService>(std::move(world),
        config->Get<std::string>("socket_path"),
        config->Get<std::string>("target_id"),
        config->Get<std::string>("resource_root"), std::move(chassis_ids), std::move(required_components),
        config->HasElement("configuration_revision") ? config->Get<std::string>("configuration_revision") : "");
    service_->Start();
  }
  void Init() override { service_->MarkNativeReady(); }
 private:
  std::unique_ptr<SimulationService> service_;
};
GZ_REGISTER_WORLD_PLUGIN(SimulationWorldPlugin)
}  // namespace xgc2_gazebo_scene
