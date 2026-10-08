#include "xgc2_gazebo_scene/simulation_service.hpp"
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
namespace xgc2_gazebo_scene {
class SimulationEntityAckPlugin final : public gazebo::ModelPlugin {
 public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr) override {
    AcknowledgeSimulationModel(std::move(model));
  }
};
GZ_REGISTER_MODEL_PLUGIN(SimulationEntityAckPlugin)
}  // namespace xgc2_gazebo_scene
