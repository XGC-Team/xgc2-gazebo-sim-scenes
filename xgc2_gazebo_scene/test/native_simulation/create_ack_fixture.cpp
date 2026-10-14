#include "xgc2_gazebo_scene/simulation_service.hpp"
#include <boost/weak_ptr.hpp>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <map>
#include <mutex>
#include <sdf/sdf.hh>

namespace xgc2_gazebo_scene {
namespace {
std::mutex mutex;
std::map<std::string, boost::weak_ptr<gazebo::physics::Model>> held;
} // namespace
// Test-only replacement for the last-plugin ACK. Model Load always returns;
// a later independently created entity releases the first model's receipt.
class CreateAckFixture final : public gazebo::ModelPlugin {
  public:
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr config) override {
        if (config->HasElement("chassisRobotId"))
            return; // Deliberately missing required HOLD owner in the failure case.
        const auto id = config->HasElement("public_entity_id") ? config->Get<std::string>("public_entity_id") : "";
        const auto root = std::getenv("XGC2_CREATE_ACK_FIXTURE_ROOT");
        if (id == "create-held" || id == "create-stop-held") {
            {
                std::lock_guard<std::mutex> lock(mutex);
                held[id] = model;
            }
            if (!root)
                throw std::runtime_error("create ACK fixture requires its isolated marker directory");
            std::ofstream(std::filesystem::path(root) / (id + ".factory")) << model->GetName();
            return;
        }
        if (root && !id.empty())
            std::ofstream(std::filesystem::path(root) / (id + ".factory")) << model->GetName();
        if (id == "create-release") {
            gazebo::physics::ModelPtr first;
            {
                std::lock_guard<std::mutex> lock(mutex);
                first = held.at("create-held").lock();
                held.erase("create-held");
            }
            if (!first)
                throw std::runtime_error("held native model was lost before ACK release");
            AcknowledgeSimulationModel(std::move(first));
        }
        AcknowledgeSimulationModel(std::move(model));
    }
};
GZ_REGISTER_MODEL_PLUGIN(CreateAckFixture)
} // namespace xgc2_gazebo_scene
