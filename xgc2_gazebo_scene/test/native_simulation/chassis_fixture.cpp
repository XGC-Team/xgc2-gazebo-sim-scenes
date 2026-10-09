#include "xgc2_gazebo_scene/chassis_hold_bridge.hpp"
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <sdf/sdf.hh>

namespace xgc2_gazebo_scene {
// A real model output owner: the HOLD callback clears its cached command and
// writes native velocity. Release therefore cannot restore the old command.
class ChassisFixture final : public gazebo::ModelPlugin {
  public:
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr config) override {
        model_ = std::move(model);
        binding_ = std::make_unique<ChassisBinding>(
            model_->GetWorld(), config->Get<std::string>("robot_id"),
            [](void* context) {
                auto& self = *static_cast<ChassisFixture*>(context);
                self.command_ = ignition::math::Vector3d::Zero;
                self.model_->SetLinearVel(self.command_);
                self.model_->SetAngularVel(ignition::math::Vector3d::Zero);
            },
            this);
        binding_->Ready();
        binding_->with_command([this](bool held) {
            model_->SetLinearVel(held ? ignition::math::Vector3d::Zero : command_);
        });
        update_ = gazebo::event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo&) {
            binding_->with_command([this](bool held) {
                model_->SetLinearVel(held ? ignition::math::Vector3d::Zero : command_);
            });
        });
    }
    ~ChassisFixture() override {
        update_.reset();
        binding_.reset();
    }

  private:
    gazebo::physics::ModelPtr model_;
    ignition::math::Vector3d command_{1, 0, 0};
    std::unique_ptr<ChassisBinding> binding_;
    gazebo::event::ConnectionPtr update_;
};
GZ_REGISTER_MODEL_PLUGIN(ChassisFixture)
} // namespace xgc2_gazebo_scene
