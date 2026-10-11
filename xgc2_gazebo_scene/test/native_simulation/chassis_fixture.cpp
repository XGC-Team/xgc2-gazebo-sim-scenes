#include "xgc2_gazebo_scene/chassis_hold.hpp"
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <sdf/sdf.hh>

namespace xgc2_gazebo_scene {
// A real model output owner: the HOLD zero clears its cached command and writes native velocity, so a release
// cannot restore the old command. The fixture takes no commands: it moves at 1 m/s until it is zeroed.
class ChassisFixture final : public gazebo::ModelPlugin {
  public:
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr config) override {
        model_ = std::move(model);
        hold_ = std::make_unique<ChassisHold>(model_->GetWorld(), config->Get<std::string>("robot_id"),
                                              ChassisOutput{[this] {
                                                                command_ = ignition::math::Vector3d::Zero;
                                                                model_->SetLinearVel(command_);
                                                                model_->SetAngularVel(ignition::math::Vector3d::Zero);
                                                            },
                                                            BodySpeed(model_.get())});
        update_ = gazebo::event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo&) {
            Move();
        });
        hold_->Ready();
        Move(); // a paused world has no update: start moving now
    }
    ~ChassisFixture() override {
        update_.reset();
        hold_.reset();
    }

  private:
    void Move() {
        hold_->Control([this](bool held) {
            model_->SetLinearVel(held ? ignition::math::Vector3d::Zero : command_);
        });
    }
    gazebo::physics::ModelPtr model_;
    ignition::math::Vector3d command_{1, 0, 0};
    std::unique_ptr<ChassisHold> hold_;
    gazebo::event::ConnectionPtr update_;
};
GZ_REGISTER_MODEL_PLUGIN(ChassisFixture)
} // namespace xgc2_gazebo_scene
