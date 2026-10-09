#pragma once
#include <functional>
#include <gazebo/physics/PhysicsTypes.hh>
#include <memory>

namespace xgc2_gazebo_scene {
namespace detail {
class WorldStartupState;
}
// Native application composition: SystemPlugin world-created may run before
// WorldPlugin Load. Queue a bounded lifetime-bound attachment, without a probe,
// timer, sidecar or provider startup. The authority calls it at native Load.
class WorldStartupBinding {
  public:
    WorldStartupBinding(gazebo::physics::WorldPtr world, std::function<void(gazebo::physics::WorldPtr)> attach);
    ~WorldStartupBinding();
    WorldStartupBinding(const WorldStartupBinding&) = delete;
    WorldStartupBinding& operator=(const WorldStartupBinding&) = delete;

  private:
    std::shared_ptr<detail::WorldStartupState> state_;
};
} // namespace xgc2_gazebo_scene
