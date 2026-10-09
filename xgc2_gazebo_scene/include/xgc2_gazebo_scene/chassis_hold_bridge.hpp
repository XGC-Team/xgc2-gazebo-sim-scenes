#pragma once
#include <gazebo/physics/PhysicsTypes.hh>
#include <memory>
#include <mutex>
#include <string>

namespace xgc2_gazebo_scene {
namespace detail {
class ChassisDomain;
}
// All command acceptance and native output writes run inside this gate. The
// world management executor takes the same gate when masking and zeroing a
// batch. RT producers skip a contended gate; they never wait for management.
class ChassisBinding {
  public:
    ChassisBinding(const gazebo::physics::WorldPtr& world, const std::string& robot_id, void (*zero)(void*),
                   void* context);
    ~ChassisBinding();
    // Call only after native Load/Init has completed all required outputs.
    void Ready();
    ChassisBinding(const ChassisBinding&) = delete;
    ChassisBinding& operator=(const ChassisBinding&) = delete;
    template <class Action> bool with_command(Action&& action) {
        auto guard = TryCommand();
        if (!guard.lock.owns_lock())
            return false;
        action(guard.held);
        return true;
    }

  private:
    struct Guard {
        std::unique_lock<std::mutex> lock;
        bool held = true;
    };
    Guard TryCommand();
    std::shared_ptr<detail::ChassisDomain> domain_;
    std::size_t index_ = 0;
    void* context_ = nullptr;
};
} // namespace xgc2_gazebo_scene
