#pragma once
#include <gazebo/physics/PhysicsTypes.hh>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace xgc2_gazebo_scene {
namespace detail {
struct NativeComponentState {
  std::atomic<int> state{0}; // missing, starting, ready, failed
  std::atomic<std::uint64_t> revision{0};
  std::atomic<bool> attached{false};
};
std::shared_ptr<NativeComponentState> AttachNativeComponent(gazebo::physics::WorldPtr world, const std::string& id);
}
// Component owners report their own actual native initialization/failure.
// No ROS/TCP probes or publisher are introduced by this lifecycle binding.
class NativeComponentBinding {
 public:
  NativeComponentBinding(gazebo::physics::WorldPtr world, std::string id)
      : state_(detail::AttachNativeComponent(std::move(world), id)) {}
  ~NativeComponentBinding() { Failed(); state_->attached.store(false); }
  NativeComponentBinding(const NativeComponentBinding&) = delete;
  NativeComponentBinding& operator=(const NativeComponentBinding&) = delete;
  void Ready() noexcept { Set(2); }
  void Failed() noexcept { Set(3); }
 private:
  void Set(int state) noexcept {
    if (state_->state.exchange(state) != state) state_->revision.fetch_add(1);
  }
  std::shared_ptr<detail::NativeComponentState> state_;
};
}
