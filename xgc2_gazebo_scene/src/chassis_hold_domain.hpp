#pragma once
#include "xgc2_gazebo_scene/chassis_hold_bridge.hpp"
#include <xgc2/chassis_hold/provider.hpp>
#include <array>
#include <vector>

namespace xgc2_gazebo_scene::detail {
class ChassisDomain {
 public:
  explicit ChassisDomain(xgc2::chassis_hold::Options options)
      : ids(options.robot_ids), provider(std::make_unique<xgc2::chassis_hold::Provider>(std::move(options))) {}
  void Tick() noexcept {
    std::lock_guard<std::mutex> lock(mutex);
    if (alive) provider->control_tick(&Zero, this, xgc2::chassis_hold::queue_capacity);
  }
  void Quiesce() noexcept {
    std::lock_guard<std::mutex> lock(mutex);
    alive = false;
    for (std::size_t n = 0; n < ids.size(); ++n) if (bindings[n].zero && bindings[n].ready) {
      try { bindings[n].zero(bindings[n].context); } catch (...) {}
    }
  }
  static bool Zero(void* context, std::size_t index) noexcept {
    auto& self = *static_cast<ChassisDomain*>(context);
    if (index >= self.ids.size() || !self.bindings[index].zero || !self.bindings[index].ready) return false;
    try { self.bindings[index].zero(self.bindings[index].context); return true; }
    catch (...) { return false; }
  }
  bool Ready(const std::string& id) {
    std::lock_guard<std::mutex> lock(mutex);
    for (std::size_t n = 0; n < ids.size(); ++n) if (ids[n] == id)
      return alive && bindings[n].zero && bindings[n].ready;
    return true; // A non-chassis entity has no chassis initialization hook.
  }
  bool ResetOutput(const std::string& id, bool retire) {
    std::lock_guard<std::mutex> lock(mutex);
    for (std::size_t n = 0; n < ids.size(); ++n) if (ids[n] == id) {
      const bool zeroed = Zero(this, n);
      if (retire) bindings[n].ready = false;
      return zeroed;
    }
    return true;
  }
  struct Binding { void (*zero)(void*) = nullptr; void* context = nullptr; bool ready = false; };
  std::mutex mutex;
  bool alive = true;
  const std::vector<std::string> ids;
  std::array<Binding, xgc2::chassis_hold::max_robots> bindings{};
  std::unique_ptr<xgc2::chassis_hold::Provider> provider;
};
void PublishChassisDomain(gazebo::physics::WorldPtr world, const std::shared_ptr<ChassisDomain>& domain);
void RetireChassisDomain(gazebo::physics::WorldPtr world, const std::shared_ptr<ChassisDomain>& domain);
}  // namespace xgc2_gazebo_scene::detail
