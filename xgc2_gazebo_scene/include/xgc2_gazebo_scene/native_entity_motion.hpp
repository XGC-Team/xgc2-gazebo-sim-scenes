#pragma once
#include <cstdint>
#include <functional>
#include <gazebo/physics/PhysicsTypes.hh>
#include <memory>
#include <string>

namespace xgc2_gazebo_scene {
struct PreparedEntityMotion;
// Resolve must enforce the world authority's EntityRef generation and role.
using NativeMotionResolve = std::function<gazebo::physics::ModelPtr(const std::string&, std::uint64_t)>;
class NativeEntityMotion {
  public:
    NativeEntityMotion(gazebo::physics::WorldPtr world, NativeMotionResolve resolve,
                       std::function<std::int64_t()> clock);
    ~NativeEntityMotion();
    std::shared_ptr<const PreparedEntityMotion> Prepare(const std::string& json);
    std::string Execute(const std::shared_ptr<const PreparedEntityMotion>& command, bool* effects_started = nullptr);
    void Update(double simulation_seconds);
    void Remove(const std::string& id, std::uint64_t generation);
    void Reset();
    void Reset(const std::string& id, std::uint64_t generation);

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace xgc2_gazebo_scene
