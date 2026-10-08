#pragma once

#include "xgc2_gazebo_scene/scene_model.hpp"
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace xgc2_gazebo_scene {

struct PreparedSceneCommand;

// The world authority performs membership changes and retains entity generations.
// This callback returns only after native realization/deletion completion. It
// must also preserve unchanged model instances when only the scene revision changes.
using NativeSceneApply = std::function<std::map<std::string, gazebo::physics::ModelPtr>(
    const std::map<std::string, SceneModel>&, std::chrono::steady_clock::time_point)>;

class NativeSceneController {
  public:
    // clock reads the world's engine-owner-published exact nanosecond cache.
    NativeSceneController(gazebo::physics::WorldPtr world, std::string mesh_root, NativeSceneApply apply,
                          std::function<std::int64_t()> clock);
    ~NativeSceneController();
    // Prepare validates and allocates off the physics path. Throws invalid_argument.
    std::shared_ptr<const PreparedSceneCommand> Prepare(const std::string& route, const std::string& json);
    // Called by the world's single bounded management worker. Returns domain
    // result JSON; the world host owns admission, operation IDs and receipts.
    std::string Execute(const std::shared_ptr<const PreparedSceneCommand>& command, bool* effects_started = nullptr);
    // Physics update only: no parsing, sockets, waiting or trajectory allocation.
    void Update(double simulation_seconds);
    void Reset();
    std::string Status() const;
    // Management-thread observation of the frozen definition and actual native
    // model state. Simulation nanoseconds are encoded as a decimal string.
    std::string Snapshot() const;
    // Allocation-free change signal for the world's held-observation dispatcher.
    std::uint64_t serial() const noexcept;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace xgc2_gazebo_scene
