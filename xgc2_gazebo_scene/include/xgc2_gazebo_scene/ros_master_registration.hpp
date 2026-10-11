#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace xgc2_gazebo_scene {
namespace detail {
class RosMasterWatcher;
struct RosRegistration;
} // namespace detail

struct RosMasterStatus {
    std::string state = "unused", uri, run_id;
    int pid = 0;
    std::uint64_t attempts = 0;
};
RosMasterStatus CurrentRosMasterStatus();

// One process-wide watcher defers ROS registration off Gazebo's initialization
// and update threads. The owner cancels this binding before releasing callback
// resources. Registrations run once; a replacement master ends this process.
class RosMasterRegistration {
  public:
    explicit RosMasterRegistration(std::function<void()> register_node);
    ~RosMasterRegistration();
    RosMasterRegistration(const RosMasterRegistration&) = delete;
    RosMasterRegistration& operator=(const RosMasterRegistration&) = delete;

  private:
    std::shared_ptr<detail::RosMasterWatcher> watcher_;
    std::shared_ptr<detail::RosRegistration> registration_;
};
} // namespace xgc2_gazebo_scene
