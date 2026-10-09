#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <gazebo/physics/PhysicsTypes.hh>
#include <memory>
#include <mutex>
#include <ros/callback_queue.h>
#include <utility>

namespace xgc2_gazebo_scene {
namespace detail {
class ChassisRosDispatcher;
class ChassisRosQueue;
} // namespace detail
// One bounded ROS data dispatcher per native world. The wrapper owns no network
// endpoint. Its atomic lifetime fence is released before entering chassis domain
// state, so teardown cannot invert the domain -> vehicle state lock order.
class ChassisRosCallbacks {
  public:
    explicit ChassisRosCallbacks(gazebo::physics::WorldPtr world);
    ~ChassisRosCallbacks();
    ChassisRosCallbacks(const ChassisRosCallbacks&) = delete;
    ChassisRosCallbacks& operator=(const ChassisRosCallbacks&) = delete;
    ros::CallbackQueueInterface* queue() const;
    void invalidate() noexcept;
    bool current() const noexcept;
    void drain();
    template <class Action> auto wrap(Action action) const {
        const auto state = state_;
        return [state, action = std::move(action)](auto&&... arguments) mutable {
            if (!state->current())
                return;
            auto value = state->count.load(std::memory_order_acquire);
            // One attempt only. A concurrent callback/teardown skips this data sample.
            if ((value & State::closed) || value >= 16 ||
                !state->count.compare_exchange_strong(value, value + 1, std::memory_order_acq_rel))
                return;
            struct Exit {
                std::shared_ptr<State> state;
                ~Exit() {
                    const auto previous = state->count.fetch_sub(1, std::memory_order_release);
                    if (previous == (State::closed | 1u)) {
                        std::lock_guard<std::mutex> lock(state->drain_mutex);
                        state->drained.notify_all();
                    }
                }
            } exit{state};
            action(std::forward<decltype(arguments)>(arguments)...);
        };
    }

  private:
    friend class detail::ChassisRosQueue;
    struct State {
        static constexpr unsigned closed = 1u << 31;
        std::atomic<unsigned> count{0};
        std::atomic<std::uint64_t> epoch{0};
        std::mutex drain_mutex;
        std::condition_variable drained;
        bool current() const noexcept;
    };
    std::shared_ptr<State> state_;
    std::shared_ptr<detail::ChassisRosDispatcher> dispatcher_;
    std::unique_ptr<detail::ChassisRosQueue> queue_;
};
} // namespace xgc2_gazebo_scene
