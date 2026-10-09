#include <boost/make_shared.hpp>
#include <gazebo/physics/World.hh>
#include <map>
#include <mutex>
#include <ros/spinner.h>
#include <stdexcept>
#include <xgc2_gazebo_scene/chassis_ros_callbacks.hpp>

namespace xgc2_gazebo_scene {
namespace detail {
class ChassisRosDispatcher {
  public:
    ChassisRosDispatcher() : spinner(1, &callbacks) { spinner.start(); }
    ~ChassisRosDispatcher() {
        callbacks.disable();
        spinner.stop();
        callbacks.clear();
    }
    ros::CallbackQueue callbacks;
    ros::AsyncSpinner spinner;
    unsigned owners = 0; // registration mutex only; native callbacks never read it
};
namespace {
thread_local const void* dispatch_owner = nullptr;
thread_local std::uint64_t dispatch_epoch = 0;
} // namespace
class ChassisRosQueue final : public ros::CallbackQueueInterface {
  public:
    using State = ChassisRosCallbacks::State;
    ChassisRosQueue(std::shared_ptr<ChassisRosDispatcher> dispatcher, std::shared_ptr<State> state)
        : dispatcher_(std::move(dispatcher)), state_(std::move(state)) {}
    void addCallback(const ros::CallbackInterfacePtr& callback, std::uint64_t id = 0) override {
        if (state_->count.load(std::memory_order_acquire) & State::closed)
            return;
        // ROS receipt side only. Native gate invalidation never allocates or parses.
        dispatcher_->callbacks.addCallback(
            boost::make_shared<EpochCallback>(callback, state_, state_->epoch.load(std::memory_order_acquire)), id);
    }
    void removeByID(std::uint64_t id) override { dispatcher_->callbacks.removeByID(id); }

  private:
    class EpochCallback final : public ros::CallbackInterface {
      public:
        EpochCallback(ros::CallbackInterfacePtr callback, std::shared_ptr<State> state, std::uint64_t epoch)
            : callback_(std::move(callback)), state_(std::move(state)), epoch_(epoch) {}
        bool ready() override { return callback_->ready(); }
        CallResult call() override {
            struct Scope {
                const void* owner = dispatch_owner;
                std::uint64_t epoch = dispatch_epoch;
                ~Scope() {
                    dispatch_owner = owner;
                    dispatch_epoch = epoch;
                }
            } scope;
            dispatch_owner = state_.get();
            dispatch_epoch = epoch_;
            // Always consume the SubscriptionQueue entry. The wrapped vehicle action
            // rejects this admission epoch, including when the subscription was full.
            return callback_->call();
        }

      private:
        ros::CallbackInterfacePtr callback_;
        std::shared_ptr<State> state_;
        const std::uint64_t epoch_;
    };
    std::shared_ptr<ChassisRosDispatcher> dispatcher_;
    std::shared_ptr<State> state_;
};

} // namespace detail
namespace {
std::mutex registration_mutex;
std::map<gazebo::physics::World*, std::weak_ptr<detail::ChassisRosDispatcher>> dispatchers;
} // namespace
ChassisRosCallbacks::ChassisRosCallbacks(gazebo::physics::WorldPtr world) : state_(std::make_shared<State>()) {
    if (!world)
        throw std::invalid_argument("chassis ROS dispatcher requires native world");
    std::lock_guard<std::mutex> lock(registration_mutex);
    auto& entry = dispatchers[world.get()];
    dispatcher_ = entry.lock();
    if (!dispatcher_) {
        dispatcher_ = std::make_shared<detail::ChassisRosDispatcher>();
        entry = dispatcher_;
    }
    if (dispatcher_->owners == 16)
        throw std::runtime_error("world chassis ROS callback owner capacity exhausted");
    queue_ = std::make_unique<detail::ChassisRosQueue>(dispatcher_, state_);
    ++dispatcher_->owners;
}
ChassisRosCallbacks::~ChassisRosCallbacks() {
    drain();
    std::lock_guard<std::mutex> lock(registration_mutex);
    --dispatcher_->owners;
    for (auto it = dispatchers.begin(); it != dispatchers.end();) {
        if (it->second.expired() || (it->second.lock() == dispatcher_ && dispatcher_->owners == 0))
            it = dispatchers.erase(it);
        else
            ++it;
    }
}
bool ChassisRosCallbacks::State::current() const noexcept {
    return detail::dispatch_owner != this || detail::dispatch_epoch == epoch.load(std::memory_order_acquire);
}
ros::CallbackQueueInterface* ChassisRosCallbacks::queue() const {
    return queue_.get();
}
void ChassisRosCallbacks::invalidate() noexcept {
    state_->epoch.fetch_add(1, std::memory_order_acq_rel);
}
bool ChassisRosCallbacks::current() const noexcept {
    return state_->current();
}
void ChassisRosCallbacks::drain() {
    invalidate();
    auto value = state_->count.fetch_or(State::closed, std::memory_order_acq_rel) | State::closed;
    while (value != State::closed) {
        state_->count.wait(value, std::memory_order_acquire);
        value = state_->count.load(std::memory_order_acquire);
    }
}
} // namespace xgc2_gazebo_scene
