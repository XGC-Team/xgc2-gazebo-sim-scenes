#include "chassis_hold_host.hpp"
#include <boost/make_shared.hpp>
#include <chrono>
#include <ros/callback_queue.h>
#include <ros/init.h>
#include <ros/spinner.h>
#include <stdexcept>

namespace xgc2_gazebo_scene::detail {
namespace {
thread_local bool dispatching = false;
thread_local std::int64_t dispatch_stamp = 0;
} // namespace

std::int64_t MonotonicNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// roscpp queues one callback per received message, from the transport thread, at the moment the message
// arrives. The queue wraps it with that moment and runs it on the dispatcher thread. A subscription that
// drops its oldest message when full queues no callback for the new one, which then runs under the stamp of
// the older callback: the stamp can only be earlier than the receipt, never later.
class CommandQueue::Impl final : public ros::CallbackQueueInterface {
  public:
    Impl() : spinner_(1, &queue_) { spinner_.start(); }
    ~Impl() override {
        queue_.disable();
        spinner_.stop();
        queue_.clear();
    }
    void addCallback(const ros::CallbackInterfacePtr& callback, uint64_t owner_id = 0) override {
        queue_.addCallback(boost::make_shared<Stamped>(callback, MonotonicNs()), owner_id);
    }
    void removeByID(uint64_t owner_id) override { queue_.removeByID(owner_id); }

  private:
    class Stamped final : public ros::CallbackInterface {
      public:
        Stamped(ros::CallbackInterfacePtr callback, std::int64_t stamp) : callback_(std::move(callback)), stamp_(stamp) {}
        bool ready() override { return callback_->ready(); }
        CallResult call() override {
            struct Scope {
                bool active = dispatching;
                std::int64_t stamp = dispatch_stamp;
                ~Scope() {
                    dispatching = active;
                    dispatch_stamp = stamp;
                }
            } scope;
            dispatching = true;
            dispatch_stamp = stamp_;
            return callback_->call();
        }

      private:
        ros::CallbackInterfacePtr callback_;
        const std::int64_t stamp_;
    };
    ros::CallbackQueue queue_;
    ros::AsyncSpinner spinner_;
};

CommandQueue::CommandQueue() {
    if (!ros::isInitialized())
        throw std::runtime_error("chassis commands require an initialized ROS node (the ROS data owner)");
    impl_ = std::make_unique<Impl>();
}
CommandQueue::~CommandQueue() = default;
ros::CallbackQueueInterface* CommandQueue::Interface() {
    return impl_.get();
}
std::int64_t CommandQueue::ReceiptTime() {
    return dispatching ? dispatch_stamp : MonotonicNs();
}
} // namespace xgc2_gazebo_scene::detail
