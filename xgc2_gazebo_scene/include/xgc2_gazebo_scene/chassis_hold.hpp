#pragma once
#include <functional>
#include <gazebo/physics/PhysicsTypes.hh>
#include <memory>
#include <mutex>
#include <string>

namespace ros {
class CallbackQueueInterface;
}

namespace xgc2_gazebo_scene {
namespace detail {
struct ChassisSeat;
}

// A chassis model's seat in the HOLD domain of its world host (xgc2-chassis-hold, see docs/chassis-hold.md).
// The world plugin owns one domain, its XRPC methods and the native tick; a model plugin only describes its
// output and routes its commands and control steps through the seat. Every model plugin follows one order:
//
//   Load:    construct the seat, subscribe cmd_vel on CommandQueue(), connect the update callbacks, Ready().
//   cmd_vel: Command([&] { remember the command; }) runs the action only for admitted commands.
//   update:  Control([&](bool held) { ... }) is the model's control step; while held it must not apply
//            commands (it may keep running a drive law whose command is zero).
//   unload:  disconnect the update callbacks, shut the subscriptions down, destroy the seat (Close()).
//
// Command, Control and the zero output run under one lock of the seat: they never overlap, so the model
// needs no lock of its own for the state they share. The lock is taken after the physics update lock; code
// inside the actions must not take the physics update lock of the world (Gazebo's getters and setters do not).
struct ChassisOutput {
    // Writes one zero motion command: clears the commands and filter state the model holds and zeroes its
    // actuators. The host calls it on every HOLD tick while the robot is held, with the world's physics update
    // lock and the seat lock held, on the HOLD thread whether the world runs or is paused. It must not block
    // and must write what the model's control step writes while held.
    std::function<void()> zero;
    // Optional trusted feedback: speed of the body in m/s and rad/s. A robot that provides it can reach the
    // `stopped` stage; one that does not never does. Same locks as `zero`.
    std::function<void(double& linear, double& angular)> speed;
};
// Feedback of a ground vehicle: the horizontal speed of the model and its yaw rate. Contact solvers leave a
// vertical residual velocity on a resting vehicle, so the vertical components do not count. The model must
// outlive the seat.
std::function<void(double&, double&)> BodySpeed(gazebo::physics::Model* model);

class ChassisHold {
  public:
    // Reserves `robot_id` for this model in the world's HOLD host (ids are 1-128 characters of [A-Za-z0-9._:-]).
    // Throws std::runtime_error when the world has no HOLD host, the id is invalid, or another model is
    // bound to it.
    ChassisHold(const gazebo::physics::WorldPtr& world, const std::string& robot_id, ChassisOutput output);
    ~ChassisHold();
    ChassisHold(const ChassisHold&) = delete;
    ChassisHold& operator=(const ChassisHold&) = delete;

    // ROS callback queue for cmd_vel: it takes the receipt time of a message when it is queued, on the clock of
    // the domain (not simulation time, which stops while the world is paused). Valid until the seat is destroyed.
    // The first call in a world starts the dispatcher thread of the world's commands; it throws
    // std::runtime_error when no ROS node is initialized.
    ros::CallbackQueueInterface* CommandQueue() const;
    // Adds the robot to the roster once the model is complete. A robot that is held starts held: the next tick
    // writes zero to this model. Throws std::runtime_error when the host is gone or the roster is full.
    void Ready();
    // Removes the robot from the roster (its HOLD state is kept for the next model with this id) and ends every
    // later action. Waits for an action in progress. Idempotent; the destructor calls it. Never touches the
    // model, whose joints and links are already finalized when its plugin is destroyed.
    void Close() noexcept;

    // Runs `action` if the command being dispatched is admitted: the robot is in the roster, not held, and the
    // command was received after its last release. Returns whether it ran. Call it from the cmd_vel callback, which
    // must be queued on CommandQueue(): only that queue knows when the message was received.
    template <class Action> bool Command(Action&& action) {
        std::lock_guard<std::mutex> lock(Mutex());
        if (Closed() || !Admitted())
            return false;
        action();
        return true;
    }
    // Runs the model's control step with the current HOLD state. Returns false after Close().
    template <class Action> bool Control(Action&& action) {
        std::lock_guard<std::mutex> lock(Mutex());
        if (Closed())
            return false;
        action(Held());
        return true;
    }

  private:
    std::mutex& Mutex() const;
    bool Closed() const;
    bool Admitted() const;
    bool Held() const;
    std::shared_ptr<detail::ChassisSeat> seat_;
};
} // namespace xgc2_gazebo_scene
