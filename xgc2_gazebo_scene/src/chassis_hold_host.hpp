#pragma once
#include "xgc2_gazebo_scene/chassis_hold.hpp"
#include <boost/weak_ptr.hpp>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <xgc2/chassis_hold/domain.hpp>
#include <xgc2/chassis_hold/http_adapter.hpp>
#include <xgc2/chassis_hold/service.hpp>

namespace xgc2_gazebo_scene::detail {
// The clock of the HOLD domain and of the receipt times of commands: monotonic, so it keeps running while
// the world is paused.
std::int64_t MonotonicNs();

// ROS callback queue for chassis commands: one dispatcher thread per world, started with the first queue.
// Each callback is stamped with MonotonicNs() when it is queued; ReceiptTime() returns the stamp of the
// callback the calling thread is dispatching.
class CommandQueue {
  public:
    CommandQueue();
    ~CommandQueue();
    CommandQueue(const CommandQueue&) = delete;
    CommandQueue& operator=(const CommandQueue&) = delete;
    ros::CallbackQueueInterface* Interface();
    // The stamp of the callback being dispatched on this thread, MonotonicNs() outside a dispatch.
    static std::int64_t ReceiptTime();

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

class ChassisHoldHost;

// One chassis model's reservation of a robot id. Shared by the model's ChassisHold, the host registry and the
// native tick; `mutex` serializes the model's actions (see ChassisOutput) and guards `closed` and `output`.
struct ChassisSeat {
    std::string id;
    std::shared_ptr<ChassisHoldHost> host;
    std::shared_ptr<CommandQueue> queue; // set by the model's thread when it asks for the queue
    std::mutex mutex;
    bool closed = false;
    ChassisOutput output;
};

// The HOLD host of one world: the chassis HOLD domain and JSON service, their http.v1 methods, the
// native tick and the seats of the chassis models. Created and stopped by the world's SimulationService; the
// seats keep the object (and so the domain) alive for as long as a model plugin is loaded.
class ChassisHoldHost : public std::enable_shared_from_this<ChassisHoldHost> {
  public:
    // `instance_id` is the instance of the world's http.v1 server: HOLD bodies and the transport agree on it.
    static std::shared_ptr<ChassisHoldHost> Create(gazebo::physics::WorldPtr world, const std::string& instance_id);
    // The running host of a world, or null.
    static std::shared_ptr<ChassisHoldHost> Of(const gazebo::physics::World* world);
    ~ChassisHoldHost();
    ChassisHoldHost(const ChassisHoldHost&) = delete;
    ChassisHoldHost& operator=(const ChassisHoldHost&) = delete;

    // Starts the domain's tick thread and accepts model seats.
    void Start();
    // Ends the seats' access to the host, stops the tick thread and answers pending engages.
    void Stop();

    const xgc2::chassis_hold::HttpAdapter& http() const { return *http_; }

    std::shared_ptr<ChassisSeat> Reserve(const std::string& id, ChassisOutput output);
    void Enroll(ChassisSeat& seat);
    // Ends a seat: optionally writes a final zero first, then removes the robot from the roster.
    void Close(ChassisSeat& seat, bool write_zero) noexcept;
    // The seat is bound to a model with a complete plugin: it is in the roster.
    bool Bound(const std::string& id) const;
    // Closes the seat of `id` (the entity is being removed, its links are about to be finalized).
    void Retire(const std::string& id);
    // Writes zero to the model of `id` now (entity and world resets). False when the model failed to.
    bool ZeroOutput(const std::string& id);

    xgc2::chassis_hold::Domain& domain() { return domain_; }
    // The command queue of the world, started by its first caller; throws without an initialized ROS node.
    std::shared_ptr<CommandQueue> Queue();

  private:
    ChassisHoldHost(gazebo::physics::WorldPtr world, const std::string& instance_id);
    class Sink;
    std::shared_ptr<ChassisSeat> Find(const std::string& id) const;
    void Wake();
    void Run();
    // Writes zero and reports feedback on the domain's one tick thread, under the physics update lock.
    void Tick();
    void Feedback();

    boost::weak_ptr<gazebo::physics::World> world_; // identity only: the host never keeps a World alive
    xgc2::chassis_hold::Domain domain_;
    std::unique_ptr<xgc2::chassis_hold::Service> service_;
    std::unique_ptr<xgc2::chassis_hold::HttpAdapter> http_;
    std::unique_ptr<Sink> sink_;
    std::int64_t last_feedback_ = 0; // under the physics update lock, like Tick()

    mutable std::mutex seats_mutex_;
    std::map<std::string, std::weak_ptr<ChassisSeat>> seats_;
    std::shared_ptr<CommandQueue> queue_;
    bool accepting_ = false;

    std::mutex wake_mutex_;
    std::condition_variable wake_cv_;
    bool wake_ = false, stopping_ = false;
    std::thread thread_;
};
} // namespace xgc2_gazebo_scene::detail
