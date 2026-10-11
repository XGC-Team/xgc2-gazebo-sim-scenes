#include "chassis_hold_host.hpp"
#include <boost/thread/recursive_mutex.hpp>
#include <chrono>
#include <cmath>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <stdexcept>
#include <utility>

namespace xgc2_gazebo_scene {
namespace detail {
namespace {
using xgc2::chassis_hold::Status;
using xgc2::xrpc::HttpReply;
using xgc2::xrpc::HttpRequest;

// The world's own HOLD host, found by the model plugins of that world.
std::mutex hosts_mutex;
std::map<const gazebo::physics::World*, std::weak_ptr<ChassisHoldHost>> hosts;

// The capability and service name of the HOLD domain on every transport.
const std::string kService = "xgc2.chassis.hold";
const std::string kCallPrefix = "/v1/call/";

constexpr std::chrono::milliseconds kPausedTick{10};
// Feedback is read at most this often: the domain needs samples no further apart than its 300 ms dwell.
constexpr std::int64_t kFeedbackIntervalNs = 5000000;
constexpr std::size_t kMaxIds = 256;
constexpr std::size_t kMaxBodyBytes = 65536;

int HttpStatus(Status status) {
    switch (status) {
    case Status::ok:
        return 200;
    case Status::invalid_argument:
        return 400;
    case Status::not_found:
        return 404;
    case Status::internal:
        break;
    }
    return 500;
}

xgc2::chassis_hold::DomainOptions DomainOptions(const std::string& instance_id) {
    xgc2::chassis_hold::DomainOptions options;
    options.instance_id = instance_id;
    options.clock = MonotonicNs;
    return options;
}

xgc2::chassis_hold::ServiceOptions ServiceOptions() {
    xgc2::chassis_hold::ServiceOptions options;
    options.max_ids = kMaxIds;
    options.max_body_bytes = kMaxBodyBytes;
    return options;
}
} // namespace

// Zero output of the domain: the zero writers of the bound models. The tick calls it with the physics update
// lock held and collects the robots it wrote, to read their feedback afterwards.
class ChassisHoldHost::Sink final : public xgc2::chassis_hold::ZeroSink {
  public:
    explicit Sink(ChassisHoldHost& host) : host_(host) {}
    bool write_zero(const std::string& id, std::string& error) override {
        const auto seat = host_.Find(id);
        if (!seat) {
            error = "no model is bound to the robot";
            return false;
        }
        std::lock_guard<std::mutex> lock(seat->mutex);
        if (seat->closed) {
            error = "the model is gone";
            return false;
        }
        seat->output.zero();
        written.push_back(seat);
        return true;
    }
    std::vector<std::shared_ptr<ChassisSeat>> written;

  private:
    ChassisHoldHost& host_;
};

std::shared_ptr<ChassisHoldHost> ChassisHoldHost::Create(gazebo::physics::WorldPtr world,
                                                         const std::string& instance_id) {
    std::shared_ptr<ChassisHoldHost> host(new ChassisHoldHost(world, instance_id));
    std::lock_guard<std::mutex> lock(hosts_mutex);
    auto& slot = hosts[world.get()];
    if (slot.lock())
        throw std::runtime_error("world already has a chassis HOLD host");
    slot = host;
    return host;
}

std::shared_ptr<ChassisHoldHost> ChassisHoldHost::Of(const gazebo::physics::World* world) {
    std::lock_guard<std::mutex> lock(hosts_mutex);
    const auto found = hosts.find(world);
    return found == hosts.end() ? nullptr : found->second.lock();
}

ChassisHoldHost::ChassisHoldHost(gazebo::physics::WorldPtr world, const std::string& instance_id)
    : world_(world), domain_(DomainOptions(instance_id)),
      service_(std::make_unique<xgc2::chassis_hold::Service>(domain_, ServiceOptions())),
      sink_(std::make_unique<Sink>(*this)) {}

ChassisHoldHost::~ChassisHoldHost() {
    Stop();
}

void ChassisHoldHost::Start() {
    {
        std::lock_guard<std::mutex> lock(seats_mutex_);
        accepting_ = true;
    }
    thread_ = std::thread([this] {
        Run();
    });
}

void ChassisHoldHost::Stop() {
    {
        std::lock_guard<std::mutex> lock(hosts_mutex);
        for (auto it = hosts.begin(); it != hosts.end();) {
            if (it->second.expired() || it->second.lock().get() == this)
                it = hosts.erase(it);
            else
                ++it;
        }
    }
    {
        std::lock_guard<std::mutex> lock(seats_mutex_);
        accepting_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(wake_mutex_);
        stopping_ = true;
    }
    wake_cv_.notify_all();
    if (thread_.joinable())
        thread_.join();
    // Pending engages are answered with the state reached so far.
    service_.reset();
}

bool ChassisHoldHost::Calls(const std::string& target) {
    return target.rfind(kCallPrefix, 0) == 0;
}

void ChassisHoldHost::Handle(HttpRequest request, HttpReply reply) {
    // POST /v1/call/xgc2.chassis.hold/<Method>: the method is the segment after the service name.
    const std::string prefix = kCallPrefix + kService + "/";
    if (request.method != "POST" || request.target.rfind(prefix, 0) != 0) {
        reply.complete(xgc2::xrpc::http_error(404, "not_found", "no such capability method"));
        return;
    }
    const std::string method = request.target.substr(prefix.size());
    if (!service_) {
        reply.complete(xgc2::xrpc::http_error(503, "unavailable", "chassis HOLD host is stopped"));
        return;
    }
    // The reply copy lives in the callback until the service has called it, then it is released.
    service_->call_async(method, request.body, request.deadline,
                         [reply](xgc2::chassis_hold::ServiceReply result) mutable {
                             reply.complete({HttpStatus(result.status), {{"Content-Type", "application/json"}},
                                             std::move(result.body), true});
                             reply = {};
                         });
    // An engage has gated the robot; the native tick writes the zero the reply waits for.
    if (method == "Engage")
        Wake();
}

void ChassisHoldHost::Wake() {
    {
        std::lock_guard<std::mutex> lock(wake_mutex_);
        wake_ = true;
    }
    wake_cv_.notify_one();
}

// The world thread ticks on every update. This thread covers the paused world, where updates stop, and
// writes the zero of a new engage at once instead of at the next update.
void ChassisHoldHost::Run() {
    if (const auto world = world_.lock())
        world->Physics()->InitForThread();
    std::unique_lock<std::mutex> lock(wake_mutex_);
    while (!stopping_) {
        wake_cv_.wait_for(lock, kPausedTick, [this] {
            return stopping_ || wake_;
        });
        if (stopping_)
            break;
        const bool requested = std::exchange(wake_, false);
        lock.unlock();
        if (const auto world = world_.lock(); world && world->Running() && (requested || world->IsPaused()))
            Tick();
        lock.lock();
    }
}

void ChassisHoldHost::Tick() {
    const auto world = world_.lock();
    if (!world)
        return;
    // Zero writes and feedback reads touch the physics objects: they never overlap a physics step, and ticks
    // of the world thread and of the tick thread never overlap each other.
    boost::recursive_mutex::scoped_lock physics(*world->Physics()->GetPhysicsUpdateMutex());
    sink_->written.clear();
    domain_.tick(*sink_);
    if (!sink_->written.empty())
        Feedback();
    sink_->written.clear();
}

// The feedback of the robots that were just zeroed, so the sample is never older than the zero.
void ChassisHoldHost::Feedback() {
    const auto now = domain_.now();
    if (now - last_feedback_ < kFeedbackIntervalNs)
        return;
    last_feedback_ = now;
    for (const auto& seat : sink_->written) {
        std::lock_guard<std::mutex> lock(seat->mutex);
        if (seat->closed || !seat->output.speed)
            continue;
        double linear = 0, angular = 0;
        try {
            seat->output.speed(linear, angular);
        } catch (...) {
            continue;
        }
        domain_.observe(seat->id, linear, angular, now);
    }
}

void ChassisHoldHost::DescribeFacts(Json::Value& facts) const {
    Json::Value capability;
    capability["name"] = kService;
    capability["entities"] = Json::Value(Json::arrayValue);
    for (const auto& robot : domain_.describe().robots)
        capability["entities"].append(robot.robot_id);
    facts["capabilities"].append(capability);
}

std::shared_ptr<ChassisSeat> ChassisHoldHost::Find(const std::string& id) const {
    std::lock_guard<std::mutex> lock(seats_mutex_);
    const auto found = seats_.find(id);
    return found == seats_.end() ? nullptr : found->second.lock();
}

std::shared_ptr<ChassisSeat> ChassisHoldHost::Reserve(const std::string& id, ChassisOutput output) {
    if (!xgc2::chassis_hold::valid_robot_id(id))
        throw std::runtime_error("chassis robot ID must be 1-128 characters of [A-Za-z0-9._:-]");
    if (!output.zero)
        throw std::invalid_argument("chassis output requires a zero writer");
    auto seat = std::make_shared<ChassisSeat>();
    seat->id = id;
    seat->host = shared_from_this();
    seat->output = std::move(output);
    std::lock_guard<std::mutex> lock(seats_mutex_);
    if (!accepting_)
        throw std::runtime_error("the chassis HOLD host of the world is not running");
    auto& slot = seats_[id];
    if (slot.lock())
        throw std::runtime_error("chassis robot is already bound to a model");
    slot = seat;
    return seat;
}

std::shared_ptr<CommandQueue> ChassisHoldHost::Queue() {
    std::lock_guard<std::mutex> lock(seats_mutex_);
    if (!queue_)
        queue_ = std::make_shared<CommandQueue>();
    return queue_;
}

void ChassisHoldHost::Enroll(ChassisSeat& seat) {
    std::lock_guard<std::mutex> lock(seat.mutex);
    if (seat.closed)
        throw std::runtime_error("chassis seat is closed");
    try {
        if (!domain_.add(seat.id, static_cast<bool>(seat.output.speed)))
            throw std::runtime_error("chassis robot is already in the HOLD roster");
    } catch (const std::length_error&) {
        throw std::runtime_error("the HOLD roster of the world is full");
    }
}

void ChassisHoldHost::Close(ChassisSeat& seat, bool write_zero) noexcept {
    {
        std::lock_guard<std::mutex> lock(seat.mutex);
        if (seat.closed)
            return;
        if (write_zero && seat.output.zero) {
            try {
                seat.output.zero();
            } catch (...) {
            }
        }
        seat.closed = true;
        seat.output = {};
    }
    // The HOLD state of the id is kept: the next model with this id inherits it.
    domain_.remove(seat.id);
    std::lock_guard<std::mutex> lock(seats_mutex_);
    const auto found = seats_.find(seat.id);
    if (found != seats_.end() && found->second.lock().get() == &seat)
        seats_.erase(found);
}

bool ChassisHoldHost::Bound(const std::string& id) const {
    return domain_.state({id}).unknown.empty();
}

void ChassisHoldHost::Retire(const std::string& id) {
    if (const auto seat = Find(id))
        Close(*seat, true);
}

bool ChassisHoldHost::ZeroOutput(const std::string& id) {
    const auto seat = Find(id);
    if (!seat)
        return true; // not a chassis model: nothing to zero
    std::lock_guard<std::mutex> lock(seat->mutex);
    if (seat->closed)
        return true;
    try {
        seat->output.zero();
        return true;
    } catch (...) {
        return false;
    }
}
} // namespace detail

std::function<void(double&, double&)> BodySpeed(gazebo::physics::Model* model) {
    return [model](double& linear, double& angular) {
        const auto velocity = model->WorldLinearVel(), rate = model->WorldAngularVel();
        linear = std::hypot(velocity.X(), velocity.Y());
        angular = std::abs(rate.Z());
    };
}

ChassisHold::ChassisHold(const gazebo::physics::WorldPtr& world, const std::string& robot_id, ChassisOutput output) {
    if (!world)
        throw std::invalid_argument("chassis HOLD requires a world");
    const auto host = detail::ChassisHoldHost::Of(world.get());
    if (!host)
        throw std::runtime_error("world does not host chassis HOLD");
    seat_ = host->Reserve(robot_id, std::move(output));
}

ChassisHold::~ChassisHold() {
    Close();
}

ros::CallbackQueueInterface* ChassisHold::CommandQueue() const {
    // The dispatcher of the world starts with its first user; a model without ROS commands needs none.
    if (!seat_->queue)
        seat_->queue = seat_->host->Queue();
    return seat_->queue->Interface();
}

void ChassisHold::Ready() {
    seat_->host->Enroll(*seat_);
}

void ChassisHold::Close() noexcept {
    if (seat_)
        seat_->host->Close(*seat_, false);
}

std::mutex& ChassisHold::Mutex() const {
    return seat_->mutex;
}

bool ChassisHold::Closed() const {
    return seat_->closed;
}

bool ChassisHold::Admitted() const {
    return seat_->host->domain().admit(seat_->id, detail::CommandQueue::ReceiptTime());
}

bool ChassisHold::Held() const {
    auto& domain = seat_->host->domain();
    return !domain.admit(seat_->id, domain.now());
}
} // namespace xgc2_gazebo_scene
