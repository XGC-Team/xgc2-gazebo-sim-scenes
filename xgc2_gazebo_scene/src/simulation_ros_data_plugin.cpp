#include "xgc2_gazebo_scene/scene_ownership.hpp"
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_msgs/ModelStates.h>
#include <map>
#include <memory>
#include <mutex>
#include <poll.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <set>
#include <stdexcept>
#include <string>
#include <sys/eventfd.h>
#include <thread>
#include <unistd.h>

namespace xgc2_gazebo_scene {
namespace {
std::string PublicName(const gazebo::physics::ModelPtr& model) {
    const auto native = model->GetName();
    if (IsSceneRuntimeModel(native))
        return native;
    const auto sdf = model->GetSDF();
    if (!sdf || !sdf->HasElement("plugin"))
        return native;
    auto last = sdf->GetElement("plugin");
    for (auto next = last->GetNextElement("plugin"); next; next = last->GetNextElement("plugin"))
        last = next;
    if (last->HasAttribute("filename") && last->Get<std::string>("filename") == "libxgc2_simulation_entity_ack.so" &&
        last->HasElement("public_entity_id"))
        return last->Get<std::string>("public_entity_id");
    return native;
}

class DataSource : public std::enable_shared_from_this<DataSource> {
  public:
    explicit DataSource(double rate) : rate_(rate), wake_fd_(eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC)) {
        if (wake_fd_ < 0)
            throw std::runtime_error("native ROS data wake allocation failed");
    }
    ~DataSource() {
        Stop();
        close(wake_fd_);
    }
    void Start() {
        worker_ = std::thread([this] {
            try {
                Work();
            } catch (const std::exception& error) {
                ROS_ERROR("native ROS data source stopped: %s", error.what());
            }
        });
    }
    void Attach(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_ || attached_)
            return;
        const auto world = gazebo::physics::get_world(name);
        if (!world)
            throw std::runtime_error("native ROS data world is unavailable");
        world_ = world;
        attached_ = true;
        // WorldCreated is the engine owner's initialization path. Prepared worlds
        // contain exactly one world, so the global TimeReset event has one owner.
        const auto initial = world->SimTime();
        clock_ns_.store(static_cast<std::int64_t>(initial.sec) * 1000000000LL + initial.nsec,
                        std::memory_order_release);
        const std::weak_ptr<DataSource> weak = shared_from_this();
        const boost::weak_ptr<gazebo::physics::World> owner = world;
        update_ = gazebo::event::Events::ConnectWorldUpdateEnd([weak, owner] {
            const auto self = weak.lock();
            const auto world = owner.lock();
            if (!self || !world || self->stopping_.load(std::memory_order_acquire))
                return;
            // SimTime is read only in the world update writer's engine context.
            const auto now = world->SimTime();
            self->clock_ns_.store(static_cast<std::int64_t>(now.sec) * 1000000000LL + now.nsec,
                                  std::memory_order_release);
            self->Wake();
        });
        reset_ = gazebo::event::Events::ConnectTimeReset([weak] {
            if (const auto self = weak.lock()) {
                self->clock_ns_.store(0, std::memory_order_release);
                self->Wake();
            }
        });
        created_ = gazebo::event::Events::ConnectAddEntity([weak](const std::string&) {
            if (const auto self = weak.lock())
                self->Wake();
        });
        deleted_ = gazebo::event::Events::ConnectDeleteEntity([weak](const std::string&) {
            if (const auto self = weak.lock())
                self->Wake();
        });
        Wake();
    }
    void Wake() noexcept {
        if (stopping_.load(std::memory_order_acquire) || requested_.exchange(true, std::memory_order_acq_rel))
            return;
        const std::uint64_t one = 1;
        const auto ignored = write(wake_fd_, &one, sizeof(one));
        (void)ignored;
    }
    void Stop() {
        if (stopping_.exchange(true))
            return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            update_.reset();
            reset_.reset();
            created_.reset();
            deleted_.reset();
        }
        const std::uint64_t one = 1;
        const auto ignored = write(wake_fd_, &one, sizeof(one));
        (void)ignored;
        if (worker_.joinable())
            worker_.join();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            world_.reset();
        }
    }

  private:
    void Work() {
        ros::NodeHandle node;
        const auto clock = node.advertise<rosgraph_msgs::Clock>("/clock", 10);
        const std::weak_ptr<DataSource> weak = shared_from_this();
        const ros::SubscriberStatusCallback connected = [weak](const ros::SingleSubscriberPublisher&) {
            if (auto self = weak.lock())
                self->Wake();
        };
        const auto models = node.advertise<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, connected,
                                                                     ros::SubscriberStatusCallback{});
        std::map<std::string, std::string> aliases;
        // Identity only: never keep a World alive across data frames or shutdown.
        const gazebo::physics::World* initialized = nullptr;
        std::int64_t last_clock = -1;
        while (!stopping_) {
            pollfd event{wake_fd_, POLLIN, 0};
            if (poll(&event, 1, -1) < 0) {
                if (errno == EINTR)
                    continue;
                break;
            }
            std::uint64_t count;
            while (read(wake_fd_, &count, sizeof(count)) == sizeof(count)) {
            }
            requested_.store(false, std::memory_order_release);
            if (stopping_)
                break;
            gazebo::physics::WorldPtr world;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                world = world_.lock();
            }
            if (!world)
                continue;
            if (initialized != world.get()) {
                world->Physics()->InitForThread();
                initialized = world.get();
            }
            try {
                rosgraph_msgs::Clock time;
                bool changed;
                {
                    const std::int64_t nanoseconds = clock_ns_.load(std::memory_order_acquire);
                    changed =
                        last_clock < 0 || nanoseconds < last_clock ||
                        (nanoseconds != last_clock && (rate_ == 0 || (nanoseconds - last_clock) * 1e-9 >= 1.0 / rate_));
                    if (changed) {
                        if (nanoseconds < 0 || nanoseconds / 1000000000LL > UINT32_MAX)
                            throw std::runtime_error("native clock exceeds ROS time range");
                        time.clock = ros::Time(static_cast<std::uint32_t>(nanoseconds / 1000000000LL),
                                               static_cast<std::uint32_t>(nanoseconds % 1000000000LL));
                        last_clock = nanoseconds;
                    }
                }
                if (changed)
                    clock.publish(time);
            } catch (const std::exception& error) {
                ROS_ERROR_THROTTLE(1.0, "native ROS clock refused: %s", error.what());
            }
            if (!models.getNumSubscribers())
                continue;
            try {
                gazebo_msgs::ModelStates states;
                {
                    boost::recursive_mutex::scoped_lock physics(*world->Physics()->GetPhysicsUpdateMutex());
                    const auto roster = world->Models();
                    if (roster.size() > 256)
                        throw std::runtime_error("native ROS model-state roster exceeds world budget");
                    states.name.reserve(roster.size());
                    states.pose.reserve(roster.size());
                    states.twist.reserve(roster.size());
                    std::set<std::string> current;
                    for (const auto& model : roster) {
                        const auto name = model->GetName();
                        current.insert(name);
                        auto alias = aliases.find(name);
                        if (alias == aliases.end())
                            alias = aliases.emplace(name, PublicName(model)).first;
                        if (alias->second.empty() || alias->second.size() > 4096)
                            throw std::runtime_error("invalid native ROS model-state name");
                        states.name.push_back(alias->second);
                        const auto pose = model->WorldPose();
                        geometry_msgs::Pose p;
                        p.position.x = pose.Pos().X();
                        p.position.y = pose.Pos().Y();
                        p.position.z = pose.Pos().Z();
                        p.orientation.x = pose.Rot().X();
                        p.orientation.y = pose.Rot().Y();
                        p.orientation.z = pose.Rot().Z();
                        p.orientation.w = pose.Rot().W();
                        states.pose.push_back(p);
                        geometry_msgs::Twist twist;
                        const auto linear = model->WorldLinearVel(), angular = model->WorldAngularVel();
                        twist.linear.x = linear.X();
                        twist.linear.y = linear.Y();
                        twist.linear.z = linear.Z();
                        twist.angular.x = angular.X();
                        twist.angular.y = angular.Y();
                        twist.angular.z = angular.Z();
                        states.twist.push_back(twist);
                    }
                    for (auto entry = aliases.begin(); entry != aliases.end();)
                        if (!current.count(entry->first))
                            entry = aliases.erase(entry);
                        else
                            ++entry;
                }
                // ROS serialization/socket work belongs only to this fixed data worker.
                models.publish(states);
            } catch (const std::exception& error) {
                ROS_ERROR_THROTTLE(1.0, "native ROS model-state snapshot refused: %s", error.what());
            }
        }
    }
    double rate_;
    int wake_fd_;
    std::atomic<bool> stopping_{false}, requested_{false};
    std::atomic<std::int64_t> clock_ns_{0};
    std::mutex mutex_;
    boost::weak_ptr<gazebo::physics::World> world_;
    bool attached_ = false;
    gazebo::event::ConnectionPtr update_, reset_, created_, deleted_;
    std::thread worker_;
};
} // namespace

class SimulationRosDataPlugin final : public gazebo::SystemPlugin {
  public:
    void Load(int, char**) override {
        if (!ros::isInitialized()) {
            std::map<std::string, std::string> remappings;
            for (const auto& item : {std::pair{"XGC_SIM_ROS_NAMESPACE", "__ns"}, std::pair{"XGC_SIM_ROS_LOG", "__log"}})
                if (const char* value = std::getenv(item.first); value && *value)
                    remappings[item.second] = value;
            const char* configured = std::getenv("XGC_SIM_ROS_NAME");
            ros::init(remappings, configured && *configured ? configured : "xgc2_simulation_user_data",
                      ros::init_options::NoSigintHandler);
            owns_ros_ = true;
        }
        double rate = 0;
        if (const char* configured = std::getenv("XGC_SIM_ROS_CLOCK_RATE_HZ")) {
            char* end = nullptr;
            errno = 0;
            rate = std::strtod(configured, &end);
            if (errno || end == configured || *end || !std::isfinite(rate) || rate < 0 || rate > 1e6)
                throw std::invalid_argument("native ROS clock rate must be finite in [0, 1000000]");
        }
        spinner_ = std::make_unique<ros::AsyncSpinner>(1);
        spinner_->start();
        data_ = std::make_shared<DataSource>(rate);
        data_->Start();
        const std::weak_ptr<DataSource> weak = data_;
        created_ = gazebo::event::Events::ConnectWorldCreated([weak](const std::string& name) {
            if (const auto data = weak.lock())
                data->Attach(name);
        });
    }
    ~SimulationRosDataPlugin() override {
        created_.reset();
        // gazebo::shutdown clears SystemPlugins before physics::fini. Drain this
        // worker here; global event callbacks hold only weak references and cannot
        // postpone World release until DiagnosticManager static destruction.
        if (data_)
            data_->Stop();
        if (spinner_)
            spinner_->stop();
        if (owns_ros_)
            ros::shutdown();
        data_.reset();
    }

  private:
    bool owns_ros_ = false;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
    std::shared_ptr<DataSource> data_;
    gazebo::event::ConnectionPtr created_;
};
GZ_REGISTER_SYSTEM_PLUGIN(SimulationRosDataPlugin)
} // namespace xgc2_gazebo_scene
