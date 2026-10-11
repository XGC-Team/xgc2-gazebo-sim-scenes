#include "xgc2_gazebo_scene/ros_master_registration.hpp"
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <mutex>
#include <netdb.h>
#include <poll.h>
#include <random>
#include <ros/master.h>
#include <ros/init.h>
#include <ros/this_node.h>
#include <ros/time.h>
#include <stdexcept>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <xmlrpcpp/XmlRpcValue.h>

namespace xgc2_gazebo_scene {
namespace detail {
using Clock = std::chrono::steady_clock;
struct RosRegistration {
    std::mutex mutex;
    std::function<void()> callback;
    bool applied = false;
};

class RosMasterWatcher {
  public:
    RosMasterWatcher() : wake_fd_(eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK)) {
        if (wake_fd_ < 0)
            throw std::runtime_error("ROS master watcher wake allocation failed");
        status_.uri = ros::master::getURI();
        status_.state = "waiting";
        ros::master::setRetryTimeout(ros::WallDuration(0.25));
        worker_ = std::thread([this] { Work(); });
    }
    ~RosMasterWatcher() {
        stopping_ = true;
        Wake();
        worker_.join();
        close(wake_fd_);
    }
    void Add(const std::shared_ptr<RosRegistration>& registration) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (registrations_.size() >= 512)
                throw std::length_error("ROS registration budget exceeded");
            registrations_.push_back(registration);
        }
        Wake();
    }
    void Remove(const std::shared_ptr<RosRegistration>& registration) {
        {
            // Cancellation drains only registration work, never a physics lock.
            std::lock_guard<std::mutex> lock(registration->mutex);
            registration->callback = {};
        }
        std::lock_guard<std::mutex> lock(mutex_);
        registrations_.erase(std::remove(registrations_.begin(), registrations_.end(), registration),
                             registrations_.end());
    }
    RosMasterStatus Status() {
        std::lock_guard<std::mutex> lock(mutex_);
        return status_;
    }

  private:
    void Wake() {
        const std::uint64_t one = 1;
        const auto ignored = write(wake_fd_, &one, sizeof(one));
        (void)ignored;
    }
    void DrainWake() {
        std::uint64_t count;
        while (read(wake_fd_, &count, sizeof(count)) == sizeof(count)) {}
    }
    bool WaitSocket(int fd, short events, Clock::time_point deadline) {
        while (!stopping_) {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
            if (remaining.count() <= 0)
                return false;
            pollfd descriptors[] = {{fd, events, 0}, {wake_fd_, POLLIN, 0}};
            const auto result = poll(descriptors, 2, static_cast<int>(remaining.count()));
            if (result < 0 && errno == EINTR)
                continue;
            if (descriptors[1].revents)
                DrainWake();
            if (result <= 0 || stopping_)
                return false;
            if (descriptors[0].revents)
                return true;
        }
        return false;
    }
    // roscpp's synchronous master calls have no socket deadline and log each
    // retry. This bounded identity read neither starts ROS nor registers topics.
    bool Call(const char* method, XmlRpc::XmlRpcValue parameters, XmlRpc::XmlRpcValue& payload) {
        addrinfo hints{}, *addresses = nullptr;
        hints.ai_socktype = SOCK_STREAM;
        const auto port = std::to_string(ros::master::getPort());
        if (getaddrinfo(ros::master::getHost().c_str(), port.c_str(), &hints, &addresses))
            return false;
        std::unique_ptr<addrinfo, decltype(&freeaddrinfo)> address_owner(addresses, freeaddrinfo);
        struct Socket {
            int fd;
            ~Socket() { if (fd >= 0) close(fd); }
        } socket_owner{-1};
        const auto deadline = Clock::now() + std::chrono::seconds(1);
        for (auto address = addresses; address && !stopping_; address = address->ai_next) {
            const int candidate = socket(address->ai_family, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
            if (candidate < 0)
                continue;
            socket_owner.fd = candidate;
            const bool connecting = connect(candidate, address->ai_addr, address->ai_addrlen) == 0 ||
                                    errno == EINPROGRESS;
            int error = 0;
            socklen_t size = sizeof(error);
            if (connecting && WaitSocket(candidate, POLLOUT, deadline) &&
                !getsockopt(candidate, SOL_SOCKET, SO_ERROR, &error, &size) && !error)
                break;
            close(candidate);
            socket_owner.fd = -1;
        }
        if (socket_owner.fd < 0)
            return false;
        const int fd = socket_owner.fd;
        std::string body = "<?xml version=\"1.0\"?><methodCall><methodName>" + std::string(method) +
                           "</methodName><params>";
        for (int i = 0; i < parameters.size(); ++i)
            body += "<param>" + parameters[i].toXml() + "</param>";
        body += "</params></methodCall>";
        const std::string request = "POST /RPC2 HTTP/1.1\r\nHost: " + ros::master::getHost() +
                                   "\r\nContent-Type: text/xml\r\nConnection: close\r\nContent-Length: " +
                                   std::to_string(body.size()) + "\r\n\r\n" + body;
        std::size_t sent = 0;
        while (sent < request.size()) {
            if (!WaitSocket(fd, POLLOUT, deadline))
                return false;
            const auto count = send(fd, request.data() + sent, request.size() - sent, MSG_NOSIGNAL);
            if (count < 0 && (errno == EAGAIN || errno == EINTR))
                continue;
            if (count <= 0)
                return false;
            sent += count;
        }
        std::string response;
        char buffer[4096];
        while (response.size() < 16384) {
            if (!WaitSocket(fd, POLLIN, deadline))
                return false;
            const auto count = recv(fd, buffer, sizeof(buffer), 0);
            if (count < 0 && (errno == EAGAIN || errno == EINTR))
                continue;
            if (count <= 0)
                break;
            response.append(buffer, count);
            if (response.find("</methodResponse>") != std::string::npos)
                break;
        }
        const auto header = response.find("\r\n\r\n");
        if (header == std::string::npos || response.find(" 200 ") > header || response.size() > 16384)
            return false;
        const auto value = response.find("<value>", header + 4);
        if (value == std::string::npos)
            return false;
        int offset = static_cast<int>(value);
        XmlRpc::XmlRpcValue result;
        if (!result.fromXml(response, &offset) || result.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            result.size() != 3 || result[0].getType() != XmlRpc::XmlRpcValue::TypeInt || int(result[0]) != 1)
            return false;
        payload = result[2];
        return true;
    }
    void Work() {
        std::mt19937 random(std::random_device{}());
        int ceiling_ms = 250;
        auto next_probe = Clock::now(), last_log = Clock::time_point{};
        std::uint64_t suppressed = 0;
        const auto failure = [&](Clock::time_point now, const char* reason) {
            ++suppressed;
            if (last_log == Clock::time_point{} || now - last_log >= std::chrono::seconds(10)) {
                std::fprintf(stderr, "ROS master unavailable (%s); %llu suppressed attempts\n",
                             reason, static_cast<unsigned long long>(suppressed - 1));
                last_log = now;
                suppressed = 0;
            }
            next_probe = Clock::now() + std::chrono::milliseconds(
                std::uniform_int_distribution<int>(0, ceiling_ms)(random));
            ceiling_ms = std::min(30000, ceiling_ms * 2);
        };
        int bound_pid = 0;
        std::string bound_run_id;
        while (!stopping_) {
            const auto now = Clock::now();
            if (now >= next_probe) {
                XmlRpc::XmlRpcValue parameters, pid, run_id;
                parameters[0] = ros::this_node::getName();
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    ++status_.attempts;
                }
                const bool available = Call("getPid", parameters, pid) &&
                                       pid.getType() == XmlRpc::XmlRpcValue::TypeInt && int(pid) > 0;
                if (available) {
                    parameters[1] = "/run_id";
                    Call("getParam", parameters, run_id);
                    const auto identity = run_id.getType() == XmlRpc::XmlRpcValue::TypeString ?
                                              std::string(run_id) : std::string{};
                    const bool replaced = bound_pid && (bound_pid != int(pid) ||
                        (!bound_run_id.empty() && !identity.empty() && bound_run_id != identity));
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        status_.state = replaced ? "replaced" : "connected";
                        status_.pid = int(pid);
                        status_.run_id = identity;
                    }
                    if (replaced) {
                        std::fprintf(stderr, "ROS master replaced; ending the bound Gazebo process\n");
                        kill(getpid(), SIGINT);
                        return;
                    }
                    bound_pid = int(pid);
                    if (!identity.empty())
                        bound_run_id = identity;
                    next_probe = Clock::now() + std::chrono::seconds(10);
                } else {
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        status_.state = bound_pid ? "unavailable" : "waiting";
                    }
                    failure(now, "identity read failed");
                }
            }
            std::vector<std::shared_ptr<RosRegistration>> registrations;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (status_.state == "connected")
                    registrations = registrations_;
            }
            for (const auto& registration : registrations) {
                std::lock_guard<std::mutex> lock(registration->mutex);
                if (!stopping_ && !registration->applied && registration->callback) {
                    try {
                        registration->callback();
                        registration->applied = true;
                    } catch (const std::exception& error) {
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            status_.state = "unavailable";
                        }
                        failure(Clock::now(), error.what());
                        break;
                    }
                }
            }
            if (Status().state == "connected") {
                ceiling_ms = 250;
                last_log = {};
                suppressed = 0;
            }
            const auto wait = std::chrono::duration_cast<std::chrono::milliseconds>(next_probe - Clock::now());
            pollfd wake{wake_fd_, POLLIN, 0};
            poll(&wake, 1, static_cast<int>(std::max<std::int64_t>(1, wait.count())));
            DrainWake();
        }
    }
    int wake_fd_;
    std::atomic<bool> stopping_{false};
    std::mutex mutex_;
    RosMasterStatus status_;
    std::vector<std::shared_ptr<RosRegistration>> registrations_;
    std::thread worker_;
};
} // namespace detail

namespace {
std::mutex watcher_mutex;
std::weak_ptr<detail::RosMasterWatcher> watcher;
}

RosMasterStatus CurrentRosMasterStatus() {
    std::lock_guard<std::mutex> lock(watcher_mutex);
    const auto active = watcher.lock();
    return active ? active->Status() : RosMasterStatus{};
}
RosMasterRegistration::RosMasterRegistration(std::function<void()> register_node)
    : registration_(std::make_shared<detail::RosRegistration>()) {
    if (!ros::isInitialized())
        throw std::logic_error("deferred ROS registration requires ros::init");
    registration_->callback = std::move(register_node);
    std::lock_guard<std::mutex> lock(watcher_mutex);
    watcher_ = watcher.lock();
    if (!watcher_) {
        watcher_ = std::make_shared<detail::RosMasterWatcher>();
        watcher = watcher_;
    }
    watcher_->Add(registration_);
}
RosMasterRegistration::~RosMasterRegistration() { watcher_->Remove(registration_); }
} // namespace xgc2_gazebo_scene
