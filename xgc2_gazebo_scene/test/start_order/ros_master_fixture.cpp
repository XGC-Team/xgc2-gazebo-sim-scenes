#include "xgc2_gazebo_scene/ros_master_registration.hpp"
#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <poll.h>
#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t stopping = 0;
void Stop(int) {
    stopping = 1;
}
} // namespace

int main(int argc, char** argv) {
    bool topics = false, fail_registration = false;
    for (int i = 1; i < argc; ++i) {
        topics |= std::string(argv[i]) == "--topics";
        fail_registration |= std::string(argv[i]) == "--fail-registration";
    }
    ros::init(argc, argv, "gazebo_start_order_fixture",
              ros::init_options::NoSigintHandler | ros::init_options::NoRosout | ros::init_options::NoSimTime);
    std::signal(SIGINT, Stop);
    std::signal(SIGTERM, Stop);
    std::atomic<unsigned> first{0}, second{0}, cancelled{0};
    std::atomic<unsigned> registration_attempts{0};
    std::atomic<std::uint64_t> desired_revision{1};
    std::atomic<bool> ready{false};
    std::unique_ptr<ros::NodeHandle> node;
    ros::Publisher publisher;
    auto registration = std::make_unique<xgc2_gazebo_scene::RosMasterRegistration>([&] {
        if (topics) {
            node = std::make_unique<ros::NodeHandle>();
            publisher = node->advertise<std_msgs::UInt64>("/fixture/latest", 1, true);
            std_msgs::UInt64 latest;
            latest.data = desired_revision.load();
            publisher.publish(latest);
        }
        ++first;
        ready.store(true, std::memory_order_release);
    });
    auto another = std::make_unique<xgc2_gazebo_scene::RosMasterRegistration>([&] {
        if (++registration_attempts == 1 && fail_registration)
            throw std::runtime_error("master lost during registration");
        ++second;
    });
    {
        xgc2_gazebo_scene::RosMasterRegistration removed([&] {
            ++cancelled;
        });
    }
    while (!stopping) {
        pollfd input{STDIN_FILENO, POLLIN, 0};
        if (poll(&input, 1, 1000) <= 0)
            continue;
        std::string command;
        if (!std::getline(std::cin, command) || command == "stop")
            break;
        if (command.rfind("set ", 0) == 0) {
            desired_revision.store(std::stoull(command.substr(4)));
            if (topics && ready.load(std::memory_order_acquire)) {
                std_msgs::UInt64 latest;
                latest.data = desired_revision.load();
                publisher.publish(latest);
            }
        }
        const auto status = xgc2_gazebo_scene::CurrentRosMasterStatus();
        std::cout << "{\"state\":\"" << status.state << "\",\"attempts\":" << status.attempts << ",\"first\":" << first
                  << ",\"second\":" << second << ",\"cancelled\":" << cancelled << ",\"revision\":" << desired_revision
                  << ",\"registration_attempts\":" << registration_attempts << "}" << std::endl;
    }
    another.reset();
    registration.reset();
    publisher.shutdown();
    node.reset();
    ros::shutdown();
    return 0;
}
