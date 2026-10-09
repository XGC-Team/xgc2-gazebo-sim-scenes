#include "chassis_hold_domain.hpp"
#include <algorithm>
#include <gazebo/physics/World.hh>
#include <map>
#include <stdexcept>

namespace xgc2_gazebo_scene {
namespace {
std::mutex domains_mutex;
std::map<gazebo::physics::World*, std::weak_ptr<detail::ChassisDomain>> domains;
} // namespace
namespace detail {
void PublishChassisDomain(gazebo::physics::WorldPtr world, const std::shared_ptr<ChassisDomain>& domain) {
    std::lock_guard<std::mutex> lock(domains_mutex);
    auto& registered = domains[world.get()];
    if (!registered.expired())
        throw std::runtime_error("world already owns chassis hold");
    registered = domain;
}
void RetireChassisDomain(gazebo::physics::WorldPtr world, const std::shared_ptr<ChassisDomain>& domain) {
    std::lock_guard<std::mutex> lock(domains_mutex);
    const auto found = domains.find(world.get());
    if (found != domains.end() && found->second.lock() == domain)
        domains.erase(found);
}
} // namespace detail
ChassisBinding::ChassisBinding(gazebo::physics::WorldPtr world, std::string robot_id, void (*zero)(void*),
                               void* context)
    : context_(context) {
    if (!world || !zero || !context)
        throw std::invalid_argument("native chassis binding requires world and zero sink");
    {
        std::lock_guard<std::mutex> lock(domains_mutex);
        const auto found = domains.find(world.get());
        if (found != domains.end())
            domain_ = found->second.lock();
    }
    if (!domain_)
        throw std::runtime_error("world does not advertise a chassis hold authority");
    std::lock_guard<std::mutex> lock(domain_->mutex);
    const auto found = std::find(domain_->ids.begin(), domain_->ids.end(), robot_id);
    if (!domain_->alive || found == domain_->ids.end())
        throw std::invalid_argument("chassis robot is not granted by this world");
    index_ = static_cast<std::size_t>(found - domain_->ids.begin());
    if (domain_->bindings[index_].zero)
        throw std::runtime_error("chassis robot already has a command output owner");
    domain_->bindings[index_] = {zero, context};
}
void ChassisBinding::Ready() {
    std::lock_guard<std::mutex> lock(domain_->mutex);
    auto& binding = domain_->bindings[index_];
    if (!domain_->alive || binding.context != context_)
        throw std::runtime_error("world ended during chassis initialization");
    if (domain_->provider->held(index_))
        binding.zero(context_);
    binding.ready = true;
}
ChassisBinding::~ChassisBinding() {
    std::lock_guard<std::mutex> lock(domain_->mutex);
    if (domain_->bindings[index_].context == context_) {
        // Gazebo destroys model plugins after joints/links Fini. Never write a
        // retired actuator here. The native removal owner zeroes before Fini.
        domain_->bindings[index_] = {};
    }
}
ChassisBinding::Guard ChassisBinding::TryCommand() {
    Guard guard{std::unique_lock<std::mutex>(domain_->mutex, std::try_to_lock), true};
    if (guard.lock.owns_lock()) {
        if (!domain_->alive || !domain_->provider || !domain_->bindings[index_].ready ||
            domain_->bindings[index_].context != context_)
            guard.lock.unlock();
        else
            guard.held = domain_->provider->held(index_);
    }
    return guard;
}
} // namespace xgc2_gazebo_scene
