#include "xgc2_gazebo_scene/physical_contact_filter.hpp"

#include <algorithm>

namespace xgc2_gazebo_scene {
namespace {

bool IsTrackedDynamicActor(const ContactModelDescriptor& model,
                           const std::vector<std::string>& tracked_model_prefixes) {
    return !model.is_static && !model.is_managed_obstacle &&
           MatchesTrackedPrefix(model.name, tracked_model_prefixes);
}

} // namespace

bool MatchesTrackedPrefix(const std::string& model_name, const std::vector<std::string>& tracked_model_prefixes) {
    if (tracked_model_prefixes.empty()) {
        return true;
    }
    return std::any_of(tracked_model_prefixes.begin(), tracked_model_prefixes.end(),
                       [&model_name](const std::string& prefix) {
                           return !prefix.empty() && model_name.compare(0, prefix.size(), prefix) == 0;
                       });
}

bool IsForbiddenPhysicalContact(const ContactModelDescriptor& first, const ContactModelDescriptor& second,
                                const std::vector<std::string>& tracked_model_prefixes) {
    if (first.name.empty() || second.name.empty() || first.name == second.name) {
        return false;
    }

    const bool first_tracked = IsTrackedDynamicActor(first, tracked_model_prefixes);
    const bool second_tracked = IsTrackedDynamicActor(second, tracked_model_prefixes);
    if ((first_tracked && second.is_managed_obstacle) || (second_tracked && first.is_managed_obstacle)) {
        return true;
    }
    return first_tracked && second_tracked;
}

} // namespace xgc2_gazebo_scene
