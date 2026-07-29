#pragma once

#include <string>
#include <vector>

namespace xgc2_gazebo_scene {

/// Gazebo-independent model metadata used to classify one physical contact.
struct ContactModelDescriptor {
    std::string name;
    bool is_static = false;
    bool is_managed_obstacle = false;
};

/// Return true when a model name begins with one of the configured prefixes.
///
/// An empty prefix list intentionally matches every model. This lets a scene
/// monitor all non-static actors without embedding platform-specific names.
bool MatchesTrackedPrefix(const std::string& model_name, const std::vector<std::string>& tracked_model_prefixes);

/// Classify contacts that invalidate a high-fidelity motion regression.
///
/// Forbidden:
///   * a tracked non-static actor against a managed scene obstacle;
///   * two distinct tracked non-static actors against each other.
///
/// Allowed:
///   * collisions belonging to the same top-level model (wheel/chassis,
///     rotor/body, and other self contacts);
///   * a tracked actor against an ordinary static model such as the ground;
///   * contacts involving only untracked actors.
bool IsForbiddenPhysicalContact(const ContactModelDescriptor& first, const ContactModelDescriptor& second,
                                const std::vector<std::string>& tracked_model_prefixes);

} // namespace xgc2_gazebo_scene
