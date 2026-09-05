#pragma once
#include <cmath>
#include <cstddef>
#include <vector>

namespace xgc2_gazebo_scene {
// Preserve fractional seconds. Zero-length segments are valid and will be
// collapsed when constructing the animation's keyframes.
inline bool SegmentDuration(double distance, double speed, double* duration) {
    if (duration == nullptr || !std::isfinite(distance) || distance < 0.0 || !std::isfinite(speed) || speed <= 0.0)
        return false;
    const double value = distance / speed;
    if (!std::isfinite(value) || (distance > 0.0 && value <= 0.0))
        return false;
    *duration = value;
    return true;
}

// Equal timestamps can arise from a zero-angle orientation key. Retain the
// last pose at each timestamp instead of feeding duplicate keys to Gazebo.
inline bool UniqueKeyframeIndices(const std::vector<double>& times, std::vector<std::size_t>* indices) {
    if (indices == nullptr)
        return false;
    indices->clear();
    for (std::size_t i = 0; i < times.size(); ++i) {
        if (!std::isfinite(times[i]) || times[i] < 0.0 || (i > 0 && times[i] < times[i - 1])) {
            indices->clear();
            return false;
        }
        if (i + 1 == times.size() || times[i + 1] != times[i])
            indices->push_back(i);
    }
    return !indices->empty();
}
} // namespace xgc2_gazebo_scene
