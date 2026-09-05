#include "xgc2_gazebo_scene/path_timing.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void Check(bool value) { if (!value) throw std::runtime_error("path timing regression"); }
void Duration(double distance, double speed, double expected) {
    double value = -1.0;
    Check(xgc2_gazebo_scene::SegmentDuration(distance, speed, &value));
    Check(std::abs(value - expected) < 1e-12);
}
int main() {
    Duration(0.5, 1.0, 0.5);
    Duration(1.5, 1.0, 1.5);
    Duration(4.0, 0.5, 8.0);
    Duration(0.25, 0.8, 0.3125);
    Duration(0.0, 0.8, 0.0);
    double value = 7.0;
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double speed : {0.0, -1.0, inf, nan}) {
        Check(!xgc2_gazebo_scene::SegmentDuration(1.0, speed, &value));
        Check(value == 7.0);
    }
    for (double distance : {-1.0, inf, nan})
        Check(!xgc2_gazebo_scene::SegmentDuration(distance, 1.0, &value));
    Check(!xgc2_gazebo_scene::SegmentDuration(1.0, 1.0, nullptr));
    Check(!xgc2_gazebo_scene::SegmentDuration(1e308, 1e-308, &value));
    std::vector<std::size_t> indices;
    Check(xgc2_gazebo_scene::UniqueKeyframeIndices({0.0, 0.5, 0.5, 2.0, 2.0}, &indices));
    Check(indices == std::vector<std::size_t>({0, 2, 4}));
    Check(xgc2_gazebo_scene::UniqueKeyframeIndices({0.0, 0.0}, &indices));
    Check(indices == std::vector<std::size_t>({1}));
    Check(!xgc2_gazebo_scene::UniqueKeyframeIndices({0.0, 1.0, 0.5}, &indices));
    Check(indices.empty());
    Check(!xgc2_gazebo_scene::UniqueKeyframeIndices({0.0, nan}, &indices));
    Check(!xgc2_gazebo_scene::UniqueKeyframeIndices({}, &indices));
    std::cout << "Fractional timing, invalid input and duplicate keyframe checks passed\n";
}
