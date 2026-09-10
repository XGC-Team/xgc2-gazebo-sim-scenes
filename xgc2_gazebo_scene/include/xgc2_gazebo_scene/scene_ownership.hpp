#pragma once

#include <string>

namespace xgc2_gazebo_scene {

inline constexpr char kSceneRuntimeModelPrefix[] = "xgc2_obstacle_scene_";

inline bool IsSceneRuntimeModel(const std::string& name) {
    return name.compare(0, sizeof(kSceneRuntimeModelPrefix) - 1, kSceneRuntimeModelPrefix) == 0;
}

} // namespace xgc2_gazebo_scene
