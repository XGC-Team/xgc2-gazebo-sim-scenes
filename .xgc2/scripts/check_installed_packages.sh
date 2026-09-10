#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s "ros-${ROS_DISTRO}-xgc2-gazebo-sim-worlds" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-gazebo-scene" >/dev/null
test "$(rospack find gazebo_sim_worlds)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds"
test "$(rospack find xgc2_gazebo_scene)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_gazebo_scene"

world_root="/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds"
test -f "${world_root}/worlds/empty/empty.world"
test -f "${world_root}/worlds/clearpath_playpen/clearpath_playpen.world"
test -f "${world_root}/worlds/corridor_dynamic_9/corridor_dynamic_9.world"
test -f "${world_root}/worlds/catalog/empty.world"
test -f "${world_root}/worlds/catalog/scene_editable.world"
test -f "${world_root}/models/corridor/model.sdf"
test -f "${world_root}/models/person/model.sdf"
test -f "${world_root}/models/jersey_barrier/model.sdf"
xmllint --noout "${world_root}/package.xml"

for library in \
    libobstaclePathPlugin.so \
    libxgc2_gazebo_scene_motion.so \
    libxgc2_scene_model.so \
    libxgc2_scene_authoring_world.so \
    libxgc2_gazebo_scene_system.so; do
  test -f "/opt/ros/${ROS_DISTRO}/lib/${library}"
done
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_gazebo_scene/ObstacleDefinition.h"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_gazebo_scene/obstacle_path_plugin.hpp"
test -f "/opt/ros/${ROS_DISTRO}/share/xgc2_gazebo_scene/msg/ObstacleDefinition.msg"
test -f "/opt/ros/${ROS_DISTRO}/share/xgc2_gazebo_scene/srv/ConfigureMotions.srv"
test -f "/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages/xgc2_gazebo_scene/msg/_ObstacleDefinition.py"

for library in libobstaclePathPlugin.so libxgc2_gazebo_scene_system.so libxgc2_scene_authoring_world.so; do
  if ldd "/opt/ros/${ROS_DISTRO}/lib/${library}" | grep -q 'not found'; then
    echo "Installed Gazebo Scene library has unresolved dependencies: ${library}" >&2
    exit 1
  fi
  nm -D --defined-only "/opt/ros/${ROS_DISTRO}/lib/${library}" \
    | grep -E '[[:space:]]RegisterPlugin$' >/dev/null
done
if readelf -d \
    "/opt/ros/${ROS_DISTRO}/lib/libobstaclePathPlugin.so" \
    "/opt/ros/${ROS_DISTRO}/lib/libxgc2_gazebo_scene_motion.so" \
    "/opt/ros/${ROS_DISTRO}/lib/libxgc2_gazebo_scene_system.so" \
    "/opt/ros/${ROS_DISTRO}/lib/libxgc2_scene_model.so" \
    "/opt/ros/${ROS_DISTRO}/lib/libxgc2_scene_authoring_world.so" \
    | grep -Eq '(RPATH|RUNPATH)'; then
  echo "Installed Gazebo Scene libraries contain RPATH/RUNPATH" >&2
  exit 1
fi

python3 -c 'from xgc2_gazebo_scene.msg import ObstacleDefinition, ObstacleStateArray'
python3 -c 'from xgc2_gazebo_scene.srv import ConfigureMotions, StopMotions'
python3 -c 'from xgc2_geometry_msgs.msg import SceneSnapshot, SceneState; from xgc2_geometry_msgs.srv import ApplyScene'

echo "Installed scene packages check passed"
