#!/usr/bin/env bash
set -euo pipefail
unset DISPLAY WAYLAND_DISPLAY
: "${XGC2_PYTHON_EXECUTABLE:?select installed Python >=3.8 with the formal XRPC wheel}"
"${XGC2_PYTHON_EXECUTABLE}" -c "import sys; assert sys.version_info >= (3,8); from xgc2_xrpc.http import Client"

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s "ros-${ROS_DISTRO}-xgc2-gazebo-sim-worlds" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-gazebo-scene" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-simple-lidar" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-gazebo-rendering" >/dev/null
test "$(rospack find gazebo_sim_worlds)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds"
test "$(rospack find xgc2_gazebo_scene)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_gazebo_scene"
test "$(rospack find xgc2_simple_lidar)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_simple_lidar"
test "$(rospack find xgc2_gazebo_rendering)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_gazebo_rendering"

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
    libxgc2_simulation_service.so \
    libxgc2_simulation_world.so \
    libxgc2_simulation_entity_ack.so \
    libxgc2_simulation_sensor_ack.so \
    libxgc2_simulation_ros_data.so \
    libxgc2_gazebo_scene_system.so; do
  test -f "/opt/ros/${ROS_DISTRO}/lib/${library}"
done
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_gazebo_scene/ObstacleDefinition.h"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_gazebo_scene/obstacle_path_plugin.hpp"
test -f "/opt/ros/${ROS_DISTRO}/share/xgc2_gazebo_scene/msg/ObstacleDefinition.msg"
test ! -e "/opt/ros/${ROS_DISTRO}/share/xgc2_gazebo_scene/srv/ConfigureMotions.srv"
test ! -e "/opt/ros/${ROS_DISTRO}/share/xgc2_gazebo_scene/srv/StopMotions.srv"
test ! -e "/opt/ros/${ROS_DISTRO}/lib/libxgc2_scene_authoring_world.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages/xgc2_gazebo_scene/msg/_ObstacleDefinition.py"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_gazebo_scene/spawn_robot_model"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_gazebo_scene/simulation_world_prepare"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_gazebo_scene/delete_robot_model"
test -x "/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_worlds/native_world_start"
test -f "${world_root}/launch/native_world.launch"
test -f "/opt/ros/${ROS_DISTRO}/share/xgc2_simple_lidar/package.xml"
test -f "/opt/ros/${ROS_DISTRO}/share/xgc2_simple_lidar/models/sensor.xacro"
test -f "/opt/ros/${ROS_DISTRO}/share/xgc2_simple_lidar/models/sensor.sdf.xacro"
test -f "/opt/ros/${ROS_DISTRO}/share/xgc2_simple_lidar/cmake/xgc2_simple_lidarConfig.cmake"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_simple_lidar/scan_projection.hpp"
test -f "/opt/ros/${ROS_DISTRO}/lib/pkgconfig/xgc2_simple_lidar.pc"
test -f "/opt/ros/${ROS_DISTRO}/lib/libxgc2_simple_lidar.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libxgc2_simple_lidar_cpu.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libxgc2_gazebo_rendering.so"
test -f "/opt/ros/${ROS_DISTRO}/share/xgc2_gazebo_rendering/package.xml"
xmllint --noout "/opt/ros/${ROS_DISTRO}/share/xgc2_simple_lidar/package.xml"

for library in libobstaclePathPlugin.so libxgc2_gazebo_scene_system.so libxgc2_simulation_world.so libxgc2_simulation_entity_ack.so libxgc2_simulation_sensor_ack.so libxgc2_simulation_ros_data.so libxgc2_simple_lidar.so libxgc2_simple_lidar_cpu.so libxgc2_gazebo_rendering.so; do
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
    "/opt/ros/${ROS_DISTRO}/lib/libxgc2_simulation_service.so" \
    "/opt/ros/${ROS_DISTRO}/lib/libxgc2_simulation_world.so" \
    "/opt/ros/${ROS_DISTRO}/lib/libxgc2_simulation_entity_ack.so" \
    "/opt/ros/${ROS_DISTRO}/lib/libxgc2_simulation_sensor_ack.so" \
    "/opt/ros/${ROS_DISTRO}/lib/libxgc2_simulation_ros_data.so" \
    | grep -Eq '(RPATH|RUNPATH)'; then
  echo "Installed Gazebo Scene libraries contain RPATH/RUNPATH" >&2
  exit 1
fi
if ldd "/opt/ros/${ROS_DISTRO}/lib/libxgc2_simple_lidar.so" "/opt/ros/${ROS_DISTRO}/lib/libxgc2_simple_lidar_cpu.so" | grep -q 'not found'; then
  echo "Installed simple lidar library has unresolved shared libraries" >&2
  exit 1
fi
if readelf -d "/opt/ros/${ROS_DISTRO}/lib/libxgc2_simple_lidar.so" "/opt/ros/${ROS_DISTRO}/lib/libxgc2_simple_lidar_cpu.so" | grep -Eq '(RPATH|RUNPATH)'; then
  echo "Installed simple lidar library contains RPATH/RUNPATH" >&2
  exit 1
fi

"${XGC2_PYTHON_EXECUTABLE}" -c 'from xgc2_gazebo_scene.msg import ObstacleDefinition, ObstacleStateArray'
"${XGC2_PYTHON_EXECUTABLE}" -c 'from xgc2_geometry_msgs.msg import SceneSnapshot, SceneState; from xgc2_scene_runtime.simulation_client import SimulationClient; from xgc2_scene_runtime.prepare import prepare'

echo "Installed scene packages check passed"
