#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s "ros-${ROS_DISTRO}-xgc2-gazebo-sim-worlds" >/dev/null
test "$(rospack find gazebo_sim_worlds)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds"

test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/empty/empty.world"
test -f "/opt/ros/${ROS_DISTRO}/lib/libobstaclePathPlugin.so"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/clearpath_playpen/clearpath_playpen.world"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/corridor_dynamic_9/corridor_dynamic_9.world"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/README.md"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/empty/README.md"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/empty/preview.png"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/models/corridor/model.sdf"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/models/person/model.sdf"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/models/jersey_barrier/model.sdf"

xmllint --noout "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/package.xml"
while IFS= read -r xml_file; do
  xmllint --noout "${xml_file}"
done < <(
  {
    find "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds" -type f -name '*.world'
    find "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/models" -type f \( -name 'model.config' -o -name 'model.sdf' \)
  } | sort
)

while IFS= read -r world; do
  scene_dir="$(dirname "${world}")"
  test -f "${scene_dir}/README.md"
done < <(find "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds" -type f -name '*.world' | sort)

echo "Installed package check passed"
