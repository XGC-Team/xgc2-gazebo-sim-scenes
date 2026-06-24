#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s "ros-${ROS_DISTRO}-xgc2-gazebo-sim-worlds" >/dev/null
test "$(rospack find gazebo_sim_worlds)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds"

test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/common/empty.world"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/scenes/weston_robot_empty.world"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/scenes/clearpath_playpen.world"

xmllint --noout \
  "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/package.xml" \
  "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/common/empty.world" \
  "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/scenes/weston_robot_empty.world" \
  "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/worlds/scenes/clearpath_playpen.world"

echo "Installed package check passed"
