#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
ros_distro="${ROS_DISTRO:-noetic}"
workspace="${XGC2_GAZEBO_SIM_SCENES_TEST_WS:-${repo_root}/.ci/package-tests}"

if ! command -v catkin_make >/dev/null 2>&1; then
  echo "Missing required package test tool: catkin_make" >&2
  exit 1
fi

if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "Missing ROS setup: /opt/ros/${ros_distro}/setup.bash" >&2
  exit 1
fi

rm -rf "${workspace}"
mkdir -p "${workspace}/src"
ln -s "${repo_root}/xgc2_gazebo_scene" "${workspace}/src/xgc2_gazebo_scene"

# shellcheck source=/dev/null
source "/opt/ros/${ros_distro}/setup.bash"

catkin_make -C "${workspace}" -DCATKIN_ENABLE_TESTING=ON
catkin_make -C "${workspace}" run_tests_xgc2_gazebo_scene
catkin_test_results "${workspace}/build/test_results"
echo "Package tests passed."
