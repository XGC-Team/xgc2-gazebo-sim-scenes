#!/usr/bin/env bash
set -euo pipefail
unset DISPLAY WAYLAND_DISPLAY
: "${XGC2_PYTHON_EXECUTABLE:?select image Python >=3.10 explicitly}"
"${XGC2_PYTHON_EXECUTABLE}" -c "import sys; assert sys.version_info >= (3,10); from xgc2_xrpc.http import Client"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
ros_distro="${ROS_DISTRO:-noetic}"
workspace="${XGC2_GAZEBO_SIM_SCENES_TEST_WS:-${repo_root}/.ci/package-tests}"

if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "Missing ROS setup: /opt/ros/${ros_distro}/setup.bash" >&2
  exit 1
fi

# shellcheck source=/dev/null
set +u
source "/opt/ros/${ros_distro}/setup.bash"
set -u
if ! command -v catkin_make >/dev/null 2>&1; then
  echo "Missing required package test tool: catkin_make" >&2
  exit 1
fi

rm -rf "${workspace}"
mkdir -p "${workspace}/src"
ln -s "${repo_root}/xgc2_gazebo_scene" "${workspace}/src/xgc2_gazebo_scene"
ln -s "${repo_root}/xgc2_simple_lidar" "${workspace}/src/xgc2_simple_lidar"
ln -s "${repo_root}/gazebo_sim_worlds" "${workspace}/src/gazebo_sim_worlds"

export GAZEBO_MODEL_PATH="${repo_root}/gazebo_sim_worlds/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""

catkin_make -C "${workspace}" -DPYTHON_EXECUTABLE="${XGC2_PYTHON_EXECUTABLE}" -DCATKIN_ENABLE_TESTING=ON
catkin_make -C "${workspace}" run_tests_xgc2_gazebo_scene
catkin_make -C "${workspace}" run_tests_xgc2_simple_lidar
catkin_test_results "${workspace}/build/test_results"
echo "Package tests passed."
