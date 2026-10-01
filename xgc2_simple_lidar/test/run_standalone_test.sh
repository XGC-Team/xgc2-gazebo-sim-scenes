#!/usr/bin/env bash
# Build and run the ROS-free unit tests (scan projection and CPU scan
# schedule) with only a C++17 compiler: the headers in test/standalone stand
# in for GoogleTest, ignition-math, sensor_msgs and ros::Time. The catkin
# build runs the same files with the real libraries.
set -euo pipefail

package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build="$(mktemp -d)"
trap 'rm -rf "${build}"' EXIT
for test in scan_projection_test scan_schedule_test; do
  "${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror \
    -I"${package}/test/standalone" -I"${package}/include" \
    "${package}/test/${test}.cpp" -o "${build}/${test}"
  "${build}/${test}"
done
