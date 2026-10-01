#!/usr/bin/env bash
# Build and run scan_projection_test.cpp with only a C++17 compiler: the
# headers in test/standalone stand in for GoogleTest, ignition-math,
# sensor_msgs and ros::Time. The catkin build runs the same file with the
# real libraries.
set -euo pipefail

package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build="$(mktemp -d)"
trap 'rm -rf "${build}"' EXIT
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror \
  -I"${package}/test/standalone" -I"${package}/include" \
  "${package}/test/scan_projection_test.cpp" -o "${build}/scan_projection_test"
"${build}/scan_projection_test"
