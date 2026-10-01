#!/usr/bin/env bash
# Build and run obstacle_messages_benchmark.cpp: the 30 Hz obstacle state and
# instance publication of the scene plugin before and after ObstacleMessages,
# on the real message types and ros::serialization, without Gazebo.
#
# Needs a C++17 compiler, the ROS headers and libraries (roscpp_serialization,
# rostime, geometry_msgs, std_msgs), the headers generated from this package's
# and xgc2_geometry_msgs' messages, and the ignition-math headers. Environment:
#   CXX                    compiler (default g++)
#   XGC2_MESSAGE_INCLUDE   directory holding xgc2_gazebo_scene/*.h and
#                          xgc2_geometry_msgs/*.h (default /opt/ros/$ROS_DISTRO/include;
#                          a catkin workspace: <workspace>/devel/include)
#   CXXFLAGS, LDFLAGS      extra flags, e.g. -I and -L for a ROS install elsewhere
set -euo pipefail

package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_prefix="/opt/ros/${ROS_DISTRO:-noetic}"
messages="${XGC2_MESSAGE_INCLUDE:-${ros_prefix}/include}"
math_flags="$(pkg-config --cflags ignition-math6 2>/dev/null || echo -I/usr/include/ignition/math6)"
build="$(mktemp -d)"
trap 'rm -rf "${build}"' EXIT
# shellcheck disable=SC2086
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Wpedantic -I"${package}/include" -I"${messages}" \
  -I"${ros_prefix}/include" ${math_flags} ${CXXFLAGS:-} \
  "${package}/test/obstacle_messages_benchmark.cpp" "${package}/src/motion_controller.cpp" \
  -L"${ros_prefix}/lib" -lroscpp_serialization -lrostime -lcpp_common ${LDFLAGS:-} \
  -Wl,-rpath,"${ros_prefix}/lib" -o "${build}/obstacle_messages_benchmark"
"${build}/obstacle_messages_benchmark"
