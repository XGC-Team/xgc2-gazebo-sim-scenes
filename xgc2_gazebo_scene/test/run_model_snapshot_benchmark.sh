#!/usr/bin/env bash
# Build and run model_snapshot_benchmark.cpp with only a C++17 compiler: the
# per-update model bookkeeping of the scene plugin before and after the
# model-list snapshot, without Gazebo.
set -euo pipefail

package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build="$(mktemp -d)"
trap 'rm -rf "${build}"' EXIT
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror -I"${package}/include" \
  "${package}/test/model_snapshot_benchmark.cpp" -o "${build}/model_snapshot_benchmark"
"${build}/model_snapshot_benchmark"
