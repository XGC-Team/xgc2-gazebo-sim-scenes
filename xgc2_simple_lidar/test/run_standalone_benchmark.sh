#!/usr/bin/env bash
# Build and run scan_projection_benchmark.cpp with only a C++17 compiler and
# the stand-in headers in test/standalone (see run_standalone_test.sh).
# Optional argument: rounds per scene (default 15). Exits nonzero if any
# frame differs from the frozen 1.4.0-2 projection.
set -euo pipefail

package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build="$(mktemp -d)"
trap 'rm -rf "${build}"' EXIT
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror \
  -I"${package}/test/standalone" -I"${package}/include" \
  "${package}/test/scan_projection_benchmark.cpp" -o "${build}/scan_projection_benchmark"
"${build}/scan_projection_benchmark" "$@"
