#!/usr/bin/env bash
# Build and run sdf_parser_benchmark.cpp: the SDF parsing of the scene adapter
# per obstacle, before and after ModelSdfParser, with libsdformat and no Gazebo.
#
# Needs a C++17 compiler and libsdformat (9) with its headers. Environment:
#   CXX                compiler (default g++)
#   CXXFLAGS, LDFLAGS  extra flags placed before and after the source file,
#                      e.g. -I and -L/-l for a libsdformat pkg-config cannot find
set -euo pipefail

package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sdformat_cflags="$(pkg-config --cflags sdformat9 2>/dev/null || echo -I/usr/include/sdformat-9.10)"
sdformat_libs="$(pkg-config --libs sdformat9 2>/dev/null || echo -lsdformat9)"
build="$(mktemp -d)"
trap 'rm -rf "${build}"' EXIT
# shellcheck disable=SC2086
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Wpedantic -I"${package}/include" -I"${package}/test" \
  ${sdformat_cflags} ${CXXFLAGS:-} "${package}/test/sdf_parser_benchmark.cpp" ${sdformat_libs} ${LDFLAGS:-} \
  -o "${build}/sdf_parser_benchmark"
"${build}/sdf_parser_benchmark"
