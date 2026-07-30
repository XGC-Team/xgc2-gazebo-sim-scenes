#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

product_version() {
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml"
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}"/*.deb

copy_path() {
  local src="$1"
  local dst_root="$2"
  if [[ -e "${src}" ]]; then
    mkdir -p "${dst_root}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${dst_root}${src#${INSTALL_ROOT}}"
  fi
}

write_control() {
  local pkg_root="$1"
  local package="$2"
  local depends="$3"
  local description="$4"

  mkdir -p "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${package}"
  {
    cat <<EOF
Package: ${package}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
EOF
    if [[ -n "${depends}" ]]; then
      printf 'Depends: %s\n' "${depends}"
    fi
    cat <<EOF
Description: ${description}
EOF
  } > "${pkg_root}/DEBIAN/control"
  printf '%s package\n' "${package}" > "${pkg_root}/usr/share/doc/${package}/README"
  chmod 0755 "${pkg_root}/DEBIAN"
}

build_worlds_deb() {
  local package="ros-${ROS_DISTRO}-xgc2-gazebo-sim-worlds"
  local pkg_root="${BUILD_DIR}/${package}"

  mkdir -p "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/gazebo_sim_worlds" "${pkg_root}"
  test -f "${pkg_root}${PREFIX}/share/gazebo_sim_worlds/package.xml"
  test -f "${pkg_root}${PREFIX}/share/gazebo_sim_worlds/worlds/empty/empty.world"
  test -f "${pkg_root}${PREFIX}/share/gazebo_sim_worlds/models/corridor/model.sdf"
  write_control \
    "${pkg_root}" \
    "${package}" \
    "" \
    "Reusable Gazebo Classic world and model assets for XGC2"
  fakeroot dpkg-deb --build \
    "${pkg_root}" \
    "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

build_scene_deb() {
  local package="ros-${ROS_DISTRO}-xgc2-gazebo-scene"
  local pkg_root="${BUILD_DIR}/${package}"
  local path_plugin="${pkg_root}${PREFIX}/lib/libobstaclePathPlugin.so"
  local contact_library="${pkg_root}${PREFIX}/lib/libxgc2_gazebo_scene_contact.so"
  local geometry_library="${pkg_root}${PREFIX}/lib/libxgc2_gazebo_scene_geometry.so"
  local motion_library="${pkg_root}${PREFIX}/lib/libxgc2_gazebo_scene_motion.so"
  local system_plugin="${pkg_root}${PREFIX}/lib/libxgc2_gazebo_scene_system.so"
  local shlibdeps_output
  local shlibdeps
  local shlibdeps_stderr="${BUILD_DIR}/dpkg-shlibdeps.stderr"
  local unexpected_stderr="${BUILD_DIR}/dpkg-shlibdeps-unexpected.stderr"

  mkdir -p "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/xgc2_gazebo_scene" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/include/xgc2_gazebo_scene" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/libobstaclePathPlugin.so" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/libxgc2_gazebo_scene_contact.so" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/libxgc2_gazebo_scene_geometry.so" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/libxgc2_gazebo_scene_motion.so" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/libxgc2_gazebo_scene_system.so" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/pkgconfig/xgc2_gazebo_scene.pc" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/python3/dist-packages/xgc2_gazebo_scene" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/common-lisp/ros/xgc2_gazebo_scene" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/gennodejs/ros/xgc2_gazebo_scene" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/roseus/ros/xgc2_gazebo_scene" "${pkg_root}"

  test -f "${pkg_root}${PREFIX}/share/xgc2_gazebo_scene/package.xml"
  test -f "${pkg_root}${PREFIX}/share/xgc2_gazebo_scene/msg/ObstacleDefinition.msg"
  test -f "${pkg_root}${PREFIX}/share/xgc2_gazebo_scene/srv/ConfigureMotions.srv"
  test -f "${pkg_root}${PREFIX}/include/xgc2_gazebo_scene/ObstacleDefinition.h"
  test -f "${pkg_root}${PREFIX}/include/xgc2_gazebo_scene/obstacle_path_plugin.hpp"
  test -f "${pkg_root}${PREFIX}/lib/pkgconfig/xgc2_gazebo_scene.pc"
  test -f "${pkg_root}${PREFIX}/lib/python3/dist-packages/xgc2_gazebo_scene/msg/_ObstacleDefinition.py"
  test -f "${path_plugin}"
  test -f "${contact_library}"
  test -f "${geometry_library}"
  test -f "${motion_library}"
  test -f "${system_plugin}"

  find "${pkg_root}${PREFIX}/lib/python3/dist-packages/xgc2_gazebo_scene" \
    -type d -name __pycache__ -prune -exec rm -rf {} +

  mkdir -p "${BUILD_DIR}/debian"
  cat > "${BUILD_DIR}/debian/control" <<EOF
Source: xgc2-gazebo-sim-scenes
Section: misc
Priority: optional
Maintainer: XGC2 <apt@example.com>

Package: ${package}
Architecture: any
EOF
  shlibdeps_output="$(
    cd "${BUILD_DIR}"
    dpkg-shlibdeps \
      -O \
      "-l${pkg_root}${PREFIX}/lib" \
      "-e${path_plugin}" \
      "-e${contact_library}" \
      "-e${geometry_library}" \
      "-e${motion_library}" \
      "-e${system_plugin}" \
      2>"${shlibdeps_stderr}"
  )"
  grep -Ev \
    "^dpkg-shlibdeps: warning: can't extract name and version from library name '(libxgc2_gazebo_scene_contact|libxgc2_gazebo_scene_geometry|libxgc2_gazebo_scene_motion|libroscpp|librosconsole|libroscpp_serialization|librostime)\\.so'$|^dpkg-shlibdeps: warning: binaries to analyze should already be installed in their package's directory$" \
    "${shlibdeps_stderr}" >"${unexpected_stderr}" || true
  if [[ -s "${unexpected_stderr}" ]]; then
    echo "dpkg-shlibdeps emitted an unexpected warning:" >&2
    cat "${unexpected_stderr}" >&2
    exit 1
  fi
  shlibdeps="${shlibdeps_output#shlibs:Depends=}"
  if [[ "${shlibdeps}" == "${shlibdeps_output}" || -z "${shlibdeps}" ]]; then
    echo "dpkg-shlibdeps did not produce scene runtime dependencies" >&2
    exit 1
  fi
  if ! grep -Eq '(^|, )libgazebo11( |[(])' <<<"${shlibdeps}"; then
    echo "Scene runtime dependencies do not include libgazebo11" >&2
    exit 1
  fi
  if grep -Eq '(^|, )(cmake|gazebo-dev|libgazebo11-dev|libeigen3-dev|ros-noetic-message-generation)( |[(,]|$)' \
      <<<"${shlibdeps}"; then
    echo "Scene runtime dependencies leaked a build-only package" >&2
    exit 1
  fi

  write_control \
    "${pkg_root}" \
    "${package}" \
    "${shlibdeps}, ros-noetic-gazebo-ros, ros-noetic-geometry-msgs, ros-noetic-message-runtime, ros-noetic-rosconsole, ros-noetic-roscpp, ros-noetic-roscpp-serialization, ros-noetic-rostime, ros-noetic-std-msgs" \
    "XGC2 Gazebo Classic scene director and obstacle controllers"
  find "${pkg_root}" -type d -exec chmod 0755 {} +
  find "${pkg_root}" -type f -exec chmod 0644 {} +

  if readelf -d \
      "${path_plugin}" \
      "${contact_library}" \
      "${geometry_library}" \
      "${motion_library}" \
      "${system_plugin}" | grep -Eq '(RPATH|RUNPATH)'; then
    echo "Scene libraries contain a build-time RPATH/RUNPATH" >&2
    exit 1
  fi
  for plugin in "${path_plugin}" "${system_plugin}"; do
    if ! nm -D --defined-only "${plugin}" | grep -E '[[:space:]]RegisterPlugin$' >/dev/null; then
      echo "Gazebo plugin does not export RegisterPlugin: ${plugin}" >&2
      exit 1
    fi
  done
  for library in "${path_plugin}" "${system_plugin}"; do
    if LD_LIBRARY_PATH="${pkg_root}${PREFIX}/lib:${PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
        ldd "${library}" | grep -q 'not found'; then
      echo "Scene library has unresolved shared libraries: ${library}" >&2
      exit 1
    fi
  done

  fakeroot dpkg-deb --build \
    "${pkg_root}" \
    "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

build_worlds_deb
build_scene_deb

find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
