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
  copy_path "${PREFIX_ROOT}/lib/gazebo_sim_worlds" "${pkg_root}"
  test -x "${pkg_root}${PREFIX}/lib/gazebo_sim_worlds/native_world_start"
  test -f "${pkg_root}${PREFIX}/share/gazebo_sim_worlds/launch/native_world.launch"
  test -f "${pkg_root}${PREFIX}/share/gazebo_sim_worlds/package.xml"
  test -f "${pkg_root}${PREFIX}/share/gazebo_sim_worlds/worlds/empty/empty.world"
  test -f "${pkg_root}${PREFIX}/share/gazebo_sim_worlds/worlds/catalog/scene_editable.world"
  test -f "${pkg_root}${PREFIX}/share/gazebo_sim_worlds/models/corridor/model.sdf"
  write_control \
    "${pkg_root}" \
    "${package}" \
    "ros-${ROS_DISTRO}-xgc2-gazebo-scene (= ${VERSION})" \
    "Reusable Gazebo Classic world and model assets for XGC2"
  fakeroot dpkg-deb --build \
    "${pkg_root}" \
    "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

build_scene_deb() {
  local package="ros-${ROS_DISTRO}-xgc2-gazebo-scene"
  local pkg_root="${BUILD_DIR}/${package}"
  local shlibdeps_output shlibdeps
  local shlibdeps_stderr="${BUILD_DIR}/scene-shlibdeps.stderr"
  local unexpected_stderr="${BUILD_DIR}/scene-shlibdeps-unexpected.stderr"
  local libraries=(
    libobstaclePathPlugin.so libxgc2_gazebo_scene_contact.so
    libxgc2_gazebo_scene_geometry.so libxgc2_gazebo_scene_motion.so
    libxgc2_gazebo_scene_system.so libxgc2_scene_model.so
    libxgc2_simulation_service.so libxgc2_simulation_world.so
    libxgc2_simulation_entity_ack.so libxgc2_simulation_sensor_ack.so
    libxgc2_simulation_ros_data.so
  )
  local plugins=(libobstaclePathPlugin.so libxgc2_gazebo_scene_system.so
    libxgc2_simulation_world.so libxgc2_simulation_entity_ack.so
    libxgc2_simulation_sensor_ack.so libxgc2_simulation_ros_data.so)
  local objects=() arguments=(-O "-x${package}" "-l${pkg_root}${PREFIX}/lib")
  mkdir -p "${pkg_root}"
  for directory in share/xgc2_gazebo_scene include/xgc2_gazebo_scene lib/xgc2_gazebo_scene \
      lib/python3/dist-packages/xgc2_gazebo_scene share/common-lisp/ros/xgc2_gazebo_scene \
      share/gennodejs/ros/xgc2_gazebo_scene share/roseus/ros/xgc2_gazebo_scene; do
    copy_path "${PREFIX_ROOT}/${directory}" "${pkg_root}"
  done
  copy_path "${PREFIX_ROOT}/lib/pkgconfig/xgc2_gazebo_scene.pc" "${pkg_root}"
  for library in "${libraries[@]}"; do
    copy_path "${PREFIX_ROOT}/lib/${library}" "${pkg_root}"
    test -f "${pkg_root}${PREFIX}/lib/${library}"
    objects+=("${pkg_root}${PREFIX}/lib/${library}")
    arguments+=("-e${pkg_root}${PREFIX}/lib/${library}")
  done
  for file in share/xgc2_gazebo_scene/package.xml share/xgc2_gazebo_scene/msg/ObstacleDefinition.msg \
      include/xgc2_gazebo_scene/ObstacleDefinition.h include/xgc2_gazebo_scene/simulation_service.hpp \
      include/xgc2_gazebo_scene/native_scene_controller.hpp \
      lib/python3/dist-packages/xgc2_gazebo_scene/msg/_ObstacleDefinition.py; do
    test -f "${pkg_root}${PREFIX}/${file}"
  done
  for retired in share/xgc2_gazebo_scene/srv/ConfigureMotions.srv \
      share/xgc2_gazebo_scene/srv/StopMotions.srv lib/libxgc2_scene_authoring_world.so; do
    if [[ -e "${PREFIX_ROOT}/${retired}" ]]; then
      echo "Retired platform control remains in install root: ${retired}" >&2
      exit 1
    fi
  done
  test -x "${pkg_root}${PREFIX}/lib/xgc2_gazebo_scene/spawn_robot_model"
  test -x "${pkg_root}${PREFIX}/lib/xgc2_gazebo_scene/simulation_world_prepare"
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
  # Analyze every real native DSO. Missing third-party or SDK package metadata
  # fails the build; no guessed dependency list substitutes for this evidence.
  shlibdeps_output="$(cd "${BUILD_DIR}"; dpkg-shlibdeps "${arguments[@]}" 2>"${shlibdeps_stderr}")"
  grep -Ev \
    "^dpkg-shlibdeps: warning: can't extract name and version from library name '(libxgc2_gazebo_scene_contact|libxgc2_gazebo_scene_geometry|libxgc2_gazebo_scene_motion|libxgc2_scene_model|libxgc2_simulation_service|libroscpp|librosconsole|libroscpp_serialization|librostime)\\.so'$|^dpkg-shlibdeps: warning: binaries to analyze should already be installed in their package's directory$" \
    "${shlibdeps_stderr}" >"${unexpected_stderr}" || true
  if [[ -s "${unexpected_stderr}" ]]; then
    cat "${unexpected_stderr}" >&2
    exit 1
  fi
  shlibdeps="${shlibdeps_output#shlibs:Depends=}"
  if [[ "${shlibdeps}" == "${shlibdeps_output}" || -z "${shlibdeps}" ]]; then
    echo "dpkg-shlibdeps did not produce native scene dependencies" >&2; exit 1
  fi
  for dependency in libgazebo11 libxgc2-xrpc1; do
    if ! grep -Eq "(^|, )${dependency}([[:space:]]|[(]|,|$)" <<<"${shlibdeps}"; then
      echo "Native scene dependency evidence lacks ${dependency}" >&2; exit 1
    fi
  done
  if grep -Eq '(^|, )(cmake|gazebo-dev|libgazebo11-dev|libeigen3-dev|libjsoncpp-dev|libxgc2-xrpc-dev|libxgc2-robotics-interfaces-dev|ros-noetic-message-generation)( |[(,]|$)' <<<"${shlibdeps}"; then
    echo "Native scene runtime dependencies leaked build-only packages" >&2; exit 1
  fi
  write_control "${pkg_root}" "${package}" \
    "${shlibdeps}, libxgc2-xrpc1 (>= 0.1.0-1), ros-noetic-gazebo-msgs, ros-noetic-geometry-msgs, ros-noetic-message-runtime, ros-noetic-roscpp, ros-noetic-rospy, ros-noetic-rosgraph-msgs, ros-noetic-std-msgs, ros-noetic-xgc2-geometry-msgs (>= 1.2.0-12), ros-noetic-xgc2-scene-runtime (>= 1.2.0-12)" \
    "Native Gazebo Classic simulation-v1 service and exact scene geometry with ROS user data"
  find "${pkg_root}" -type d -exec chmod 0755 {} +
  find "${pkg_root}" -type f -exec chmod 0644 {} +
  chmod 0755 "${pkg_root}${PREFIX}/lib/xgc2_gazebo_scene/spawn_robot_model" \
    "${pkg_root}${PREFIX}/lib/xgc2_gazebo_scene/simulation_world_prepare"
  if readelf -d "${objects[@]}" | grep -Eq '(RPATH|RUNPATH)'; then
    echo "Native scene libraries contain build-time RPATH/RUNPATH" >&2; exit 1
  fi
  for plugin in "${plugins[@]}"; do
    if ! nm -D --defined-only "${pkg_root}${PREFIX}/lib/${plugin}" | grep -E '[[:space:]]RegisterPlugin$' >/dev/null; then
      echo "Gazebo plugin lacks RegisterPlugin: ${plugin}" >&2; exit 1
    fi
  done
  if LD_LIBRARY_PATH="${pkg_root}${PREFIX}/lib:${PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
      ldd "${objects[@]}" | grep -q 'not found'; then
    echo "Native scene libraries have unresolved dependencies" >&2; exit 1
  fi
  fakeroot dpkg-deb --build "${pkg_root}" "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

build_lidar_deb() {
  local package="ros-${ROS_DISTRO}-xgc2-simple-lidar"
  local pkg_root="${BUILD_DIR}/${package}"
  local lidar_library="${pkg_root}${PREFIX}/lib/libxgc2_simple_lidar.so"
  local cpu_library="${pkg_root}${PREFIX}/lib/libxgc2_simple_lidar_cpu.so"
  local shlibdeps_output
  local shlibdeps
  local shlibdeps_stderr="${BUILD_DIR}/lidar-dpkg-shlibdeps.stderr"
  local unexpected_stderr="${BUILD_DIR}/lidar-dpkg-shlibdeps-unexpected.stderr"

  mkdir -p "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/xgc2_simple_lidar" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/include/xgc2_simple_lidar" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/libxgc2_simple_lidar.so" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/libxgc2_simple_lidar_cpu.so" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/pkgconfig/xgc2_simple_lidar.pc" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/xgc2_simple_lidar" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/python3/dist-packages/xgc2_simple_lidar" "${pkg_root}"

  test -f "${pkg_root}${PREFIX}/share/xgc2_simple_lidar/package.xml"
  test -f "${pkg_root}${PREFIX}/share/xgc2_simple_lidar/models/sensor.xacro"
  test -f "${pkg_root}${PREFIX}/share/xgc2_simple_lidar/models/sensor.sdf.xacro"
  test -f "${pkg_root}${PREFIX}/share/xgc2_simple_lidar/cmake/xgc2_simple_lidarConfig.cmake"
  test -f "${pkg_root}${PREFIX}/include/xgc2_simple_lidar/scan_projection.hpp"
  test -f "${pkg_root}${PREFIX}/lib/pkgconfig/xgc2_simple_lidar.pc"
  test -f "${lidar_library}"
  test -f "${cpu_library}"
  test -x "${pkg_root}${PREFIX}/lib/xgc2_simple_lidar/launch_robot.py"
  test -f "${pkg_root}${PREFIX}/lib/python3/dist-packages/xgc2_simple_lidar/configuration.py"

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
      "-e${lidar_library}" \
      "-e${cpu_library}" \
      2>"${shlibdeps_stderr}"
  )"
  grep -Ev \
    "^dpkg-shlibdeps: warning: can't extract name and version from library name '(libxgc2_simple_lidar|libxgc2_simple_lidar_cpu|libroscpp|librosconsole|libroscpp_serialization|librostime)\\.so'$|^dpkg-shlibdeps: warning: binaries to analyze should already be installed in their package's directory$" \
    "${shlibdeps_stderr}" >"${unexpected_stderr}" || true
  if [[ -s "${unexpected_stderr}" ]]; then
    echo "dpkg-shlibdeps emitted an unexpected lidar warning:" >&2
    cat "${unexpected_stderr}" >&2
    exit 1
  fi
  shlibdeps="${shlibdeps_output#shlibs:Depends=}"
  if [[ "${shlibdeps}" == "${shlibdeps_output}" || -z "${shlibdeps}" ]]; then
    echo "dpkg-shlibdeps did not produce lidar runtime dependencies" >&2
    exit 1
  fi
  if ! grep -Eq '(^|, )libgazebo11([[:space:]]|[(]|,|$)' <<<"${shlibdeps}"; then
    echo "Lidar runtime dependencies do not include libgazebo11" >&2
    exit 1
  fi
  if grep -Eq '(^|, )(cmake|catkin|gazebo-dev|libgazebo11-dev|libeigen3-dev|ros-noetic-message-generation)( |[(,]|$)' \
      <<<"${shlibdeps}"; then
    echo "Lidar runtime dependencies leaked a build-only package" >&2
    exit 1
  fi

  write_control \
    "${pkg_root}" \
    "${package}" \
    "${shlibdeps}, python3, ros-${ROS_DISTRO}-roslaunch, ros-${ROS_DISTRO}-gazebo-ros, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-sensor-msgs, ros-${ROS_DISTRO}-xacro" \
    "CPU/GPU ray simple lidar plugins and reusable sensor xacro for XGC2"
  find "${pkg_root}" -type d -exec chmod 0755 {} +
  find "${pkg_root}" -type f -exec chmod 0644 {} +
  chmod 0755 "${pkg_root}${PREFIX}/lib/xgc2_simple_lidar/launch_robot.py"

  if readelf -d "${lidar_library}" "${cpu_library}" | grep -Eq '(RPATH|RUNPATH)'; then
    echo "Lidar library contains a build-time RPATH/RUNPATH" >&2
    exit 1
  fi
  if LD_LIBRARY_PATH="${pkg_root}${PREFIX}/lib:${PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
      ldd "${lidar_library}" "${cpu_library}" | grep -q 'not found'; then
    echo "Lidar library has unresolved shared libraries" >&2
    exit 1
  fi

  fakeroot dpkg-deb --build \
    "${pkg_root}" \
    "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

build_rendering_deb() {
  local package="ros-${ROS_DISTRO}-xgc2-gazebo-rendering"
  local pkg_root="${BUILD_DIR}/${package}"
  local library="${pkg_root}${PREFIX}/lib/libxgc2_gazebo_rendering.so"
  local shlibdeps_output
  local shlibdeps
  local shlibdeps_stderr="${BUILD_DIR}/rendering-dpkg-shlibdeps.stderr"
  local unexpected_stderr="${BUILD_DIR}/rendering-dpkg-shlibdeps-unexpected.stderr"

  mkdir -p "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/xgc2_gazebo_rendering" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/libxgc2_gazebo_rendering.so" "${pkg_root}"
  test -f "${pkg_root}${PREFIX}/share/xgc2_gazebo_rendering/package.xml"
  test -f "${library}"

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
    dpkg-shlibdeps -O "-l${pkg_root}${PREFIX}/lib" "-e${library}" 2>"${shlibdeps_stderr}"
  )"
  grep -Ev \
    "^dpkg-shlibdeps: warning: can't extract name and version from library name 'libxgc2_gazebo_rendering\\.so'$|^dpkg-shlibdeps: warning: binaries to analyze should already be installed in their package's directory$" \
    "${shlibdeps_stderr}" >"${unexpected_stderr}" || true
  if [[ -s "${unexpected_stderr}" ]]; then
    echo "dpkg-shlibdeps emitted an unexpected rendering warning:" >&2
    cat "${unexpected_stderr}" >&2
    exit 1
  fi
  shlibdeps="${shlibdeps_output#shlibs:Depends=}"
  if [[ "${shlibdeps}" == "${shlibdeps_output}" || -z "${shlibdeps}" ]]; then
    echo "dpkg-shlibdeps did not produce rendering runtime dependencies" >&2
    exit 1
  fi
  if ! grep -Eq '(^|, )libgazebo11([[:space:]]|[(]|,|$)' <<<"${shlibdeps}"; then
    echo "Rendering runtime dependencies do not include libgazebo11" >&2
    exit 1
  fi
  write_control \
    "${pkg_root}" \
    "${package}" \
    "${shlibdeps}" \
    "Gazebo Classic shadow-depth plugin for XGC2"
  find "${pkg_root}" -type d -exec chmod 0755 {} +
  find "${pkg_root}" -type f -exec chmod 0644 {} +
  chmod 0755 "${library}"
  if readelf -d "${library}" | grep -Eq '(RPATH|RUNPATH)'; then
    echo "Rendering library contains a build-time RPATH/RUNPATH" >&2
    exit 1
  fi
  if ! nm -D --defined-only "${library}" | grep -E '[[:space:]]RegisterPlugin$' >/dev/null; then
    echo "Gazebo plugin does not export RegisterPlugin: ${library}" >&2
    exit 1
  fi
  fakeroot dpkg-deb --build \
    "${pkg_root}" \
    "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

build_worlds_deb
build_scene_deb
build_lidar_deb
build_rendering_deb

find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
