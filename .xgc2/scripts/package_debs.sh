#!/usr/bin/env bash
set -euo pipefail

OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PACKAGE="ros-${ROS_DISTRO}-xgc2-gazebo-sim-worlds"
ROS_PACKAGE="gazebo_sim_worlds"

product_version() {
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml"
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
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

if [[ -z "${OUTPUT_DIR}" ]]; then
  echo "--output-dir is required" >&2
  exit 1
fi

if [[ -z "${VERSION}" ]]; then
  echo "package version is missing" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}"/*.deb

pkg_root="${BUILD_DIR}/${PACKAGE}"
share_root="${pkg_root}/opt/ros/${ROS_DISTRO}/share/${ROS_PACKAGE}"
lib_root="${pkg_root}/opt/ros/${ROS_DISTRO}/lib"
mkdir -p "${share_root}" "${lib_root}" "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${PACKAGE}"

plugin_library="${OBSTACLE_PATH_PLUGIN_LIBRARY:-}"
if [[ -z "${plugin_library}" ]]; then
  for candidate in \
    "${REPO_ROOT}/devel/lib/libobstaclePathPlugin.so" \
    "${REPO_ROOT}/build/devel/lib/libobstaclePathPlugin.so"; do
    if [[ -f "${candidate}" ]]; then
      plugin_library="${candidate}"
      break
    fi
  done
fi

if [[ -z "${plugin_library}" || ! -f "${plugin_library}" ]]; then
  echo "libobstaclePathPlugin.so is missing; build gazebo_sim_worlds before packaging" >&2
  exit 1
fi

cp -a "${REPO_ROOT}/package.xml" "${share_root}/package.xml"
cp -a "${REPO_ROOT}/worlds" "${share_root}/worlds"
cp -a "${REPO_ROOT}/models" "${share_root}/models"
cp -a "${plugin_library}" "${lib_root}/libobstaclePathPlugin.so"

cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ros-${ROS_DISTRO}-gazebo-ros, libgazebo11
Description: Shared Gazebo Classic world assets for XGC2 simulation products
EOF
printf 'xgc2-gazebo-sim-worlds package\n' > "${pkg_root}/usr/share/doc/${PACKAGE}/README"
chmod 0755 "${pkg_root}/DEBIAN"

fakeroot dpkg-deb --build "${pkg_root}" "${OUTPUT_DIR}/${PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
