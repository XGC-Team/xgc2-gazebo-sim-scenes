#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
DOCKER_NETWORK="${DOCKER_NETWORK:-bridge}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image)
      DOCKER_IMAGE="$2"
      shift 2
      ;;
    --network)
      DOCKER_NETWORK="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --skip-install-check)
      INSTALL_CHECK=false
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

docker pull "${DOCKER_IMAGE}"
docker run --rm \
  --network "${DOCKER_NETWORK}" \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -v "${REPO_ROOT}:/workspace/gazebo-sim-worlds:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail

    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      dpkg-dev \
      fakeroot \
      file \
      gazebo11 \
      libeigen3-dev \
      libxml2-utils \
      ripgrep \
      ros-noetic-catkin \
      ros-noetic-gazebo-ros \
      ros-noetic-rospack

    cd /workspace/gazebo-sim-worlds
    .xgc2/scripts/check_package_compliance.sh

    source /opt/ros/noetic/setup.bash
    rm -rf /workspace/work/catkin_ws
    mkdir -p /workspace/work/catkin_ws/src
    ln -s /workspace/gazebo-sim-worlds /workspace/work/catkin_ws/src/gazebo_sim_worlds
    catkin_init_workspace /workspace/work/catkin_ws/src
    cd /workspace/work/catkin_ws
    catkin_make --pkg gazebo_sim_worlds
    plugin_library="$(find /workspace/work/catkin_ws/devel -type f -name libobstaclePathPlugin.so | head -n1)"
    test -f "${plugin_library}"

    cd /workspace/gazebo-sim-worlds
    OBSTACLE_PATH_PLUGIN_LIBRARY="${plugin_library}" .xgc2/scripts/package_debs.sh --output-dir /workspace/out

    dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-sim-worlds_*.deb \
      | grep -F /opt/ros/noetic/share/gazebo_sim_worlds/worlds/empty/empty.world >/dev/null
    dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-sim-worlds_*.deb \
      | grep -F /opt/ros/noetic/lib/libobstaclePathPlugin.so >/dev/null
    dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-sim-worlds_*.deb \
      | grep -F /opt/ros/noetic/share/gazebo_sim_worlds/worlds/clearpath_playpen/clearpath_playpen.world >/dev/null
    dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-sim-worlds_*.deb \
      | grep -F /opt/ros/noetic/share/gazebo_sim_worlds/worlds/corridor_dynamic_9/corridor_dynamic_9.world >/dev/null
    dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-sim-worlds_*.deb \
      | grep -F /opt/ros/noetic/share/gazebo_sim_worlds/models/corridor/model.sdf >/dev/null
    dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-sim-worlds_*.deb \
      | grep -F /opt/ros/noetic/share/gazebo_sim_worlds/models/jersey_barrier/model.sdf >/dev/null

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y /workspace/out/*.deb
      /workspace/gazebo-sim-worlds/.xgc2/scripts/check_installed_packages.sh
    fi
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
