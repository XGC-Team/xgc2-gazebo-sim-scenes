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
  -e XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}" \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -v "${REPO_ROOT}:/workspace/gazebo-sim-scenes:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail

    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    apt-get install -y --no-install-recommends ca-certificates curl
    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://xgc2.apt.xiaokang.ink/xgc2-archive-keyring.gpg \
      -o /etc/apt/keyrings/xgc2-archive-keyring.gpg
    chmod 0644 /etc/apt/keyrings/xgc2-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] https://xgc2.apt.xiaokang.ink focal main" \
      > /etc/apt/sources.list.d/xgc2.list
    if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
      sed "s#${XGC2_APT_BASE_URL:-https://xgc2.apt.xiaokang.ink}#${XGC2_APT_OVERLAY_URL%/}#g" \
        /etc/apt/sources.list.d/xgc2.list \
        > /etc/apt/sources.list.d/00-xgc2-release-train.list
    fi
    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      dpkg-dev \
      fakeroot \
      file \
      gazebo11 \
      git \
      libeigen3-dev \
      libgazebo11-dev \
      libxml2-utils \
      netbase \
      ripgrep \
      rsync \
      ros-noetic-catkin \
      ros-noetic-gazebo-msgs \
      ros-noetic-gazebo-ros \
      ros-noetic-geometry-msgs \
      ros-noetic-message-generation \
      ros-noetic-roscpp \
      ros-noetic-roslaunch \
      ros-noetic-rospack \
      ros-noetic-rostest \
      ros-noetic-rosunit \
      ros-noetic-std-msgs \
      ros-noetic-std-srvs \
      ros-noetic-tf2 \
      ros-noetic-tf2-ros \
      ros-noetic-xgc2-geometry-msgs

    cd /workspace/gazebo-sim-scenes
    .xgc2/scripts/check_package_compliance.sh

    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src/xgc2_gazebo_sim_scenes
    rsync -a --delete /workspace/gazebo-sim-scenes/ /workspace/work/src/xgc2_gazebo_sim_scenes/

    cd /workspace/work
    source /opt/ros/noetic/setup.bash
    DESTDIR=/workspace/work/install-root catkin_make install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCATKIN_ENABLE_TESTING=OFF

    /workspace/gazebo-sim-scenes/.xgc2/scripts/package_debs.sh \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out

    dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-sim-worlds_*.deb \
      | grep -F /opt/ros/noetic/share/gazebo_sim_worlds/worlds/empty/empty.world >/dev/null
    dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-sim-worlds_*.deb \
      | grep -F /opt/ros/noetic/share/gazebo_sim_worlds/models/corridor/model.sdf >/dev/null
    if dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-sim-worlds_*.deb \
        | grep -F /opt/ros/noetic/lib/libobstaclePathPlugin.so; then
      echo "World assets package must not own model-control libraries" >&2
      exit 1
    fi

    for library in \
        libobstaclePathPlugin.so \
        libxgc2_gazebo_scene_motion.so \
        libxgc2_gazebo_scene_system.so; do
      dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-scene_*.deb \
        | grep -F "/opt/ros/noetic/lib/${library}" >/dev/null
    done
    dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-scene_*.deb \
      | grep -F /opt/ros/noetic/share/xgc2_gazebo_scene/msg/ObstacleDefinition.msg >/dev/null
    dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-scene_*.deb \
      | grep -F /opt/ros/noetic/lib/python3/dist-packages/xgc2_gazebo_scene/msg/_ObstacleDefinition.py >/dev/null
    dpkg-deb -f /workspace/out/ros-noetic-xgc2-gazebo-scene_*.deb Depends \
      | grep -E "(^|, )libgazebo11( |\\()" >/dev/null
    if dpkg-deb -f /workspace/out/ros-noetic-xgc2-gazebo-scene_*.deb Depends \
        | grep -E "(^|, )(cmake|gazebo-dev|libgazebo11-dev|libeigen3-dev|ros-noetic-message-generation)( |\\(|,|$)"; then
      echo "Scene package leaked a build-only dependency" >&2
      exit 1
    fi

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y --no-install-recommends /workspace/out/*.deb
      /workspace/gazebo-sim-scenes/.xgc2/scripts/check_installed_packages.sh
    fi
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
