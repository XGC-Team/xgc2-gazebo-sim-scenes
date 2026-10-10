#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DOCKER_IMAGE="${DOCKER_IMAGE:-}"
DOCKER_NETWORK="${DOCKER_NETWORK:-bridge}"
WORK_DIR="${WORK_DIR:-}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) DOCKER_IMAGE="$2"; shift 2 ;;
    --network) DOCKER_NETWORK="$2"; shift 2 ;;
    --work-dir) WORK_DIR="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --skip-install-check) INSTALL_CHECK=false; shift ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done
if [[ ! "${DOCKER_IMAGE}" =~ @sha256:[0-9a-f]{64}$ ]]; then
  echo "--image/DOCKER_IMAGE requires the approved immutable Classic/Noetic build image digest" >&2
  exit 1
fi
: "${XGC2_PYTHON_EXECUTABLE:?select the image Python interpreter >=3.8 explicitly}"
[[ "${XGC2_PYTHON_EXECUTABLE}" == /* ]]
if [[ -z "${WORK_DIR}" ]]; then WORK_DIR="$(mktemp -d -t xgc2-scenes-build.XXXXXX)"; fi
mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"
WORK_DIR="$(realpath "${WORK_DIR}")"
case "${WORK_DIR}/" in
  "${REPO_ROOT}/"*) echo "Build work directory must be outside the source/runner workspace" >&2; exit 1 ;;
esac
docker pull "${DOCKER_IMAGE}"
docker run --rm --network "${DOCKER_NETWORK}" \
  -e XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}" \
  -e XGC2_DEPENDENCY_SET_DIGEST="${XGC2_DEPENDENCY_SET_DIGEST:-}" \
  -e XGC2_PYTHON_EXECUTABLE \
  -e DEBIAN_FRONTEND=noninteractive -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
  -v "${REPO_ROOT}:/workspace/gazebo-sim-scenes:ro" \
  -v "${WORK_DIR}:/workspace/work" "${DOCKER_IMAGE}" bash -lc '
    set -euo pipefail
    trap '\''chown -R "${HOST_UID}:${HOST_GID}" /workspace/work'\'' EXIT
    unset DISPLAY WAYLAND_DISPLAY
    export SETUPTOOLS_USE_DISTUTILS=stdlib
    export CC=/usr/bin/clang-10 CXX=/usr/bin/clang++-10
    source /opt/ros/noetic/setup.bash
    test "$(. /etc/os-release; printf "%s" "$VERSION_CODENAME")" = focal
    cmake --version
    "${XGC2_PYTHON_EXECUTABLE}" -c "import sys; assert sys.version_info >= (3, 8)"
    printf "#include <span>\nint main(){int a[1]{};return std::span<int>(a).size()!=1;}\n" > /workspace/work/toolchain.cpp
    "${CXX}" -std=c++20 -fsyntax-only /workspace/work/toolchain.cpp
    # The image supplies third-party tools/official ROS. XGC2 products are
    # resolved here from production or the explicit release-train overlay.
    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://xgc2.apt.xiaokang.ink/xgc2-archive-keyring.gpg -o /etc/apt/keyrings/xgc2-archive-keyring.gpg
    chmod 0644 /etc/apt/keyrings/xgc2-archive-keyring.gpg
    printf "deb [arch=%s signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] https://xgc2.apt.xiaokang.ink focal main\n" "$(dpkg --print-architecture)" > /etc/apt/sources.list.d/xgc2.list
    if [[ -n "${XGC2_APT_OVERLAY_URL}" ]]; then
      [[ "${XGC2_DEPENDENCY_SET_DIGEST}" =~ ^[0-9a-f]{64}$ ]]
      sed -i "s#https://xgc2.apt.xiaokang.ink#${XGC2_APT_OVERLAY_URL%/}#g" /etc/apt/sources.list.d/xgc2.list
    fi
    apt-get update -o Dir::Etc::sourcelist=sources.list.d/xgc2.list -o Dir::Etc::sourceparts="-"
    apt-get install -y --no-install-recommends libxgc2-xrpc-dev libxgc2-chassis-hold-dev \
      ros-noetic-xgc2-geometry-msgs ros-noetic-xgc2-scene-runtime
    for relation in "libxgc2-xrpc-dev 0.1.0-2~focal" "libxgc2-chassis-hold-dev 0.1.0-1~focal" \
      "ros-noetic-xgc2-geometry-msgs 1.2.0-13" "ros-noetic-xgc2-scene-runtime 1.2.0-13"; do
      read -r package floor <<<"${relation}"
      version="$(dpkg-query -W -f '\''${Version}'\'' "${package}")"
      dpkg --compare-versions "${version}" ge "${floor}"
      printf "%s=%s\n" "${package}" "${version}"
    done
    wheel="/workspace/work/xgc2_xrpc-0.1.0-py3-none-any.whl"
    curl -fL https://github.com/XGC-Team/xgc2-xrpc/releases/download/v0.1.0-1/xgc2_xrpc-0.1.0-py3-none-any.whl -o "${wheel}"
    printf "%s  %s\n" 8e505ab2366eed198dcd4343e758fed5b7936990b2a72ba635d73d81b195187c "${wheel}" | sha256sum --check --status
    # Install only the formal first-party wheel; third-party requirements are
    # image-owned and may not be bootstrapped by this product build.
    "${XGC2_PYTHON_EXECUTABLE}" -m pip install --no-deps --no-index "${wheel}"
    "${XGC2_PYTHON_EXECUTABLE}" -c "from xgc2_xrpc.http import Client; import yaml; import rospy"
    cd /workspace/gazebo-sim-scenes
    .xgc2/scripts/check_package_compliance.sh
    CPP_QUALITY_WORK_DIR=/workspace/work/cpp-quality .xgc2/scripts/check_cpp_quality.sh
    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root /workspace/work/debs
    mkdir -p /workspace/work/src/xgc2_gazebo_sim_scenes
    rsync -a --delete --exclude=.git --exclude=.work --exclude=.ci --exclude=debs /workspace/gazebo-sim-scenes/ /workspace/work/src/xgc2_gazebo_sim_scenes/
    cd /workspace/work
    DESTDIR=/workspace/work/install-root catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_C_COMPILER="${CC}" -DCMAKE_CXX_COMPILER="${CXX}" -DPYTHON_EXECUTABLE="${XGC2_PYTHON_EXECUTABLE}" -DCMAKE_SKIP_INSTALL_RPATH=ON -DCATKIN_ENABLE_TESTING=OFF
    /workspace/gazebo-sim-scenes/.xgc2/scripts/package_debs.sh --install-root /workspace/work/install-root --output-dir /workspace/work/debs
    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y --no-install-recommends /workspace/work/debs/*.deb
      /workspace/gazebo-sim-scenes/.xgc2/scripts/check_installed_packages.sh
    fi
  '
# Copy as the invoking identity: no root-owned artifacts in GITHUB_WORKSPACE.
cp "${WORK_DIR}"/debs/*.deb "${OUTPUT_DIR}/"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
