#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
ros_distro="${ROS_DISTRO:-noetic}"
workspace="${CPP_QUALITY_WORK_DIR:-${repo_root}/.ci/cpp-quality}"

for tool in catkin_make clang-format clang-tidy rsync; do
  if ! command -v "${tool}" >/dev/null 2>&1; then
    echo "Missing required C++ quality tool: ${tool}" >&2
    exit 1
  fi
done

if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "Missing ROS setup: /opt/ros/${ros_distro}/setup.bash" >&2
  exit 1
fi

mapfile -t cpp_files < <(
  find "${repo_root}/xgc2_gazebo_scene" -type f \
    \( -name '*.cpp' -o -name '*.cc' -o -name '*.cxx' -o -name '*.h' -o -name '*.hpp' \) \
    -print | sort
)
if [[ ${#cpp_files[@]} -eq 0 ]]; then
  echo "No C++ files found under xgc2_gazebo_scene" >&2
  exit 1
fi

clang-format -n -Werror "${cpp_files[@]}"

rm -rf "${workspace}"
mkdir -p "${workspace}/src/xgc2_gazebo_scene"
rsync -a --delete \
  "${repo_root}/xgc2_gazebo_scene/" \
  "${workspace}/src/xgc2_gazebo_scene/"
cp "${repo_root}/.clang-tidy" "${workspace}/src/.clang-tidy"

# shellcheck source=/dev/null
source "/opt/ros/${ros_distro}/setup.bash"
catkin_make -C "${workspace}" \
  -DCATKIN_ENABLE_TESTING=ON \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

compile_db="${workspace}/build/compile_commands.json"
if [[ ! -f "${compile_db}" ]]; then
  echo "Compile database was not generated: ${compile_db}" >&2
  exit 1
fi

mapfile -t tidy_files < <(
  find "${workspace}/src/xgc2_gazebo_scene" -type f \
    \( -name '*.cpp' -o -name '*.cc' -o -name '*.cxx' \) -print | sort
)
for file in "${tidy_files[@]}"; do
  clang-tidy -p "${workspace}/build" \
    -header-filter="${workspace}/src/xgc2_gazebo_scene/(include|src|test)/.*" \
    "${file}"
done

echo "C++ quality checks passed."
