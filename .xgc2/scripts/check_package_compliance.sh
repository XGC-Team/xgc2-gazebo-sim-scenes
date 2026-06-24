#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"

cd "${repo_root}"

bash -n .xgc2/scripts/*.sh

nested_git="$(
  find . \
    -path ./.git -prune -o \
    -path ./.ci -prune -o \
    -path './*/build' -prune -o \
    -path './*/devel' -prune -o \
    -path './*/install' -prune -o \
    -name .git -print
)"
if [[ -n "${nested_git}" ]]; then
  echo "Nested .git directory found." >&2
  echo "${nested_git}" >&2
  exit 1
fi

if git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.ci|\.work|debs)(/|$)' >/dev/null; then
  echo "Generated build artifacts are tracked." >&2
  git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.ci|\.work|debs)(/|$)' >&2
  exit 1
fi

required_files=(
  .github/workflows/build-debs.yml
  .xgc2/product.yml
  .xgc2/scripts/build_debs_in_docker.sh
  .xgc2/scripts/check_installed_packages.sh
  .xgc2/scripts/check_package_compliance.sh
  .xgc2/scripts/check_version_bump.sh
  .xgc2/scripts/package_debs.sh
  .xgc2/scripts/publish_apt_repo.sh
  CMakeLists.txt
  package.xml
  worlds/empty/empty.world
  worlds/weston_robot_empty/weston_robot_empty.world
  worlds/clearpath_playpen/clearpath_playpen.world
)

for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    echo "Missing required file: ${file}" >&2
    exit 1
  fi
done

xmllint --noout \
  package.xml \
  worlds/empty/empty.world \
  worlds/weston_robot_empty/weston_robot_empty.world \
  worlds/clearpath_playpen/clearpath_playpen.world

while IFS= read -r scene_dir; do
  scene_name="$(basename "${scene_dir}")"
  if [[ ! -f "${scene_dir}/${scene_name}.world" ]]; then
    echo "Scene directory must include same-name world file: ${scene_dir}/${scene_name}.world" >&2
    exit 1
  fi
done < <(find worlds -mindepth 1 -maxdepth 1 -type d | sort)

while IFS= read -r world; do
  if [[ "$(head -n 1 "${world}")" != '<?xml version="1.0"?>' ]]; then
    echo "World file must start with XML declaration: ${world}" >&2
    exit 1
  fi
done < <(find worlds -type f -name '*.world' | sort)

if rg -n '<node|<include file=' worlds >/dev/null; then
  echo "gazebo_sim_worlds must remain a pure world asset package." >&2
  exit 1
fi

echo "Package compliance checks passed."
