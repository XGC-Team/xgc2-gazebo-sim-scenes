#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"

cd "${repo_root}"
export GAZEBO_MODEL_PATH="${repo_root}/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI:-}"

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

if command -v git >/dev/null && git rev-parse --is-inside-work-tree >/dev/null 2>&1 \
  && git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.ci|\.work|debs)(/|$)' >/dev/null; then
  echo "Generated build artifacts are tracked." >&2
  git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.ci|\.work|debs)(/|$)' >&2
  exit 1
fi

required_files=(
  .github/workflows/ci.yml
  .github/workflows/release.yml
  .xgc2/product.yml
  .xgc2/scripts/build_debs_in_docker.sh
  .xgc2/scripts/check_installed_packages.sh
  .xgc2/scripts/check_package_compliance.sh
  .xgc2/scripts/check_version_bump.sh
  .xgc2/scripts/package_debs.sh
  CMakeLists.txt
  package.xml
  include/gazebo_sim_worlds/obstaclePathPlugin.hh
  src/obstaclePathPlugin.cc
  worlds/empty/empty.world
  worlds/clearpath_playpen/clearpath_playpen.world
  worlds/corridor_dynamic_9/corridor_dynamic_9.world
  models/corridor/model.config
  models/corridor/model.sdf
  models/person/model.config
  models/person/model.sdf
  models/jersey_barrier/model.config
  models/jersey_barrier/model.sdf
)

for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    echo "Missing required file: ${file}" >&2
    exit 1
  fi
done

xmllint --noout package.xml

while IFS= read -r xml_file; do
  xmllint --noout "${xml_file}"
done < <(
  {
    find worlds -type f -name '*.world'
    find models -type f \( -name 'model.config' -o -name 'model.sdf' \)
  } | sort
)

while IFS= read -r scene_dir; do
  scene_name="$(basename "${scene_dir}")"
  if [[ ! -f "${scene_dir}/${scene_name}.world" ]]; then
    echo "Scene directory must include same-name world file: ${scene_dir}/${scene_name}.world" >&2
    exit 1
  fi
  if [[ ! -f "${scene_dir}/README.md" ]]; then
    echo "Scene directory must include README.md: ${scene_dir}/README.md" >&2
    exit 1
  fi
  if ! grep -Fq "[\`${scene_name}\`](worlds/${scene_name}/README.md)" README.md; then
    echo "Root README catalog does not reference scene README: ${scene_name}" >&2
    exit 1
  fi
  if [[ -f "${scene_dir}/preview.png" ]]; then
    if [[ "$(od -An -tx1 -N8 "${scene_dir}/preview.png" | tr -d ' \n')" != "89504e470d0a1a0a" ]]; then
      echo "Scene preview is not a PNG: ${scene_dir}/preview.png" >&2
      exit 1
    fi
    if ! grep -Fq '](preview.png)' "${scene_dir}/README.md"; then
      echo "Scene README does not embed its companion preview: ${scene_dir}/README.md" >&2
      exit 1
    fi
  fi
done < <(find worlds -mindepth 1 -maxdepth 1 -type d | sort)

while IFS= read -r world; do
  if [[ "$(head -n 1 "${world}")" != '<?xml version="1.0"?>' ]]; then
    echo "World file must start with XML declaration: ${world}" >&2
    exit 1
  fi
done < <(find worlds -type f -name '*.world' | sort)

while IFS= read -r sdf_file; do
  echo "Validating SDF: ${sdf_file}"
  if ! timeout 60s gz sdf -k "${sdf_file}" >/tmp/xgc2-gazebo-sim-worlds-sdf-check.log 2>&1; then
    echo "Gazebo SDF validation failed: ${sdf_file}" >&2
    cat /tmp/xgc2-gazebo-sim-worlds-sdf-check.log >&2
    exit 1
  fi
done < <(
  {
    find worlds -type f -name '*.world'
    find models -type f -name 'model.sdf'
  } | sort
)

while IFS= read -r model_dir; do
  if [[ ! -f "${model_dir}/model.config" ]]; then
    echo "Model directory missing model.config: ${model_dir}" >&2
    exit 1
  fi
  if [[ ! -f "${model_dir}/model.sdf" ]]; then
    echo "Model directory missing model.sdf: ${model_dir}" >&2
    exit 1
  fi
done < <(find models -mindepth 1 -maxdepth 1 -type d | sort)

missing_models="$(
  {
    rg -o 'model://[A-Za-z0-9_.-]+' worlds models -S | sed 's#.*model://##'
  } | sort -u | while IFS= read -r model_name; do
    case "${model_name}" in
      ''|ground_plane|sun)
        continue
        ;;
    esac
    if [[ ! -d "models/${model_name}" ]]; then
      printf '%s\n' "${model_name}"
    fi
  done
)"
if [[ -n "${missing_models}" ]]; then
  echo "World/model asset references are missing shared models:" >&2
  echo "${missing_models}" >&2
  exit 1
fi

if rg -n '<node|<include file=' worlds >/dev/null; then
  echo "gazebo_sim_worlds must remain a pure world asset package." >&2
  exit 1
fi

echo "Package compliance checks passed."
