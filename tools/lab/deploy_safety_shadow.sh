#!/usr/bin/env bash
# Build and apply a small, reversible ROS overlay for safety-supervisor SHADOW
# mode. It does not touch STM32 firmware, map volumes, or serial devices.
set -euo pipefail

source_dir="${MOWGLI_SAFETY_SOURCE:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)}"
runtime_dir="${MOWGLI_RUNTIME_DIR:-$source_dir}"
overlay_dir="$runtime_dir/docker/safety-overlay"
compose_file="$runtime_dir/docker/docker-compose.yaml"
override_file="$source_dir/tools/lab/safety-shadow-compose.yaml"
env_file="$runtime_dir/docker/.env"

[[ -f "$env_file" ]] || { echo "Missing $env_file" >&2; exit 1; }
[[ -f "$source_dir/ros2/src/mowgli_safety/package.xml" ]] || { echo "mowgli_safety source missing in $source_dir" >&2; exit 1; }
[[ -f "$runtime_dir/docker/stack.sh" ]] || { echo "Mowgli runtime checkout missing in $runtime_dir" >&2; exit 1; }

set -a
source "$env_file"
set +a
: "${COMPOSE_PROJECT_NAME:=install}"
: "${MOWGLI_ROS2_IMAGE:?MOWGLI_ROS2_IMAGE is required in docker/.env}"

case "${1:-apply}" in
  apply)
    "$runtime_dir/docker/stack.sh" regen
    if [[ -e "$overlay_dir" ]] && [[ -n "$(find "$overlay_dir" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
      echo "Refusing to overwrite existing overlay: $overlay_dir" >&2
      echo "Run '$0 rollback' first, then archive or inspect that directory manually." >&2
      exit 1
    fi
    mkdir -p "$overlay_dir"
    docker run --rm --user "$(id -u):$(id -g)" \
      -e HOME=/tmp/mowgli-safety-home -e ROS_HOME=/tmp/mowgli-safety-home/.ros \
      -v "$source_dir/ros2/src:/src:ro" -v "$overlay_dir:/overlay" \
      "$MOWGLI_ROS2_IMAGE" bash -lc '
        source /opt/ros/kilted/setup.bash
        source /ros2_ws/install/setup.bash
        mkdir -p /tmp/ws/src
        ln -s /src/mowgli_safety /tmp/ws/src/mowgli_safety
        ln -s /src/mowgli_bringup /tmp/ws/src/mowgli_bringup
        cd /tmp/ws
        colcon --log-base /tmp/mowgli-safety-log build --merge-install \
          --install-base /overlay --packages-select mowgli_safety mowgli_bringup
      '
    export SAFETY_OVERLAY_DIR="$overlay_dir"
    docker compose --project-name "$COMPOSE_PROJECT_NAME" --project-directory "$runtime_dir" \
      --env-file "$env_file" -f "$compose_file" -f "$override_file" up -d mowgli
    echo "Safety overlay deployed in SHADOW mode. Verify with:"
    echo "docker exec mowgli-ros2 ros2 param get /safety_supervisor shadow_mode"
    ;;
  rollback)
    "$runtime_dir/docker/stack.sh" up
    echo "Returned to the standard image; overlay is no longer mounted."
    ;;
  *) echo "Usage: $0 {apply|rollback}" >&2; exit 2 ;;
esac
