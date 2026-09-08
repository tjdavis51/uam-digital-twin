#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=../config/versions.env
source "$REPO_ROOT/config/versions.env"

set +u
source "/opt/ros/${UAM_ROS_DISTRO}/setup.bash"
source "$UAM_PX4_VENV/bin/activate"
set -u
cd "$UAM_PX4_DIR"

export PX4_UXRCE_DDS_PORT="$UAM_DDS_PORT"
export PX4_GZ_SIM_RENDER_ENGINE="$UAM_RENDER_ENGINE"

exec make px4_sitl "$UAM_GZ_MODEL"
