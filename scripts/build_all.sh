#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=../config/versions.env
source "$REPO_ROOT/config/versions.env"

set +u
source "/opt/ros/${UAM_ROS_DISTRO}/setup.bash"
set -u

printf 'Building ROS 2 PX4 support workspace...\n'
cd "$UAM_ROS_WS"
colcon build

printf 'Building PX4 SITL...\n'
set +u
source "$UAM_PX4_VENV/bin/activate"
set -u
cd "$UAM_PX4_DIR"
make px4_sitl_default

printf '\nBuild complete. Run %s/scripts/validate_static.sh\n' "$REPO_ROOT"
