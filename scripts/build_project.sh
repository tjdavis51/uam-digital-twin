#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=../config/versions.env
source "$REPO_ROOT/config/versions.env"
if [[ ! -f "/opt/ros/${UAM_ROS_DISTRO}/setup.bash" ]]; then
  printf 'ROS %s is required. For the standalone core use scripts/test_workbench.sh.\n' "$UAM_ROS_DISTRO" >&2
  exit 1
fi
set +u
source "/opt/ros/${UAM_ROS_DISTRO}/setup.bash"
# No PX4 dependency in milestone 1; use the support overlay if it is already built.
if [[ -f "$UAM_ROS_WS/install/setup.bash" ]]; then
  source "$UAM_ROS_WS/install/setup.bash"
fi
set -u
cd "$REPO_ROOT"
colcon --log-base log/project build --base-paths src   --build-base build/project --install-base install/project   --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
