#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=../config/versions.env
source "$REPO_ROOT/config/versions.env"

set +u
source "/opt/ros/${UAM_ROS_DISTRO}/setup.bash"
set -u

printf 'OS:          '
lsb_release -ds
printf 'Architecture: %s\n' "$(uname -m)"
printf 'Kernel:       %s\n' "$(uname -r)"
printf 'ROS distro:   %s\n' "$ROS_DISTRO"
printf 'Gazebo Sim:   %s\n' "$(gz sim --versions | head -n 1)"
printf 'PX4:          %s\n' "$(git -C "$UAM_PX4_DIR" describe --tags --always --dirty)"
printf 'PX4 commit:   %s\n' "$(git -C "$UAM_PX4_DIR" rev-parse HEAD)"
printf 'px4_msgs:     %s\n' "$(git -C "$UAM_ROS_WS/src/px4_msgs" describe --tags --always --dirty)"
printf 'XRCE Agent:   %s\n' "$(git -C "$UAM_ROS_WS/src/Micro-XRCE-DDS-Agent" describe --tags --always --dirty)"
