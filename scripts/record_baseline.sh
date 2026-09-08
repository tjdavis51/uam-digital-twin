#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=../config/versions.env
source "$REPO_ROOT/config/versions.env"

set +u
source "/opt/ros/${UAM_ROS_DISTRO}/setup.bash"
source "$UAM_ROS_WS/install/setup.bash"
set -u

mkdir -p "$UAM_BAG_DIR"
name="${1:-px4_x500_$(date +%Y%m%d_%H%M%S)}"

cd "$UAM_BAG_DIR"
exec ros2 bag record -o "$name" --topics \
  /fmu/out/vehicle_odometry \
  /fmu/out/vehicle_attitude \
  /fmu/out/vehicle_local_position_v1 \
  /fmu/out/sensor_combined \
  /fmu/out/vehicle_status_v1 \
  /fmu/out/timesync_status
