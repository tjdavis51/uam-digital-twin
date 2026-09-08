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

required_topics=(
  /fmu/out/sensor_combined
  /fmu/out/vehicle_attitude
  /fmu/out/vehicle_local_position_v1
  /fmu/out/vehicle_odometry
  /fmu/out/vehicle_status_v1
  /fmu/in/offboard_control_mode
  /fmu/in/trajectory_setpoint
  /fmu/in/vehicle_command
)

printf 'Waiting up to 30 seconds for PX4 DDS topics...\n'
deadline=$((SECONDS + 30))
while (( SECONDS < deadline )); do
  topic_list="$(ros2 topic list 2>/dev/null || true)"
  if grep -qx '/fmu/out/vehicle_odometry' <<<"$topic_list"; then
    break
  fi
  sleep 1
done

topic_list="$(ros2 topic list 2>/dev/null || true)"
failures=0
for topic in "${required_topics[@]}"; do
  if grep -qx "$topic" <<<"$topic_list"; then
    printf 'PASS  %s\n' "$topic"
  else
    printf 'FAIL  %s\n' "$topic"
    failures=$((failures + 1))
  fi
done

if (( failures > 0 )); then
  printf '\nLive validation failed with %d missing topic(s).\n' "$failures" >&2
  exit 1
fi

printf '\nReceiving one odometry sample...\n'
timeout 10s ros2 topic echo /fmu/out/vehicle_odometry \
  --qos-reliability best_effort \
  --qos-durability volatile \
  --once

printf '\nLive validation passed.\n'
