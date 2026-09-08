#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=../config/versions.env
source "$REPO_ROOT/config/versions.env"

failures=0

check_equal() {
  local label="$1"
  local expected="$2"
  local actual="$3"
  if [[ "$actual" == "$expected" ]]; then
    printf 'PASS  %-28s %s\n' "$label" "$actual"
  else
    printf 'FAIL  %-28s expected %s, got %s\n' "$label" "$expected" "$actual"
    failures=$((failures + 1))
  fi
}

source /etc/os-release
check_equal 'Ubuntu version' "$UAM_UBUNTU_VERSION" "$VERSION_ID"
check_equal 'PX4 commit' "$UAM_PX4_COMMIT" \
  "$(git -C "$UAM_PX4_DIR" rev-parse HEAD)"
check_equal 'px4_msgs commit' "$UAM_PX4_MSGS_COMMIT" \
  "$(git -C "$UAM_ROS_WS/src/px4_msgs" rev-parse HEAD)"
check_equal 'XRCE Agent commit' "$UAM_XRCE_AGENT_COMMIT" \
  "$(git -C "$UAM_ROS_WS/src/Micro-XRCE-DDS-Agent" rev-parse HEAD)"

set +u
source "/opt/ros/${UAM_ROS_DISTRO}/setup.bash"
source "$UAM_ROS_WS/install/setup.bash"
set -u

for executable in ros2 gz MicroXRCEAgent; do
  if command -v "$executable" >/dev/null; then
    printf 'PASS  command %-20s %s\n' "$executable" "$(command -v "$executable")"
  else
    printf 'FAIL  command missing: %s\n' "$executable"
    failures=$((failures + 1))
  fi
done

if [[ -x "$UAM_PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
  printf 'PASS  PX4 SITL executable\n'
else
  printf 'FAIL  PX4 SITL executable missing\n'
  failures=$((failures + 1))
fi

if ros2 interface show px4_msgs/msg/VehicleOdometry >/dev/null; then
  printf 'PASS  px4_msgs VehicleOdometry interface\n'
else
  printf 'FAIL  px4_msgs VehicleOdometry interface\n'
  failures=$((failures + 1))
fi

if (( failures > 0 )); then
  printf '\nStatic validation failed with %d problem(s).\n' "$failures" >&2
  exit 1
fi

printf '\nStatic validation passed.\n'
