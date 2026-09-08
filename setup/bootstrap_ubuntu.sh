#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=../config/versions.env
source "$REPO_ROOT/config/versions.env"

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

ensure_checkout() {
  local url="$1"
  local destination="$2"
  local commit="$3"

  if [[ ! -e "$destination" ]]; then
    mkdir -p "$(dirname -- "$destination")"
    git clone "$url" "$destination"
  fi

  [[ -d "$destination/.git" ]] || die "$destination exists but is not a Git repository"

  local current
  current="$(git -C "$destination" rev-parse HEAD)"
  if [[ "$current" != "$commit" ]]; then
    if [[ -n "$(git -C "$destination" status --porcelain)" ]]; then
      die "$destination contains changes; refusing to change its revision"
    fi
    git -C "$destination" fetch --tags origin
    git -C "$destination" checkout --detach "$commit"
  fi
}

[[ -r /etc/os-release ]] || die 'This installer requires Ubuntu 24.04'
# shellcheck source=/etc/os-release
source /etc/os-release
[[ "$ID" == ubuntu ]] || die "Expected Ubuntu, found $ID"
[[ "$VERSION_ID" == "$UAM_UBUNTU_VERSION" ]] || \
  die "Expected Ubuntu $UAM_UBUNTU_VERSION, found $VERSION_ID"

case "$(uname -m)" in
  x86_64|aarch64) ;;
  *) die "Unsupported architecture: $(uname -m)" ;;
esac

printf 'Installing base tools and ROS repository support...\n'
sudo apt-get update
sudo apt-get install -y \
  ca-certificates curl git gnupg locales lsb-release \
  python3-pip python3-venv software-properties-common
sudo add-apt-repository -y universe

if ! dpkg-query -W ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION="${ROS_APT_SOURCE_VERSION:-1.2.0}"
  TEMP_DIR="$(mktemp -d)"
  trap 'rm -rf "$TEMP_DIR"' EXIT
  curl -fL \
    -o "$TEMP_DIR/ros2-apt-source.deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
  sudo dpkg -i "$TEMP_DIR/ros2-apt-source.deb"
fi

sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  ros-dev-tools \
  "ros-${UAM_ROS_DISTRO}-desktop" \
  "ros-${UAM_ROS_DISTRO}-gz-sim-vendor" \
  "ros-${UAM_ROS_DISTRO}-gz-tools-vendor" \
  "ros-${UAM_ROS_DISTRO}-ros-gz"

ensure_checkout "$UAM_PX4_URL" "$UAM_PX4_DIR" "$UAM_PX4_COMMIT"
git -C "$UAM_PX4_DIR" submodule update --init --recursive

mkdir -p "$(dirname -- "$UAM_PX4_VENV")"
if [[ ! -x "$UAM_PX4_VENV/bin/python" ]]; then
  python3 -m venv "$UAM_PX4_VENV"
fi

# Use PX4's pinned dependency installer, but leave Gazebo to the ROS Jazzy
# vendor packages above. NuttX is optional because SITL does not require it.
# Set UAM_INSTALL_NUTTX=1 when preparing an amd64 hardware-firmware workstation.
set +u
source "$UAM_PX4_VENV/bin/activate"
set -u
PX4_SETUP_ARGS=(--no-sim-tools)
if [[ "${UAM_INSTALL_NUTTX:-0}" != 1 ]]; then
  PX4_SETUP_ARGS+=(--no-nuttx)
fi
bash "$UAM_PX4_DIR/Tools/setup/ubuntu.sh" "${PX4_SETUP_ARGS[@]}"
python -m pip install --upgrade pip
python -m pip install --requirement "$REPO_ROOT/config/px4-venv-requirements.lock"
set +u
deactivate
set -u

mkdir -p "$UAM_ROS_WS/src"
ensure_checkout \
  "$UAM_PX4_MSGS_URL" "$UAM_ROS_WS/src/px4_msgs" "$UAM_PX4_MSGS_COMMIT"
ensure_checkout \
  "$UAM_XRCE_AGENT_URL" "$UAM_ROS_WS/src/Micro-XRCE-DDS-Agent" \
  "$UAM_XRCE_AGENT_COMMIT"

if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
set +u
source "/opt/ros/${UAM_ROS_DISTRO}/setup.bash"
set -u
rosdep install --from-paths "$UAM_ROS_WS/src" --ignore-src \
  --rosdistro "$UAM_ROS_DISTRO" -y

printf '\nBootstrap complete. Build with:\n  %s/scripts/build_all.sh\n' "$REPO_ROOT"
