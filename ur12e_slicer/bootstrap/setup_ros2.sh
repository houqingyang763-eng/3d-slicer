#!/usr/bin/env bash
# Ubuntu 24.04: official ROS 2 Jazzy debs + MoveIt 2 + Universal Robots driver.
# This installs packages only; it does not start or connect to a robot.
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
LOG_BASE="${UR12E_SETUP_LOG_DIR:-$PROJECT_DIR/output/environment}"
RUN_DIR="$LOG_BASE/setup-$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p -- "$RUN_DIR"
exec > >(tee -a "$RUN_DIR/setup.log") 2>&1

on_exit() {
  local result=$?
  trap - EXIT
  {
    printf 'package\tversion\n'
    dpkg-query --show --showformat='${Package}\t${Version}\n' \
      'ros-jazzy-*' ros2-apt-source python3 python3-numpy python3-yaml 2>/dev/null || true
  } > "$RUN_DIR/package_versions.tsv"
  printf '%s\n' "$result" > "$RUN_DIR/exit_code.txt"
  printf '\nSetup exit code: %s\nLogs: %s\n' "$result" "$RUN_DIR"
  exit "$result"
}
trap on_exit EXIT

if [[ ! -r /etc/os-release ]]; then
  printf 'ERROR: /etc/os-release is unavailable. Run inside Ubuntu 24.04.\n' >&2
  exit 2
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 24.04 ]]; then
  printf 'ERROR: expected Ubuntu 24.04; found %s %s.\n' "${ID:-unknown}" "${VERSION_ID:-unknown}" >&2
  exit 2
fi
if [[ "$(dpkg --print-architecture)" != amd64 ]]; then
  printf 'ERROR: the reproducible demo target is Ubuntu 24.04 amd64.\n' >&2
  exit 2
fi

if (( EUID == 0 )); then
  AS_ROOT=()
else
  command -v sudo >/dev/null || { printf 'ERROR: sudo is required.\n' >&2; exit 2; }
  sudo -v
  AS_ROOT=(sudo)
fi
export DEBIAN_FRONTEND=noninteractive

printf 'Installing Ubuntu prerequisites from configured Ubuntu repositories.\n'
"${AS_ROOT[@]}" apt-get update
"${AS_ROOT[@]}" apt-get install -y --no-install-recommends \
  ca-certificates curl locales software-properties-common python3 python3-venv \
  python3-numpy python3-yaml git
"${AS_ROOT[@]}" locale-gen en_US en_US.UTF-8
"${AS_ROOT[@]}" update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
"${AS_ROOT[@]}" add-apt-repository -y universe

# Current Jazzy docs warn that minimal Noble images may omit updates/backports.
# Do not silently rewrite an existing machine's Ubuntu repository policy.
if [[ -f /etc/apt/sources.list.d/ubuntu.sources ]]; then
  for suite in noble-updates noble-backports; do
    if ! grep -Eq "^Suites:.*\\b${suite}\\b" /etc/apt/sources.list.d/ubuntu.sources; then
      printf 'ERROR: %s missing from ubuntu.sources; see bootstrap/README.md.\n' "$suite" >&2
      exit 3
    fi
  done
fi

printf 'Verifying the pinned official ros2-apt-source 1.3.0 package.\n'
# The URL and official GitHub release asset digest are recorded in assets/.
# No latest-release lookup, silent mirror substitution or source-package downgrade.
ROS_SOURCE_VERSION=1.3.0
ROS_SOURCE_DEBIAN_VERSION='1.3.0~noble'
ROS_SOURCE_SHA256=f31d84adf5054c7d60ded0e82c0f776a77ea33af53d08f40b9ad7c94cca55296
if [[ -n "${ROS_APT_SOURCE_VERSION:-}" && "$ROS_APT_SOURCE_VERSION" != "$ROS_SOURCE_VERSION" ]]; then
  printf 'ERROR: this installer pins ros2-apt-source %s; update its source manifest before changing it.\n' "$ROS_SOURCE_VERSION" >&2
  exit 3
fi
INSTALLED_SOURCE_VERSION="$(dpkg-query --show --showformat='${Version}' ros2-apt-source 2>/dev/null || true)"
if [[ -n "$INSTALLED_SOURCE_VERSION" && "$INSTALLED_SOURCE_VERSION" != "$ROS_SOURCE_DEBIAN_VERSION" ]]; then
  printf 'ERROR: existing ros2-apt-source %s differs from pinned %s; retained without modification.\n' "$INSTALLED_SOURCE_VERSION" "$ROS_SOURCE_DEBIAN_VERSION" >&2
  exit 3
fi
if [[ "$INSTALLED_SOURCE_VERSION" == "$ROS_SOURCE_DEBIAN_VERSION" ]]; then
  printf 'Pinned source package is already installed; retaining its managed configuration.\n'
else
  if [[ -n "${ROS_APT_SOURCE_DEB:-}" ]]; then
    [[ -f "$ROS_APT_SOURCE_DEB" ]] || { printf 'ERROR: local ROS source package is missing.\n' >&2; exit 3; }
    cp -- "$ROS_APT_SOURCE_DEB" "$RUN_DIR/ros2-apt-source.deb"
  else
    curl --fail --show-error --silent --location --retry 3 --connect-timeout 15 --max-time 120 \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_SOURCE_VERSION}/ros2-apt-source_${ROS_SOURCE_VERSION}.noble_all.deb" \
      -o "$RUN_DIR/ros2-apt-source.deb"
  fi
  printf '%s  %s\n' "$ROS_SOURCE_SHA256" "$RUN_DIR/ros2-apt-source.deb" | sha256sum --check --status || {
    printf 'ERROR: official ROS source package SHA256 mismatch; file preserved in %s.\n' "$RUN_DIR" >&2; exit 3;
  }
  printf '%s\n' "$ROS_SOURCE_VERSION" > "$RUN_DIR/ros-apt-source-version.txt"
  "${AS_ROOT[@]}" dpkg -i "$RUN_DIR/ros2-apt-source.deb"
fi

printf 'Installing ROS 2 Jazzy, MoveIt 2 and the official UR metapackage.\n'
"${AS_ROOT[@]}" apt-get update
# Upgrade only dependencies selected by apt for these requested packages. Do not
# run a whole-system upgrade on a colleague's existing development machine.
"${AS_ROOT[@]}" apt-get install -y \
  ros-jazzy-desktop ros-dev-tools ros-jazzy-moveit ros-jazzy-ur \
  ros-jazzy-ros2controlcli ros-jazzy-tf2-tools
if [[ "${UR12E_SKIP_SLICER:-0}" != 1 ]]; then
  # fonts-noto-cjk is needed by this project's Chinese labels. It is separate
  # from the upstream Slicer system-library requirements listed below.
  "${AS_ROOT[@]}" apt-get install -y --no-install-recommends \
    libglu1-mesa libpulse-mainloop-glib0 libnss3 libasound2t64 qt5dxcb-plugin fonts-noto-cjk
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
printf 'Checking installed packages and UR12e launch arguments (no nodes launched).\n'
for package in rclpy moveit_msgs moveit_ros_move_group ur_description ur_robot_driver ur_moveit_config; do
  ros2 pkg prefix "$package"
done
ros2 launch ur_robot_driver ur_control.launch.py --show-args > "$RUN_DIR/ur_control_launch_args.txt"
ros2 launch ur_moveit_config ur_moveit.launch.py --show-args > "$RUN_DIR/ur_moveit_launch_args.txt"
if ! grep -q 'ur12e' "$RUN_DIR/ur_control_launch_args.txt" || \
   ! grep -q 'ur12e' "$RUN_DIR/ur_moveit_launch_args.txt"; then
  printf 'ERROR: installed launch descriptions do not advertise ur12e.\n' >&2
  exit 4
fi
python3 - <<'PY'
import numpy
import rclpy
import yaml
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
print("Python imports available: numpy, rclpy, yaml, MoveGroup, ExecuteTrajectory")
PY
printf '\nInstallation checks passed. Runtime planning and execution remain separate checks.\n'
printf 'For each new shell: source /opt/ros/jazzy/setup.bash\n'
