#!/usr/bin/env bash
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  printf 'ROS 2 Jazzy is missing. Run bootstrap/install_linux.sh first.\n' >&2
  exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=62 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$HERE/ros2_ws/src/ur12e_slicer_bridge${PYTHONPATH:+:$PYTHONPATH}"
exec python3 "$HERE/validation/validate_anatomy.py" "$@"
