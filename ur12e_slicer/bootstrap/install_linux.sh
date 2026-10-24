#!/usr/bin/env bash
# Install dependencies/resources only. Does not start ROS, Slicer or a robot.
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
CHECK_ONLY=0
OFFLINE=0
SKIP_SLICER=0
for argument in "$@"; do
  case "$argument" in
    --check-only) CHECK_ONLY=1 ;;
    --offline) OFFLINE=1 ;;
    --skip-slicer) SKIP_SLICER=1 ;;
    -h|--help)
      printf 'Usage: bash bootstrap/install_linux.sh [--check-only] [--offline] [--skip-slicer]\n'
      printf 'Online install requires Ubuntu 24.04 x86_64 and sudo. Does not start applications.\n'
      printf '--check-only: read-only, no network, no sudo. --offline: restore cached assets, check installed dependencies; no apt.\n'
      exit 0 ;;
    *) printf 'Unknown option: %s\n' "$argument" >&2; exit 2 ;;
  esac
done
command -v python3 >/dev/null || { printf 'Python 3 is required (Ubuntu 24.04 default).\n' >&2; exit 2; }
FLAGS=()
(( SKIP_SLICER == 0 )) || FLAGS+=(--skip-slicer)
if (( CHECK_ONLY )); then
  exec python3 "$SCRIPT_DIR/check_linux.py" "${FLAGS[@]}"
fi
if (( OFFLINE )); then
  python3 "$SCRIPT_DIR/fetch_assets.py" --offline "${FLAGS[@]}"
else
  UR12E_SKIP_SLICER="$SKIP_SLICER" bash "$SCRIPT_DIR/setup_ros2.sh"
  python3 "$SCRIPT_DIR/fetch_assets.py" "${FLAGS[@]}"
fi
python3 "$SCRIPT_DIR/check_linux.py" "${FLAGS[@]}"
printf 'Installation checks passed. Runtime validation is separate; use start_linux.sh when ready.\n'
