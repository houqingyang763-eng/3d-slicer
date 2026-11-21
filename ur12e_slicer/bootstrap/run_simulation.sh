#!/usr/bin/env bash
# Start only the task's UR12e mock robot and MoveIt graph.
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
RUNTIME="$PROJECT/runtime"
KEEPOUT=false
HEAD_ASSETS=""
for argument in "$@"; do
  case "$argument" in
    --keepout) KEEPOUT=true ;;
    --anatomy)
      KEEPOUT=true
      HEAD_ASSETS="$PROJECT/.local/head_anatomy"
      ;;
    *) printf 'Unknown mock argument: %s\n' "$argument" >&2; exit 2 ;;
  esac
done
if [[ -n "$HEAD_ASSETS" && ! -r "$HEAD_ASSETS/manifest.json" ]]; then
  printf 'Prepare BrainWeb head assets with bootstrap/prepare_anatomy.py first.\n' >&2
  exit 2
fi
PROJECT_KEY="$(printf '%s' "$PROJECT" | sha256sum | cut -c1-12)"
WORKSPACE="${UR12E_BUILD_DIR:-${XDG_CACHE_HOME:-$HOME/.cache}/ur12e_slicer/$PROJECT_KEY/ros2_ws}"
mkdir -p -- "$RUNTIME" "$WORKSPACE"
if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  printf 'ROS 2 Jazzy is missing. Run bootstrap/install_linux.sh first.\n' >&2
  exit 2
fi
if [[ "${UR12E_OWN_SESSION:-0}" != 1 ]]; then
  # WSL must keep its entry process alive while the new session runs.
  exec env UR12E_OWN_SESSION=1 setsid --wait bash "$0" "$@"
fi
export ROS_DOMAIN_ID=62
export ROS_LOCALHOST_ONLY=1
set +u
source /opt/ros/jazzy/setup.bash
set -u
python3 - "$RUNTIME/linux_launch.json" "$KEEPOUT" "$HEAD_ASSETS" <<'PY'
import json, os, sys, time
from pathlib import Path
p = Path(sys.argv[1])
if p.exists():
    previous = json.loads(p.read_text(encoding="utf-8"))
    old = Path(f"/proc/{previous.get('pid', 0)}")
    if old.exists():
        ticks = old.joinpath("stat").read_text().split(") ", 1)[1].split()[19]
        command = old.joinpath("cmdline").read_bytes()
        if ticks == str(previous.get("start_ticks")) and b"ur12e" in command:
            raise SystemExit("This task already has a live simulation process. Stop it first.")
pid = os.getppid()
ticks = Path(f"/proc/{pid}/stat").read_text().split(") ", 1)[1].split()[19]
p.write_text(json.dumps({"pid": pid, "start_ticks": ticks, "started_at": time.time(), "domain_id": 62,
                         "mode": "mock_hardware", "keepout": sys.argv[2] == "true",
                         "anatomy": bool(sys.argv[3])}), encoding="utf-8")
PY
cd -- "$WORKSPACE"
colcon --log-base "$WORKSPACE/log" build \
  --base-paths "$PROJECT/ros2_ws/src" \
  --build-base "$WORKSPACE/build" --install-base "$WORKSPACE/install" \
  --packages-select ur12e_slicer_bridge --symlink-install
set +u
source "$WORKSPACE/install/setup.bash"
set -u
# stop_simulation.sh signals launch alone. Noninteractive mode makes launch
# forward SIGINT to its children instead of assuming a terminal already did.
launch_args=("runtime_dir:=$RUNTIME" "launch_rviz:=${UR12E_RVIZ:-false}" "keepout:=$KEEPOUT")
if [[ -n "$HEAD_ASSETS" ]]; then
  launch_args+=("head_assets_dir:=$HEAD_ASSETS")
fi
exec ros2 launch --noninteractive ur12e_slicer_bridge mock.launch.py "${launch_args[@]}"
