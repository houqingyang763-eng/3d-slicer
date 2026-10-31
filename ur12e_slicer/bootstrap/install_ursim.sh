#!/usr/bin/env bash
# Add optional local URSim dependencies; the existing mock installation is separate.
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
if [[ "$EUID" == 0 ]]; then
  printf 'Run as an ordinary Linux user; sudo is used only for system packages.\n' >&2
  exit 2
fi
mkdir -p -- "$PROJECT/output/ursim_install"
LOG="$PROJECT/output/ursim_install/$(date -u +%Y%m%dT%H%M%SZ).log"
exec > >(tee -a "$LOG") 2>&1
printf 'URSim installation log: %s\n' "$LOG"
if ! command -v docker >/dev/null; then
  sudo apt-get update
  sudo apt-get install -y docker.io
  sudo systemctl enable --now docker
  sudo usermod -aG docker "$USER"
fi
if ! docker --host unix:///var/run/docker.sock info >/dev/null; then
  printf 'Local Docker is unavailable to this user. Check the daemon and docker group; reopen the Linux session after adding group membership.\n' >&2
  exit 2
fi
python3 "$SCRIPT_DIR/prepare_ursim.py"
printf 'Assets are prepared. See URSIM_LINUX.md for the first PolyScope configuration.\n'
