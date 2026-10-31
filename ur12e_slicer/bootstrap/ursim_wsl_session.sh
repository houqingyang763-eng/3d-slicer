#!/usr/bin/env bash
# Keep the WSL invocation alive while this task's simulator is running.
# systemd services alone do not keep an otherwise idle WSL distribution alive.
set -Eeuo pipefail
SCRIPT_FILE="$(readlink -f -- "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_FILE")" && pwd)"
exec python3 - "$SCRIPT_DIR" "$@" <<'PY'
import fcntl
import json
import os
import subprocess
import sys
import time
from pathlib import Path


def guardian_identity(lifecycle, owner, script_dir):
    if not owner or owner.get('project_key') != lifecycle.PROJECT_KEY or owner.get('script_dir') != str(script_dir):
        return False
    process = lifecycle.common().process(owner['pid'])
    return bool(process and process['state'] not in ('Z', 'X')
                and str(process['start_ticks']) == str(owner.get('start_ticks'))
                and str(script_dir).encode() in process['command'])


def read_owner(path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except FileNotFoundError:
        return None


def wait_stopped(lifecycle, lock, owner_path, script_dir, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            owner = read_owner(owner_path)
            if owner is not None and not guardian_identity(lifecycle, owner, script_dir):
                raise RuntimeError('Guardian owner PID/start ticks/project mismatch; no signal sent.')
        else:
            owner = read_owner(owner_path)
            if owner is None or not guardian_identity(lifecycle, owner, script_dir):
                print(json.dumps({'guardian_stopped': True, 'owner': owner}), flush=True)
                return
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
        time.sleep(0.1)
    raise RuntimeError('The recorded URSim guardian did not exit within 15 seconds; owner record retained, no signal sent.')


def main(arguments):
    script_dir = Path(arguments[0]).resolve()
    sys.path.insert(0, str(script_dir))
    import ursim_lifecycle as lifecycle
    if os.geteuid() == 0:
        raise RuntimeError('Run the WSL guardian as the ordinary project user.')
    folder = lifecycle.cache_root() / 'guardian'
    folder.mkdir(parents=True, exist_ok=True)
    owner_path = folder / 'owner.json'
    with (folder / 'session.lock').open('a') as lock:
        if arguments[1:] == ['--wait-stopped']:
            wait_stopped(lifecycle, lock, owner_path, script_dir)
            return
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            owner = read_owner(owner_path)
            if owner is not None and not guardian_identity(lifecycle, owner, script_dir):
                raise RuntimeError('Active guardian identity mismatch; no new guardian was started.')
            print(json.dumps({'guardian_reused': True, 'owner': owner}), flush=True)
            return
        process = lifecycle.common().process(os.getpid())
        if not process:
            raise RuntimeError('Cannot record the guardian process identity.')
        owner = {'pid': os.getpid(), 'start_ticks': process['start_ticks'],
                 'project_key': lifecycle.PROJECT_KEY, 'script_dir': str(script_dir),
                 'started_at': time.time(), 'stopped_at': None}
        lifecycle.write_json(owner_path, owner)
        try:
            subprocess.run([sys.executable, str(script_dir / 'ursim_lifecycle.py'), 'start', *arguments[1:]], check=True)
            docker = lifecycle.Docker()
            while lifecycle.verified_container(docker)['State']['Running']:
                time.sleep(1)
        finally:
            owner['stopped_at'] = time.time()
            lifecycle.write_json(owner_path, owner)


if __name__ == '__main__':
    main(sys.argv[1:])
PY
