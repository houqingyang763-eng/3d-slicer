"""Encode real Slicer frame captures using their recorded wall-clock intervals.

Run with runtime/media_env/Scripts/python.exe (imageio_ffmpeg provides ffmpeg).
This script does not render or interpolate robot poses.
"""
from pathlib import Path
import argparse
import json
import subprocess

import imageio_ffmpeg


def encode(folder, destination=None, width=1600):
    folder = Path(folder).resolve()
    manifest = json.loads((folder / "frames.json").read_text(encoding="utf-8"))
    frames = manifest["frames"]
    if not frames:
        raise ValueError("No real captured frames")
    lines = []
    for index, frame in enumerate(frames):
        filename = frame["file"]
        if Path(filename).name != filename or not (folder / filename).is_file():
            raise ValueError("Invalid recorded frame: " + filename)
        duration = (frames[index + 1]["timestamp"] - frame["timestamp"]) if index + 1 < len(frames) else 1 / manifest["frame_rate"]
        if duration <= 0:
            raise ValueError("Capture timestamps are not increasing")
        lines += ["file '" + filename + "'", "duration {:.9f}".format(duration)]
    lines.append("file '" + frames[-1]["file"] + "'")
    duration_s = frames[-1]["timestamp"] - frames[0]["timestamp"] + 1 / manifest["frame_rate"]
    concat = folder / "ffmpeg_frames.txt"
    concat.write_text("\n".join(lines) + "\n", encoding="utf-8")
    destination = Path(destination).resolve() if destination else folder.parent / "UR12e_Slicer_MoveIt2_demo.mp4"
    # ffmpeg's concat demuxer does not reliably resolve a Windows backslash
    # directory as the base for relative frame names. Run inside that directory.
    command = [imageio_ffmpeg.get_ffmpeg_exe(), "-hide_banner", "-y", "-f", "concat", "-safe", "0", "-i", concat.name,
               "-vsync", "vfr", "-vf", "scale={}:trunc(ow/a/2)*2".format(width),
               "-c:v", "libx264", "-bf", "0", "-crf", "20", "-preset", "fast", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
               "-t", "{:.9f}".format(duration_s), str(destination)]
    result = subprocess.run(command, cwd=str(folder), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
    (folder.parent / "video_encode.log").write_text(result.stdout, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(result.stdout)
    report = {"video": str(destination), "source_frames": len(frames), "width": width,
              "source_elapsed_s": frames[-1]["timestamp"] - frames[0]["timestamp"],
              "requested_duration_s": duration_s,
              "capture": manifest["capture"], "simulation_only": True,
              "timing": "Variable frame duration from original Slicer capture timestamps; no synthetic motion frames",
              "ffmpeg": imageio_ffmpeg.get_ffmpeg_exe(), "bytes": destination.stat().st_size}
    (folder.parent / "video_manifest.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("frames_directory", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--width", type=int, default=1600)
    args = parser.parse_args()
    print(json.dumps(encode(args.frames_directory, args.output, args.width), indent=2))
