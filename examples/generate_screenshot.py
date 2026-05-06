#!/usr/bin/env python3
"""Run a rayrai example and capture a documentation screenshot.

The default target is rayrai_pbr_texture_maps because its assets need a short
settling period before the screenshot is useful.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def require_tool(name: str) -> None:
    if shutil.which(name) is None:
        raise RuntimeError(f"required command not found: {name}")


def resolve_executable(root: Path, build_dir: Path, example: str) -> Path:
    candidates = [
        build_dir / "examples" / example,
        build_dir / "bin" / example,
        build_dir / example,
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    searched = "\n  ".join(str(path) for path in candidates)
    raise FileNotFoundError(f"could not find executable for {example}; searched:\n  {searched}")


def find_window_id(example: str) -> str:
    result = subprocess.run(
        ["xwininfo", "-root", "-tree"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=True,
    )
    pattern = re.compile(re.escape(example), re.IGNORECASE)
    for line in result.stdout.splitlines():
        if pattern.search(line):
            return line.split()[0]
    raise RuntimeError(f"could not find a window matching {example}")


def capture_window(window_id: str, output: Path, titlebar_crop: int, edge_crop: int) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    raw = output.with_suffix(output.suffix + ".raw.png")
    try:
        subprocess.run(["import", "-window", window_id, str(raw)], check=True)
        convert_cmd = [
            "convert",
            str(raw),
            "-alpha",
            "off",
        ]
        if titlebar_crop > 0:
            convert_cmd.extend(["-gravity", "North", "-chop", f"0x{titlebar_crop}", "+repage"])
        if edge_crop > 0:
            convert_cmd.extend(["-shave", f"{edge_crop}x{edge_crop}"])
        convert_cmd.extend(["-resize", "1280x720>", str(output)])
        subprocess.run(convert_cmd, check=True)
    finally:
        raw.unlink(missing_ok=True)


def main() -> int:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--example", default="rayrai_pbr_texture_maps")
    parser.add_argument("--build-dir", type=Path, default=root / "build")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--wait", type=float, default=18.0, help="seconds to wait before capture")
    parser.add_argument("--titlebar-crop", type=int, default=44)
    parser.add_argument("--edge-crop", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=5.0, help="seconds to wait while stopping the example")
    args = parser.parse_args()

    require_tool("xwininfo")
    require_tool("import")
    require_tool("convert")

    build_dir = args.build_dir if args.build_dir.is_absolute() else root / args.build_dir
    executable = resolve_executable(root, build_dir, args.example)
    output = args.output or root / "docs" / "image" / f"{args.example}.png"
    if not output.is_absolute():
        output = root / output

    env = os.environ.copy()
    lib_paths = [root / "raisim" / "lib", root / "rayrai" / "lib"]
    existing_ld_path = env.get("LD_LIBRARY_PATH")
    env["LD_LIBRARY_PATH"] = ":".join(str(path) for path in lib_paths if path.exists())
    if existing_ld_path:
        env["LD_LIBRARY_PATH"] += f":{existing_ld_path}"

    log_path = Path("/tmp") / f"{args.example}_screenshot.log"
    with log_path.open("wb") as log:
        process = subprocess.Popen(
            [str(executable)],
            cwd=root,
            stdout=log,
            stderr=subprocess.STDOUT,
            env=env,
        )
        try:
            time.sleep(args.wait)
            window_id = find_window_id(args.example)
            capture_window(window_id, output, args.titlebar_crop, args.edge_crop)
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=args.timeout)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=args.timeout)

    print(f"wrote {output}")
    print(f"log: {log_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
