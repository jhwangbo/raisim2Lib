"""Measure async forest loading across serial launches without changing assets.

The viewer must print 'Loading seconds' and 'Loading updates', as the separate
forest_render_test does. An empty --cache-dir measures cache creation followed
by reuse; an existing directory measures reuse. No files are deleted.
"""
import argparse
import json
import os
from pathlib import Path
import re
import statistics
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--cache-dir", type=Path)
    parser.add_argument("--library-dir", type=Path,
                        help="Optional Linux library override for comparing builds")
    parser.add_argument("--runs", type=int, default=3)
    args = parser.parse_args()
    if args.runs < 1:
        parser.error("--runs must be positive")
    env = dict(os.environ)
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
                "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[key] = "1"
    if args.cache_dir:
        env["RAYRAI_ASYNC_LOD_CACHE_DIR"] = str(args.cache_dir.resolve())
    if args.library_dir:
        env["LD_LIBRARY_PATH"] = str(args.library_dir.resolve())
    args.out.mkdir(parents=True, exist_ok=True)
    samples = []
    for index in range(args.runs):
        output = args.out / str(index)
        output.mkdir(exist_ok=True)
        result = subprocess.run(
            [str(args.viewer.resolve()), "--hidden", "--frames", "3"],
            cwd=output, env=env, capture_output=True, text=True, timeout=300)
        (output / "run.log").write_text(result.stdout + result.stderr)
        result.check_returncode()
        seconds = re.search(r"Loading seconds: ([0-9.e+-]+)", result.stdout)
        updates = re.search(
            r"Loading updates: (\d+), longest update seconds: ([0-9.e+-]+)",
            result.stdout)
        if not seconds or not updates:
            raise RuntimeError("Viewer did not report async loading measurements")
        samples.append(dict(run=index, loading_seconds=float(seconds[1]),
                            loading_updates=int(updates[1]),
                            longest_loading_update_seconds=float(updates[2])))
        report = dict(runs=samples, cache_dir=env.get("RAYRAI_ASYNC_LOD_CACHE_DIR"),
                      subsequent_launch_median_seconds=statistics.median(
                          x["loading_seconds"] for x in samples[1:]) if index else None,
                      note="Serial launches; existing async workers remain enabled. "
                           "Loading excludes the subsequent warmup/render benchmark. "
                           "OS file caches are not flushed.")
        (args.out / "loading.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(samples[-1]), flush=True)


if __name__ == "__main__":
    main()
