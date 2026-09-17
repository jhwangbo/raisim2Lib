"""Serial completed-frame benchmark; writes a small JSON report outside the source tree."""
import argparse
import json
import os
import platform
import re
import statistics
import subprocess
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--viewer', type=Path, required=True,
                    help='Path to forest_render_test (the interactive example has no frame limit)')
parser.add_argument('--out', type=Path, required=True)
parser.add_argument('--frames', type=int, default=300)
parser.add_argument('--runs', type=int, default=3)
args = parser.parse_args()
if args.frames < 1 or args.runs < 1:
    parser.error('frames and runs must be positive')
env = dict(os.environ)
for name in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS',
             'VECLIB_MAXIMUM_THREADS','NUMEXPR_NUM_THREADS']:
    env[name] = '1'
fps, loading_updates, loading_stalls = [], [], []
for _ in range(args.runs):
    run = subprocess.run([str(args.viewer.resolve()),'--hidden','--frames',str(args.frames)],
                         env=env,check=True,capture_output=True,text=True,timeout=300)
    match = re.search(r'Completed-frame FPS: ([0-9.]+)',run.stdout)
    if not match:
        raise RuntimeError(run.stdout + run.stderr)
    fps.append(float(match.group(1)))
    loading = re.search(r'Loading updates: ([0-9]+), longest update seconds: ([0-9.e+-]+)', run.stdout)
    if not loading:
        raise RuntimeError('Loading responsiveness measurements missing: ' + run.stdout)
    loading_updates.append(int(loading.group(1)))
    loading_stalls.append(float(loading.group(2)))
report = dict(platform=platform.platform(),resolution=[1280,800],msaa=4,warmup=60,
              frames=args.frames,runs=args.runs,fps=fps,median_fps=statistics.median(fps),
              loading_updates=loading_updates,max_loading_update_seconds=loading_stalls,
              includes='simulation, UI, rendering, swap, and GPU completion; excludes loading/warmup')
args.out.parent.mkdir(parents=True,exist_ok=True)
args.out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
