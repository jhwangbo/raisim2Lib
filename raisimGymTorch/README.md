## raisimGymTorch

Python and C++ bindings for training RaiSim environments with PyTorch.

### Requirements

- Python 3.9 or newer
- CMake 3.10 or newer
- A C++20 compiler
- OpenMP

RaiSim, rayrai, Eigen, and nanobind are included in the parent repository.

### Install and build

From this directory:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python setup.py build_ext --inplace
```

This creates an optimized Release build and uses the PyTorch installation from
the active environment. Additional compiler flags are not required.

### Train ANYmal

```bash
python raisimGymTorch/env/envs/rsg_anymal/runner.py --mode train
```

The runner automatically uses the optimized C++ environment, C++ return
calculation, CPU thread settings, and C++ policy evaluation.
If the C++ TorchScript evaluator is unavailable, evaluation automatically falls
back to PyTorch on CUDA when available, otherwise CPU.

To use another network, change `build_actor_network()` and
`build_critic_network()` in `runner.py`. If it needs a special evaluation
wrapper, also change `build_evaluation_network()`. No C++ changes are needed.

Resume from a checkpoint:

```bash
python raisimGymTorch/env/envs/rsg_anymal/runner.py \
  --mode retrain --weight /path/to/full_N.pt
```

Test a checkpoint:

```bash
python raisimGymTorch/env/envs/rsg_anymal/tester.py \
  --weight /path/to/full_N.pt
```

### Benchmark

```bash
python raisimGymTorch/env/envs/rsg_anymal/benchmark_rollout.py
```
