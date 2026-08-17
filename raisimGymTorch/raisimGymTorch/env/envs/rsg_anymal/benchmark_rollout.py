"""Benchmark deterministic ANYmal rollout implementations and OpenMP counts.

This benchmark never invokes PPO or mutates training code. Rendering and the
real-time frame limiter are disabled so it measures simulation/policy throughput.
"""

import argparse
import io
import os
import statistics
import sys
import tempfile
import time

if sys.platform.startswith('linux'):
    os.environ.setdefault('OMP_PROC_BIND', 'close')
    os.environ.setdefault('OMP_PLACES', 'cores')
os.environ.setdefault('OMP_DYNAMIC', 'FALSE')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np
import torch
import torch.nn as nn
from ruamel.yaml import YAML

import raisimGymTorch.algo.ppo.module as ppo_module
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.env.bin import rsg_anymal


def activation_fn():
    return nn.LeakyReLU(inplace=True)


def parse_thread_counts(value):
    counts = []
    for item in value.split(','):
        count = int(item)
        if count <= 0:
            raise argparse.ArgumentTypeError('thread counts must be positive')
        if count not in counts:
            counts.append(count)
    return counts


def time_python(env, network, steps, warmup):
    env.reset()
    with torch.inference_mode():
        for _ in range(warmup):
            observation = env.observe(False)
            env.step(network(torch.from_numpy(observation)).numpy())
        start = time.perf_counter()
        for _ in range(steps):
            observation = env.observe(False)
            env.step(network(torch.from_numpy(observation)).numpy())
        return time.perf_counter() - start


def time_torch_cpp(env, runner, reward_history, warmup_history):
    env.reset()
    runner.run(warmup_history, 0.0)
    start = time.perf_counter()
    runner.run(reward_history, 0.0)
    return time.perf_counter() - start


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--threads', type=parse_thread_counts,
                        default=parse_thread_counts('1,2,4,8,12,16,20,24,26,28,32'))
    parser.add_argument('--steps', type=int, default=1200)
    parser.add_argument('--warmup', type=int, default=20)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--torch-threads', type=int, default=1)
    parser.add_argument('--seed', type=int, default=1,
                        help='use the same initialized policy for every thread count')
    parser.add_argument('--weight', default='',
                        help='optional full_*.pt checkpoint whose actor policy should be benchmarked')
    args = parser.parse_args()

    torch.set_num_threads(args.torch_threads)
    yaml = YAML()
    task_path = os.path.dirname(os.path.realpath(__file__))
    home_path = os.path.normpath(os.path.join(task_path, '../../../../..'))
    with open(os.path.join(task_path, 'cfg.yaml'), 'r', encoding='utf-8') as config_file:
        config = yaml.load(config_file)
    base_cfg = config['environment']
    policy_shape = [int(size) for size in config['architecture']['policy_net']]
    actor_state = None
    if args.weight:
        checkpoint = torch.load(args.weight, map_location='cpu', weights_only=False)
        actor_state = checkpoint['actor_architecture_state_dict']

    thread_counts = [count for count in args.threads
                     if count <= (os.cpu_count() or count)]
    if not thread_counts:
        raise ValueError('no requested thread count is available on this machine')

    env_cfg = base_cfg.copy()
    env_cfg['render'] = False
    env_cfg['num_threads'] = max(thread_counts)
    cfg_stream = io.StringIO()
    yaml.dump(env_cfg, cfg_stream)
    env = VecEnv(rsg_anymal.RaisimGymEnv(
        os.path.join(home_path, 'rsc'), cfg_stream.getvalue()))

    torch.manual_seed(args.seed)
    policy_module = ppo_module.MLP(
        policy_shape, activation_fn, env.num_obs, env.num_acts)
    if actor_state is not None:
        policy_module.load_state_dict(actor_state)
    policy = policy_module.architecture
    scripted_policy = torch.jit.script(policy)
    policy_directory = tempfile.TemporaryDirectory(prefix='rsg-anymal-policy-')
    policy_path = os.path.join(policy_directory.name, 'policy.pt')
    torch.jit.save(scripted_policy, policy_path)
    torch_runner = (rsg_anymal.TorchPolicyRunner(env.wrapper, policy_path)
                    if hasattr(rsg_anymal, 'TorchPolicyRunner') else None)

    reward_dim = len(env.get_reward_names())
    reward_history = np.empty(
        (args.steps, env.num_envs, reward_dim), dtype=np.float32)
    warmup_history = np.empty(
        (args.warmup, env.num_envs, reward_dim), dtype=np.float32)

    print('milliseconds per control step (lower is better)')
    print('threads    python  cpp-torchscript  torch-speedup')
    for thread_count in thread_counts:
        env.set_num_threads(thread_count)
        measurements = {'python': [], 'torchscript': []}
        for _ in range(args.repeats):
            measurements['python'].append(
                time_python(env, policy, args.steps, args.warmup))
            if torch_runner is not None:
                measurements['torchscript'].append(
                    time_torch_cpp(
                        env, torch_runner, reward_history, warmup_history))

        per_step_ms = {
            name: statistics.median(values) * 1000.0 / args.steps
            for name, values in measurements.items() if values
        }
        torch_ms = per_step_ms.get('torchscript', float('nan'))
        print(f'{thread_count:7d}  {per_step_ms["python"]:8.4f}'
              f'  {torch_ms:15.4f}'
              f'  {per_step_ms["python"] / torch_ms:13.2f}x',
              flush=True)
    del torch_runner
    policy_directory.cleanup()
    env.close()


if __name__ == '__main__':
    main()
