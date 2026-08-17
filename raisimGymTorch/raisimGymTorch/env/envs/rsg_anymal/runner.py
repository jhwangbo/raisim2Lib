from ruamel.yaml import YAML
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param, tensorboard_launcher
from raisimGymTorch.env.bin.rsg_anymal import NormalSampler
from raisimGymTorch.env.bin.rsg_anymal import RaisimGymEnv
from raisimGymTorch.env.RewardAnalyzer import RewardAnalyzer
import os
import io
import math
import time
import raisimGymTorch.algo.ppo.module as ppo_module
import raisimGymTorch.algo.ppo.ppo as PPO
import torch.nn as nn
from torch.distributions import Normal
import numpy as np
import torch
import datetime
import argparse


# task specification
task_name = "anymal_locomotion"


def activation_fn():
    return nn.LeakyReLU(inplace=True)


class ZeroEntropyGaussian(ppo_module.MultivariateGaussianDiagonalCovariance):
    """Avoid materializing entropy when PPO's entropy coefficient is zero."""

    def evaluate(self, logits, outputs):
        distribution = Normal(logits, self.std.reshape(self.dim))
        return distribution.log_prob(outputs).sum(dim=1), logits.new_zeros(())

# configuration
parser = argparse.ArgumentParser()
parser.add_argument('-m', '--mode', help='set mode either train or test', type=str, default='train')
parser.add_argument('-w', '--weight', help='pre-trained weight path', type=str, default='')
args = parser.parse_args()
mode = args.mode
weight_path = args.weight

# check if gpu is available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"[RAISIM_GYM] Using device: {device}")

# The policy is small, while RaiSim already occupies most CPU cores. Letting
# PyTorch use its default full CPU pool causes severe nested-thread contention
# during rollout. PPO updates run while RaiSim is idle and benefit from a larger
# pool, so use phase-specific thread counts.
if device.type == 'cpu':
    rollout_torch_threads = 2
    update_torch_threads = min(12, os.cpu_count() or 1)
    torch.set_num_threads(rollout_torch_threads)

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# config
yaml = YAML()
cfg = yaml.load(open(task_path + "/cfg.yaml", 'r'))
cfg_stream = io.StringIO()
yaml.dump(cfg['environment'], cfg_stream)
cfg_env_str = cfg_stream.getvalue()

# create environment from the configuration file
env = VecEnv(RaisimGymEnv(home_path + "/rsc", cfg_env_str))
env.seed(cfg['seed'])

# shortcuts
ob_dim = env.num_obs
act_dim = env.num_acts

# Training
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * env.num_envs

actor = ppo_module.Actor(ppo_module.MLP(cfg['architecture']['policy_net'], activation_fn, ob_dim, act_dim),
                         ZeroEntropyGaussian(act_dim,
                                             env.num_envs,
                                             1.0,
                                             NormalSampler(act_dim),
                                             cfg['seed']),
                         device)
critic = ppo_module.Critic(ppo_module.MLP(cfg['architecture']['value_net'], activation_fn, ob_dim, 1),
                           device)
minimum_action_std = torch.full((act_dim,), 0.2, device=device)

saver = ConfigurationSaver(log_dir=home_path + "/raisimGymTorch/data/"+task_name,
                           save_items=[task_path + "/cfg.yaml", task_path + "/Environment.hpp"])
tensorboard_launcher(saver.data_dir+"/..")  # press refresh (F5) after the first ppo update

ppo = PPO.PPO(actor=actor,
              critic=critic,
              num_envs=cfg['environment']['num_envs'],
              num_transitions_per_env=n_steps,
              num_learning_epochs=4,
              gamma=0.996,
              lam=0.95,
              entropy_coef=0.0,
              num_mini_batches=4,
              device=device,
              log_dir=saver.data_dir,
              shuffle_batch=False,
              )

if device.type == 'cpu':
    # The fused CPU kernel preserves Adam's algorithm while reducing optimizer
    # dispatch overhead for the small actor/critic parameter set.
    ppo.optimizer = torch.optim.Adam(
        [*actor.parameters(), *critic.parameters()],
        lr=ppo.learning_rate,
        fused=True,
    )

reward_analyzer = RewardAnalyzer(env, ppo.writer)

if mode == 'retrain':
    load_param(weight_path, env, actor, critic, ppo.optimizer, saver.data_dir)

for update in range(1000000):
    start = time.time()
    env.reset()
    reward_sum = 0
    done_sum = 0

    if update % cfg['environment']['eval_every_n'] == 0:
        print("Visualizing and evaluating the current policy")
        torch.save({
            'actor_architecture_state_dict': actor.architecture.state_dict(),
            'actor_distribution_state_dict': actor.distribution.state_dict(),
            'critic_architecture_state_dict': critic.architecture.state_dict(),
            'optimizer_state_dict': ppo.optimizer.state_dict(),
        }, saver.data_dir+"/full_"+str(update)+'.pt')
        if device.type == 'cpu':
            loaded_graph = actor.architecture
        else:
            loaded_graph = ppo_module.MLP(cfg['architecture']['policy_net'], activation_fn, ob_dim, act_dim)
            loaded_graph.load_state_dict(actor.architecture.state_dict())

        env.turn_on_visualization()
        env.start_video_recording(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4')

        with torch.inference_mode():
            for step in range(n_steps):
                frame_start = time.time()
                obs = env.observe(False)
                action = loaded_graph.architecture(torch.from_numpy(obs))
                reward, dones = env.step(action.numpy())
                reward_analyzer.add_reward_info(env.get_reward_info())
                frame_end = time.time()
                wait_time = cfg['environment']['control_dt'] - (frame_end-frame_start)
                if wait_time > 0.:
                    time.sleep(wait_time)

        env.stop_video_recording()
        env.turn_off_visualization()

        reward_analyzer.analyze_and_plot(update)
        env.reset()
        env.save_scaling(saver.data_dir, str(update))

    # actual training
    for step in range(n_steps):
        obs = env.observe()
        action = ppo.act(obs)
        reward, dones = env.step(action)
        ppo.step(value_obs=obs, rews=reward, dones=dones)
        done_sum += np.sum(dones)
        reward_sum += np.sum(reward)

    # take st step to get value obs
    obs = env.observe()
    if device.type == 'cpu':
        torch.set_num_threads(update_torch_threads)
    ppo.update(actor_obs=obs, value_obs=obs, log_this_iteration=update % 10 == 0, update=update)
    average_ll_performance = reward_sum / total_steps
    average_dones = done_sum / total_steps

    actor.update()
    actor.distribution.enforce_minimum_std(minimum_action_std)
    if device.type == 'cpu':
        torch.set_num_threads(rollout_torch_threads)

    # curriculum update. Implement it in Environment.hpp
    env.curriculum_callback()

    end = time.time()

    print('----------------------------------------------------')
    print('{:>6}th iteration'.format(update))
    print('{:<40} {:>6}'.format("average ll reward: ", '{:0.10f}'.format(average_ll_performance)))
    print('{:<40} {:>6}'.format("dones: ", '{:0.6f}'.format(average_dones)))
    print('{:<40} {:>6}'.format("time elapsed in this iteration: ", '{:6.4f}'.format(end - start)))
    print('{:<40} {:>6}'.format("fps: ", '{:6.0f}'.format(total_steps / (end - start))))
    print('{:<40} {:>6}'.format("real time factor: ", '{:6.0f}'.format(total_steps / (end - start)
                                                                       * cfg['environment']['control_dt'])))
    print('----------------------------------------------------\n')
