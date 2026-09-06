from datetime import datetime
import math
import os
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter
from .storage import RolloutStorage

_LOG_SQRT_2PI = 0.5 * math.log(2.0 * math.pi)


class CudaGraphRollout:
    """Replay the whole per-step rollout policy as a single captured CUDA graph.

    A rollout step costs roughly 135 us of launch and synchronization latency
    against 850 us of simulation: seven eager dispatches for a two-layer MLP, a
    pageable host-to-device copy of the observation, and two hard
    synchronizations. Capturing the forward pass, the action sampling and the
    writes into the rollout buffers leaves five host-visible operations per
    step, and the transition never leaves the device afterwards.

    The buffer index lives on the device and is incremented inside the graph, so
    a replay needs no argument. It is re-zeroed whenever a new rollout starts.
    """

    def __init__(self, actor, storage, device):
        self.actor = actor
        self.storage = storage
        self.device = device

        network = actor.architecture.architecture
        num_envs = storage.num_envs
        obs_dim = storage.actor_obs_tc.shape[-1]
        action_dim = storage.actions_tc.shape[-1]
        self.log_normalizer = action_dim * _LOG_SQRT_2PI

        self.observation_host = torch.empty(num_envs, obs_dim, dtype=torch.float32, pin_memory=True)
        self.observation_np = self.observation_host.numpy()
        self.action_host = torch.empty(num_envs, action_dim, dtype=torch.float32, pin_memory=True)
        self.action_np = self.action_host.numpy()

        self.observation = torch.zeros(num_envs, obs_dim, dtype=torch.float32, device=device)
        self.action = torch.zeros(num_envs, action_dim, dtype=torch.float32, device=device)
        self.index = torch.zeros(1, dtype=torch.long, device=device)
        self.done = torch.cuda.Event()

        # Capture under no_grad rather than inference mode: the captured
        # intermediates are replayed from inference-mode rollout loops, and
        # inference tensors may not be written to from outside inference mode.
        stream = torch.cuda.Stream()
        stream.wait_stream(torch.cuda.current_stream())
        with torch.cuda.stream(stream), torch.no_grad():
            for _ in range(3):
                self._body(network)
        torch.cuda.current_stream().wait_stream(stream)

        self.graph = torch.cuda.CUDAGraph()
        with torch.no_grad(), torch.cuda.graph(self.graph):
            self._body(network)
        self.index.zero_()

    def _body(self, network):
        std = self.actor.distribution.std
        mean = network(self.observation)
        noise = torch.randn_like(mean)
        action = torch.addcmul(mean, noise, std)
        log_prob = (noise.square().sum(1) * -0.5) - std.log().sum() - self.log_normalizer

        index = self.index.remainder(self.storage.num_transitions_per_env)
        self.storage.actor_obs_tc.index_copy_(0, index, self.observation.unsqueeze(0))
        self.storage.actions_tc.index_copy_(0, index, action.unsqueeze(0))
        self.storage.mu_tc.index_copy_(0, index, mean.unsqueeze(0))
        self.storage.actions_log_prob_tc.index_copy_(0, index, log_prob.view(1, -1, 1))
        self.action.copy_(action)
        self.index.add_(1)

    def act(self, actor_obs):
        if self.storage.step == 0:
            self.index.zero_()
            # The action standard deviation is a policy parameter, so it is
            # constant for the whole rollout. Broadcasting it once beats writing
            # an identical row on every one of the captured steps.
            self.storage.sigma_tc.copy_(self.actor.distribution.std.detach().view(1, 1, -1))

        np.copyto(self.observation_np, actor_obs)
        self.observation.copy_(self.observation_host, non_blocking=True)
        self.graph.replay()
        self.action_host.copy_(self.action, non_blocking=True)
        self.done.record()
        self.done.synchronize()
        return self.action_np


class PPO:
    def __init__(self,
                 actor,
                 critic,
                 num_envs,
                 num_transitions_per_env,
                 num_learning_epochs,
                 num_mini_batches,
                 clip_param=0.2,
                 gamma=0.998,
                 lam=0.95,
                 value_loss_coef=0.5,
                 entropy_coef=0.0,
                 learning_rate=5e-4,
                 max_grad_norm=0.5,
                 learning_rate_schedule='adaptive',
                 desired_kl=0.01,
                 use_clipped_value_loss=True,
                 log_dir='run',
                 device='cpu',
                 shuffle_batch=True,
                 shared_observations=False,
                 returns_calculator=None,
                 compile_networks=False,
                 cuda_graph_rollout=True):

        # PPO components
        self.actor = actor
        self.critic = critic
        self.shared_observations = shared_observations
        self.storage = RolloutStorage(
            num_envs, num_transitions_per_env, actor.obs_shape, critic.obs_shape,
            actor.action_shape, device, shared_observations, returns_calculator)

        if shuffle_batch:
            self.batch_sampler = self.storage.mini_batch_generator_shuffle
        else:
            self.batch_sampler = self.storage.mini_batch_generator_inorder

        self.trainable_parameters = [*self.actor.parameters(), *self.critic.parameters()]
        self.device = device
        self.device_type = torch.device(device).type

        # The adaptive schedule compares the sampled KL against a threshold once
        # per minibatch. Reading that comparison on the host stalls the pipeline
        # every minibatch, so on an accelerator the learning rate is kept as a
        # device tensor and handed to the fused optimizer, which reads it there.
        self.lr_tensor = None
        if self.device_type == 'cuda' and learning_rate_schedule == 'adaptive':
            try:
                self.lr_tensor = torch.tensor(float(learning_rate), device=self.device)
                self.optimizer = optim.Adam(self.trainable_parameters, lr=self.lr_tensor, fused=True)
                self.lr_decay = torch.tensor(1.0 / 1.2, device=self.device)
                self.lr_growth = torch.tensor(1.2, device=self.device)
                self.lr_hold = torch.tensor(1.0, device=self.device)
            except (RuntimeError, ValueError) as error:
                print(f'[RAISIM_GYM] Device-side learning rate unavailable, adapting on the host: {error}')
                self.lr_tensor = None
        if self.lr_tensor is None:
            self.optimizer = optim.Adam(self.trainable_parameters, lr=learning_rate)

        # env parameters
        self.num_transitions_per_env = num_transitions_per_env
        self.num_envs = num_envs

        # PPO parameters
        self.clip_param = clip_param
        self.num_learning_epochs = num_learning_epochs
        self.num_mini_batches = num_mini_batches
        self.value_loss_coef = value_loss_coef
        self.entropy_coef = entropy_coef
        self.gamma = gamma
        self.lam = lam
        self.max_grad_norm = max_grad_norm
        self.use_clipped_value_loss = use_clipped_value_loss

        # Log
        self.log_dir = os.path.join(log_dir, datetime.now().strftime('%b%d_%H-%M-%S'))
        self.writer = SummaryWriter(log_dir=self.log_dir, flush_secs=10)
        self.tot_timesteps = 0
        self.tot_time = 0

        # ADAM
        self.learning_rate = learning_rate
        self.desired_kl = desired_kl
        self.schedule = learning_rate_schedule

        # temps
        self.actions = None
        self.actions_log_prob = None
        self.actor_obs = None
        self.is_recurrent = getattr(self.actor.architecture, "is_recurrent", False)
        if self.is_recurrent:
            hidden_size = self.actor.architecture.hidden_size
            self.actor_hidden = torch.zeros(1, num_envs, hidden_size, device=self.device, dtype=torch.float32)
            self.critic_hidden = torch.zeros(1, num_envs, hidden_size, device=self.device, dtype=torch.float32)

        if compile_networks:
            self._compile_networks()

        # The captured rollout owns the actor observation buffer, so it needs the
        # critic to read the same one. A separate critic observation is only
        # available on the host, after the graph has already run.
        self.graph_rollout = None
        if (cuda_graph_rollout and self.device_type == 'cuda'
                and not self.is_recurrent and shared_observations):
            try:
                self.graph_rollout = CudaGraphRollout(actor, self.storage, self.device)
                self.storage.use_device_writes()
                print('[RAISIM_GYM] Rollout policy captured as a CUDA graph')
            except Exception as error:
                print(f'[RAISIM_GYM] CUDA graph capture failed; using the eager rollout: {error}')
                self.graph_rollout = None

    def _compile_networks(self):
        """Compile the networks used by the PPO update.

        Only the update is compiled: it runs a fixed, large minibatch shape many
        times per iteration. The rollout keeps the eager modules, which is what
        the captured graph replays.
        """
        if self.is_recurrent:
            return
        try:
            self.actor.set_compiled_architecture(torch.compile(self.actor.architecture.architecture))
            self.critic.set_compiled_architecture(torch.compile(self.critic.architecture.architecture))
        except Exception as error:
            print(f'[RAISIM_GYM] torch.compile unavailable; using eager updates: {error}')
            self.actor.set_compiled_architecture(None)
            self.critic.set_compiled_architecture(None)

    def act(self, actor_obs):
        with torch.no_grad():
            return self.act_inference(actor_obs)

    def act_inference(self, actor_obs):
        """Act while the caller owns a surrounding no-grad context."""
        self.actor_obs = actor_obs
        if self.graph_rollout is not None:
            # The graph writes the observation, action, mean and log probability
            # of this transition straight into the rollout buffers.
            self.actions = self.graph_rollout.act(actor_obs)
            return self.actions
        obs_tc = torch.from_numpy(actor_obs)
        if obs_tc.dtype != torch.float32 or self.device_type != 'cpu':
            obs_tc = obs_tc.to(self.device, dtype=torch.float32)
        if self.is_recurrent:
            if self.actor_hidden.dtype != obs_tc.dtype:
                self.actor_hidden = self.actor_hidden.to(dtype=obs_tc.dtype)
            self.actions, self.actions_log_prob, self.actor_hidden = \
                self.actor.sample_recurrent(obs_tc, self.actor_hidden)
            self.actor_hidden = self.actor_hidden.detach()
        else:
            self.actions, self.actions_log_prob = self.actor.sample(obs_tc)
        return self.actions

    def step(self, value_obs, rews, dones):
        if self.graph_rollout is not None:
            self.storage.record_step_rewards(rews, dones)
            self.storage.advance()
            return

        hidden_a = None
        hidden_c = None
        if self.is_recurrent:
            value_obs_tc = torch.from_numpy(value_obs).to(self.device, dtype=torch.float32)
            if self.critic_hidden.dtype != value_obs_tc.dtype:
                self.critic_hidden = self.critic_hidden.to(dtype=value_obs_tc.dtype)
            values, self.critic_hidden = self.critic.evaluate_recurrent(value_obs_tc, self.critic_hidden)
            self.critic_hidden = self.critic_hidden.detach()
            hidden_a = self.actor_hidden.detach()
            hidden_c = self.critic_hidden.detach()
        self.storage.add_transitions(self.actor_obs, value_obs, self.actions, self.actor.action_mean,
                                     self.actor.distribution.std_np, rews, dones,
                                     self.actions_log_prob, hidden_a, hidden_c)
        if self.is_recurrent:
            done_mask = torch.from_numpy(dones.astype(float)).to(self.device).view(1, -1, 1)
            self.actor_hidden = self.actor_hidden * (1.0 - done_mask)
            self.critic_hidden = self.critic_hidden * (1.0 - done_mask)

    def update(self, actor_obs, value_obs, log_this_iteration, update):
        self._bind_learning_rate()
        if self.is_recurrent:
            last_values = self._predict_recurrent_last_values(value_obs)
        else:
            last_values = self.critic.predict(torch.from_numpy(value_obs).to(self.device, dtype=torch.float32))

        # Learning step
        self.storage.compute_returns(last_values.to(self.device), self.critic, self.gamma, self.lam)
        if self.is_recurrent:
            mean_value_loss, mean_surrogate_loss, infos = self._train_step_recurrent(log_this_iteration)
        else:
            mean_value_loss, mean_surrogate_loss, infos = self._train_step(log_this_iteration)
        self.storage.clear()

        if log_this_iteration:
            self.log({**locals(), **infos, 'it': update})

    def _bind_learning_rate(self):
        """Keep the optimizer pointing at the device-side learning rate.

        Restoring a checkpoint through optimizer.load_state_dict() replaces the
        entry with the plain value it was saved as, which would silently freeze
        the adaptive schedule.
        """
        if self.lr_tensor is None:
            return
        for param_group in self.optimizer.param_groups:
            if param_group['lr'] is not self.lr_tensor:
                self.lr_tensor.fill_(float(param_group['lr']))
                param_group['lr'] = self.lr_tensor

    def current_learning_rate(self):
        """Read the learning rate, synchronizing only when it lives on the device."""
        return self.lr_tensor.item() if self.lr_tensor is not None else self.learning_rate

    def log(self, variables):
        self.tot_timesteps += self.num_transitions_per_env * self.num_envs
        mean_std = self.actor.distribution.std.mean()
        self.writer.add_scalar('PPO/value_function', variables['mean_value_loss'], variables['it'])
        self.writer.add_scalar('PPO/surrogate', variables['mean_surrogate_loss'], variables['it'])
        self.writer.add_scalar('PPO/mean_noise_std', mean_std.item(), variables['it'])
        self.writer.add_scalar('PPO/learning_rate', self.current_learning_rate(), variables['it'])

    def _train_step(self, log_this_iteration):
        mean_value_loss = 0
        mean_surrogate_loss = 0
        for epoch in range(self.num_learning_epochs):
            for actor_obs_batch, critic_obs_batch, actions_batch, old_sigma_batch, old_mu_batch, current_values_batch, advantages_batch, returns_batch, old_actions_log_prob_batch \
                    in self.batch_sampler(self.num_mini_batches):

                actions_log_prob_batch, entropy_batch = self.actor.evaluate(actor_obs_batch, actions_batch)
                value_batch = self.critic.evaluate(critic_obs_batch)

                # Adjusting the learning rate using KL divergence
                mu_batch = self.actor.action_mean
                sigma_batch = self.actor.distribution.std

                # KL
                if self.desired_kl != None and self.schedule == 'adaptive':
                    with torch.no_grad():
                        kl = torch.sum(
                            torch.log(sigma_batch / old_sigma_batch + 1.e-5) + (torch.square(old_sigma_batch) + torch.square(old_mu_batch - mu_batch)) / (2.0 * torch.square(sigma_batch)) - 0.5, axis=-1)
                        kl_mean = torch.mean(kl)

                        if self.lr_tensor is not None:
                            # Same schedule, applied on the device. Branching on
                            # kl_mean here instead would synchronize the host
                            # with the accelerator once per minibatch.
                            scale = torch.where(
                                kl_mean > self.desired_kl * 2.0,
                                self.lr_decay,
                                torch.where((kl_mean < self.desired_kl / 2.0) & (kl_mean > 0.0),
                                            self.lr_growth, self.lr_hold))
                            self.lr_tensor.mul_(scale).clamp_(1e-5, 1e-2)
                        else:
                            if kl_mean > self.desired_kl * 2.0:
                                self.learning_rate = max(1e-5, self.learning_rate / 1.2)
                            elif kl_mean < self.desired_kl / 2.0 and kl_mean > 0.0:
                                self.learning_rate = min(1e-2, self.learning_rate * 1.2)

                            for param_group in self.optimizer.param_groups:
                                param_group['lr'] = self.learning_rate

                # Surrogate loss
                ratio = torch.exp(actions_log_prob_batch - torch.squeeze(old_actions_log_prob_batch))
                surrogate = -torch.squeeze(advantages_batch) * ratio
                surrogate_clipped = -torch.squeeze(advantages_batch) * torch.clamp(ratio, 1.0 - self.clip_param,
                                                                                   1.0 + self.clip_param)
                surrogate_loss = torch.max(surrogate, surrogate_clipped).mean()

                # Value function loss
                if self.use_clipped_value_loss:
                    value_clipped = current_values_batch + (value_batch - current_values_batch).clamp(-self.clip_param,
                                                                                                    self.clip_param)
                    value_losses = (value_batch - returns_batch).pow(2)
                    value_losses_clipped = (value_clipped - returns_batch).pow(2)
                    value_loss = torch.max(value_losses, value_losses_clipped).mean()
                else:
                    value_loss = (returns_batch - value_batch).pow(2).mean()

                loss = surrogate_loss + self.value_loss_coef * value_loss
                if self.entropy_coef:
                    loss = loss - self.entropy_coef * entropy_batch.mean()

                # Gradient step
                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.trainable_parameters, self.max_grad_norm)
                self.optimizer.step()

                if log_this_iteration:
                    mean_value_loss += value_loss.item()
                    mean_surrogate_loss += surrogate_loss.item()

        if log_this_iteration:
            num_updates = self.num_learning_epochs * self.num_mini_batches
            mean_value_loss /= num_updates
            mean_surrogate_loss /= num_updates

        return mean_value_loss, mean_surrogate_loss, locals()

    def _predict_recurrent_last_values(self, value_obs):
        obs_tc = torch.from_numpy(value_obs).to(self.device)
        hidden = torch.zeros(1, self.num_envs, self.critic.architecture.hidden_size, device=self.device)
        values, _ = self.critic.predict_recurrent(obs_tc, hidden)
        return values

    def _train_step_recurrent(self, log_this_iteration):
        mean_value_loss = 0
        mean_surrogate_loss = 0

        generator = self.storage.recurrent_mini_batch_generator(self.num_mini_batches, self.num_learning_epochs)

        for (actor_obs_batch,
             critic_obs_batch,
             actions_batch,
             values_batch,
             advantages_batch,
             returns_batch,
             old_actions_log_prob_batch,
             hidden_states_batch,
             masks_batch) in generator:

            hidden_a, hidden_c = hidden_states_batch
            logits_seq, _ = self.actor.architecture.architecture(actor_obs_batch, hidden_a)
            values_seq, _ = self.critic.architecture.architecture(critic_obs_batch, hidden_c)

            mask_flat = masks_batch.reshape(-1)
            if not torch.any(mask_flat):
                continue

            act_dim = actions_batch.shape[-1]
            logits_flat = logits_seq.reshape(-1, act_dim)[mask_flat]
            actions_flat = actions_batch.reshape(-1, act_dim)[mask_flat]
            old_logp_flat = old_actions_log_prob_batch.reshape(-1)[mask_flat]
            adv_flat = advantages_batch.reshape(-1)[mask_flat]
            returns_flat = returns_batch.reshape(-1)[mask_flat]
            current_values_flat = values_batch.reshape(-1)[mask_flat]
            values_flat = values_seq.reshape(-1)[mask_flat]

            logp_flat, entropy_flat = self.actor.distribution.evaluate(logits_flat, actions_flat)

            ratio = torch.exp(logp_flat - old_logp_flat)
            surrogate = -adv_flat * ratio
            surrogate_clipped = -adv_flat * torch.clamp(ratio, 1.0 - self.clip_param, 1.0 + self.clip_param)
            surrogate_loss = torch.max(surrogate, surrogate_clipped).mean()

            if self.use_clipped_value_loss:
                value_clipped = current_values_flat + (values_flat - current_values_flat).clamp(-self.clip_param, self.clip_param)
                value_losses = (values_flat - returns_flat).pow(2)
                value_losses_clipped = (value_clipped - returns_flat).pow(2)
                value_loss = torch.max(value_losses, value_losses_clipped).mean()
            else:
                value_loss = (returns_flat - values_flat).pow(2).mean()

            loss = surrogate_loss + self.value_loss_coef * value_loss
            if self.entropy_coef:
                loss = loss - self.entropy_coef * entropy_flat.mean()

            self.optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(self.trainable_parameters, self.max_grad_norm)
            self.optimizer.step()

            if log_this_iteration:
                mean_value_loss += value_loss.item()
                mean_surrogate_loss += surrogate_loss.item()

        if log_this_iteration:
            mean_value_loss /= self.num_learning_epochs
            mean_surrogate_loss /= self.num_learning_epochs

        return mean_value_loss, mean_surrogate_loss, locals()
