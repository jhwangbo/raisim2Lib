import torch
from torch.utils.data.sampler import BatchSampler, SubsetRandomSampler
import numpy as np


def split_and_pad_trajectories(tensor: torch.Tensor, dones: torch.Tensor):
    """Split trajectories at done indices and pad to max length. Returns (padded, masks)."""
    dones = dones.clone()
    dones[-1] = 1
    # flatten dones in env-major order
    flat_dones = dones.transpose(1, 0).reshape(-1, 1)
    done_indices = torch.cat((flat_dones.new_tensor([-1], dtype=torch.int64), flat_dones.nonzero()[:, 0]))
    trajectory_lengths = done_indices[1:] - done_indices[:-1]
    trajectory_lengths_list = trajectory_lengths.tolist()

    trajectories = torch.split(tensor.transpose(1, 0).flatten(0, 1), trajectory_lengths_list)
    trajectories = (*trajectories, torch.zeros(tensor.shape[0], *tensor.shape[2:], device=tensor.device))
    padded = torch.nn.utils.rnn.pad_sequence(trajectories)
    padded = padded[:, :-1]

    masks = trajectory_lengths > torch.arange(0, tensor.shape[0], device=tensor.device).unsqueeze(1)
    return padded, masks


class RolloutStorage:
    """Rollout buffer whose per-transition tensors live on the training device.

    Every consumer of a transition -- the PPO update and the value pass -- runs
    on the device, so the buffer is allocated there and the update no longer
    re-uploads it (47 MB per update with 400 environments and 400 transitions).

    How a transition gets in depends on the producer. A rollout policy that runs
    on the device writes into the buffers itself; call use_device_writes() and
    the storage keeps no host copy of them. Any other producer hands over host
    arrays, which are staged in pinned memory and uploaded once per update:
    per-step uploads of a few tens of kilobytes are dominated by launch latency
    and cost several times more than a single bulk transfer.

    Rewards, dones and the generalized-advantage outputs always stay in host
    memory, because the advantage recurrence runs natively on the CPU.
    """

    def __init__(self, num_envs, num_transitions_per_env, actor_obs_shape, critic_obs_shape,
                 actions_shape, device, shared_observations=False, returns_calculator=None):
        self.device = device
        self.device_type = torch.device(device).type
        self.shared_observations = shared_observations
        self.returns_calculator = returns_calculator
        if shared_observations and actor_obs_shape != critic_obs_shape:
            raise ValueError("shared actor/critic observations must have identical shapes")

        self.num_transitions_per_env = num_transitions_per_env
        self.num_envs = num_envs
        self.host_buffers = {}

        # Core
        self.actor_obs, self.actor_obs_tc = self._staged_buffer('actor_obs', *actor_obs_shape)
        if shared_observations:
            self.critic_obs, self.critic_obs_tc = self.actor_obs, self.actor_obs_tc
        else:
            self.critic_obs, self.critic_obs_tc = self._staged_buffer('critic_obs', *critic_obs_shape)
        self.actions, self.actions_tc = self._staged_buffer('actions', *actions_shape)

        # For PPO
        self.actions_log_prob, self.actions_log_prob_tc = self._staged_buffer('actions_log_prob', 1)
        self.mu, self.mu_tc = self._staged_buffer('mu', *actions_shape)
        self.sigma, self.sigma_tc = self._staged_buffer('sigma', *actions_shape)

        # The transition fields a device-side rollout policy can write itself.
        # A shared critic observation is an alias, not a buffer of its own.
        self.transition_fields = ['actor_obs', 'actions', 'actions_log_prob', 'mu', 'sigma']
        if not shared_observations:
            self.transition_fields.append('critic_obs')

        # Read and written by the host-side advantage recurrence.
        self.rewards = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)
        self.dones = np.zeros([num_transitions_per_env, num_envs, 1], dtype=bool)
        self.values, self.values_tc = self._staged_buffer('values', 1)
        self.returns, self.returns_tc = self._staged_buffer('returns', 1)
        self.advantages, self.advantages_tc = self._staged_buffer('advantages', 1)

        # saved hidden states for recurrent policies
        self.saved_hidden_state_a = None
        self.saved_hidden_state_c = None

        self.step = 0

    def _staged_buffer(self, name, *trailing):
        """Allocate one buffer as a host/device pair.

        On a CPU device the two share storage, so staging costs nothing.
        """
        shape = (self.num_transitions_per_env, self.num_envs, *trailing)
        if self.device_type == 'cpu':
            host = torch.zeros(*shape, dtype=torch.float32)
            device_tensor = host
        else:
            host = torch.zeros(*shape, dtype=torch.float32, pin_memory=True)
            device_tensor = torch.zeros(*shape, dtype=torch.float32, device=self.device)
        self.host_buffers[name] = host
        return host.numpy(), device_tensor

    def use_device_writes(self):
        """Declare that transitions are produced directly in the device buffers.

        Releases the host staging of every field the producer writes, so nothing
        is uploaded before an update.
        """
        for name in self.transition_fields:
            self.host_buffers.pop(name, None)
            setattr(self, name, None)
        self.transition_fields = []
        self.critic_obs = None

    def add_transitions(self, actor_obs, critic_obs, actions, mu, sigma, rewards, dones, actions_log_prob,
                        hidden_state_a=None, hidden_state_c=None):
        if self.step >= self.num_transitions_per_env:
            raise AssertionError("Rollout buffer overflow")
        if self.actor_obs is None:
            raise RuntimeError("this storage expects transitions to be written on the device")
        if self.shared_observations:
            if actor_obs is not critic_obs:
                raise ValueError("shared observations require the same actor and critic input")
        else:
            self.critic_obs[self.step] = critic_obs
        self.actor_obs[self.step] = actor_obs
        self.actions[self.step] = actions
        self.mu[self.step] = mu
        self.sigma[self.step] = sigma
        self.actions_log_prob[self.step] = np.asarray(actions_log_prob).reshape(-1, 1)
        self.record_step_rewards(rewards, dones)
        self._save_hidden_states(hidden_state_a, hidden_state_c)
        self.step += 1

    def record_step_rewards(self, rewards, dones):
        """Record the host-side signals of the current transition.

        Under use_device_writes() the rest of the transition is written by the
        rollout policy, which already holds it on the device.
        """
        self.rewards[self.step] = rewards.reshape(-1, 1)
        self.dones[self.step] = dones.reshape(-1, 1)

    def advance(self):
        self.step += 1

    def clear(self):
        self.step = 0

    def _save_hidden_states(self, hidden_state_a, hidden_state_c):
        if hidden_state_a is None and hidden_state_c is None:
            return

        if hidden_state_a is not None and not isinstance(hidden_state_a, tuple):
            hidden_state_a = (hidden_state_a,)
        if hidden_state_c is not None and not isinstance(hidden_state_c, tuple):
            hidden_state_c = (hidden_state_c,)

        if self.saved_hidden_state_a is None and hidden_state_a is not None:
            self.saved_hidden_state_a = [
                torch.zeros(self.num_transitions_per_env, *h.shape, device=self.device) for h in hidden_state_a
            ]
        if self.saved_hidden_state_c is None and hidden_state_c is not None:
            self.saved_hidden_state_c = [
                torch.zeros(self.num_transitions_per_env, *h.shape, device=self.device) for h in hidden_state_c
            ]

        if hidden_state_a is not None:
            for i in range(len(hidden_state_a)):
                self.saved_hidden_state_a[i][self.step].copy_(hidden_state_a[i])
        if hidden_state_c is not None:
            for i in range(len(hidden_state_c)):
                self.saved_hidden_state_c[i][self.step].copy_(hidden_state_c[i])

    def compute_returns(self, last_values, critic, gamma, lam):
        if self.device_type != 'cpu':
            for name in self.transition_fields:
                getattr(self, name + '_tc').copy_(self.host_buffers[name], non_blocking=True)

        with torch.no_grad():
            if getattr(critic.architecture, "is_recurrent", False):
                hidden = torch.zeros(1,
                                     self.num_envs,
                                     critic.architecture.hidden_size,
                                     device=self.device,
                                     dtype=torch.float32)
                dones_tc = torch.from_numpy(self.dones).to(self.device, dtype=torch.float32)
                for t in range(self.num_transitions_per_env):
                    hidden = hidden * (1.0 - dones_tc[t].view(1, -1, 1))
                    val_t, hidden = critic.predict_recurrent(self.critic_obs_tc[t], hidden)
                    self.values_tc[t].copy_(val_t.view(-1, 1))
            else:
                self.values_tc.copy_(critic.predict(self.critic_obs_tc).view_as(self.values_tc))

        if self.device_type != 'cpu':
            # One device-to-host transfer feeds the native advantage recurrence.
            self.host_buffers['values'].copy_(self.values_tc)

        if self.returns_calculator is not None:
            self.returns_calculator(
                self.rewards, self.dones, self.values, last_values.cpu().numpy(),
                self.returns, gamma, lam)
        else:
            advantage = 0

            for step in reversed(range(self.num_transitions_per_env)):
                if step == self.num_transitions_per_env - 1:
                    next_values = last_values.cpu().numpy()
                else:
                    next_values = self.values[step + 1]

                next_is_not_terminal = 1.0 - self.dones[step]
                delta = self.rewards[step] + next_is_not_terminal * gamma * next_values - self.values[step]
                advantage = delta + next_is_not_terminal * gamma * lam * advantage
                self.returns[step] = advantage + self.values[step]

        # Compute and normalize the advantages
        np.subtract(self.returns, self.values, out=self.advantages)
        self.advantages -= self.advantages.mean()
        self.advantages /= (self.advantages.std() + 1e-8)

        if self.device_type != 'cpu':
            self.returns_tc.copy_(self.host_buffers['returns'], non_blocking=True)
            self.advantages_tc.copy_(self.host_buffers['advantages'], non_blocking=True)

    def recurrent_mini_batch_generator(self, num_mini_batches, num_epochs=8):
        dones = torch.from_numpy(self.dones).to(self.device)

        padded_actor_obs, masks = split_and_pad_trajectories(self.actor_obs_tc, dones)
        padded_critic_obs, _ = split_and_pad_trajectories(self.critic_obs_tc, dones)
        padded_actions, _ = split_and_pad_trajectories(self.actions_tc, dones)
        padded_old_logp, _ = split_and_pad_trajectories(self.actions_log_prob_tc, dones)
        padded_adv, _ = split_and_pad_trajectories(self.advantages_tc, dones)
        padded_returns, _ = split_and_pad_trajectories(self.returns_tc, dones)
        padded_values, _ = split_and_pad_trajectories(self.values_tc, dones)

        num_mini_batches = max(1, min(num_mini_batches, self.num_envs))
        mini_batch_size = max(1, self.num_envs // num_mini_batches)
        for _ in range(num_epochs):
            first_traj = 0
            for i in range(num_mini_batches):
                start = i * mini_batch_size
                stop = (i + 1) * mini_batch_size

                dones_t = dones.squeeze(-1)
                last_was_done = torch.zeros_like(dones_t, dtype=torch.bool)
                last_was_done[1:] = dones_t[:-1]
                last_was_done[0] = True
                trajectories_batch_size = torch.sum(last_was_done[:, start:stop])
                last_traj = first_traj + trajectories_batch_size

                masks_batch = masks[:, first_traj:last_traj]
                actor_obs_batch = padded_actor_obs[:, first_traj:last_traj]
                critic_obs_batch = padded_critic_obs[:, first_traj:last_traj]
                actions_batch = padded_actions[:, first_traj:last_traj]
                old_logp_batch = padded_old_logp[:, first_traj:last_traj]
                advantages_batch = padded_adv[:, first_traj:last_traj]
                returns_batch = padded_returns[:, first_traj:last_traj]
                values_batch = padded_values[:, first_traj:last_traj]

                hidden_state_a_batch = None
                hidden_state_c_batch = None
                if self.saved_hidden_state_a is not None:
                    hidden_state_a_batch = [
                        saved_hidden_state.permute(2, 0, 1, 3)[last_was_done.transpose(1, 0)]
                        [first_traj:last_traj]
                        .transpose(1, 0)
                        .contiguous()
                        for saved_hidden_state in self.saved_hidden_state_a
                    ]
                    hidden_state_a_batch = hidden_state_a_batch[0] if len(hidden_state_a_batch) == 1 else hidden_state_a_batch
                if self.saved_hidden_state_c is not None:
                    hidden_state_c_batch = [
                        saved_hidden_state.permute(2, 0, 1, 3)[last_was_done.transpose(1, 0)]
                        [first_traj:last_traj]
                        .transpose(1, 0)
                        .contiguous()
                        for saved_hidden_state in self.saved_hidden_state_c
                    ]
                    hidden_state_c_batch = hidden_state_c_batch[0] if len(hidden_state_c_batch) == 1 else hidden_state_c_batch

                yield (
                    actor_obs_batch,
                    critic_obs_batch,
                    actions_batch,
                    values_batch,
                    advantages_batch,
                    returns_batch,
                    old_logp_batch,
                    (hidden_state_a_batch, hidden_state_c_batch),
                    masks_batch,
                )

                first_traj = last_traj

    def mini_batch_generator_shuffle(self, num_mini_batches):
        batch_size = self.num_envs * self.num_transitions_per_env
        mini_batch_size = batch_size // num_mini_batches

        for indices in BatchSampler(SubsetRandomSampler(range(batch_size)), mini_batch_size, drop_last=True):
            actor_obs_batch = self.actor_obs_tc.view(-1, *self.actor_obs_tc.size()[2:])[indices]
            critic_obs_batch = self.critic_obs_tc.view(-1, *self.critic_obs_tc.size()[2:])[indices]
            actions_batch = self.actions_tc.view(-1, self.actions_tc.size(-1))[indices]
            sigma_batch = self.sigma_tc.view(-1, self.sigma_tc.size(-1))[indices]
            mu_batch = self.mu_tc.view(-1, self.mu_tc.size(-1))[indices]
            values_batch = self.values_tc.view(-1, 1)[indices]
            returns_batch = self.returns_tc.view(-1, 1)[indices]
            old_actions_log_prob_batch = self.actions_log_prob_tc.view(-1, 1)[indices]
            advantages_batch = self.advantages_tc.view(-1, 1)[indices]
            yield actor_obs_batch, critic_obs_batch, actions_batch, sigma_batch, mu_batch, values_batch, advantages_batch, returns_batch, old_actions_log_prob_batch

    def mini_batch_generator_inorder(self, num_mini_batches):
        batch_size = self.num_envs * self.num_transitions_per_env
        mini_batch_size = batch_size // num_mini_batches

        for batch_id in range(num_mini_batches):
            yield self.actor_obs_tc.view(-1, *self.actor_obs_tc.size()[2:])[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.critic_obs_tc.view(-1, *self.critic_obs_tc.size()[2:])[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.actions_tc.view(-1, self.actions_tc.size(-1))[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.sigma_tc.view(-1, self.sigma_tc.size(-1))[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.mu_tc.view(-1, self.mu_tc.size(-1))[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.values_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.advantages_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.returns_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.actions_log_prob_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size]
