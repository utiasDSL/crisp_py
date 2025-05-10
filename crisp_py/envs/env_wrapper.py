import gym
import numpy as np
from typing import Tuple

from crisp_py.envs.franka_env import FrankaBaseEnv

def stack_gym_space(space: gym.Space, repeat: int):
    """Repeat a Gym space definition."""

    if isinstance(space, gym.spaces.Box):
        return gym.spaces.Box(
            low=np.repeat(space.low[None], repeat, axis=0),
            high=np.repeat(space.high[None], repeat, axis=0),
            dtype=space.dtype,
        )
    elif isinstance(space, gym.spaces.Dict):
        return gym.spaces.Dict(
            {k: stack_gym_space(v, repeat) for k, v in space.spaces.items()}
        )
    else:
        raise ValueError(f"Space {space} is not supported.")

class WindowWrapper(gym.Wrapper):
    """
    A Gym wrapper that stacks a fixed-size window of past observations along a new time dimension.
    This allows agents to receive a temporal context of the environment by maintaining a history
    of the most recent `window_size` observations.
    """
    def __init__(self, env: FrankaBaseEnv, window_size: int):
        super().__init__(env)
        self.window_size = window_size
        self.window = []
        self.observation_space = stack_gym_space(self.env.observation_space, self.window_size)
    
    def step(self, action, **kwargs) -> Tuple[dict, float, bool, bool, dict]:
        obs, reward, terminated, truncated, info = self.env.step(action, **kwargs)
        self.window.append(obs)
        self.window = self.window[-self.window_size:]
        obs = {key: np.stack([frame[key] for frame in self.window]) for key in self.window[0].keys()}
        return obs, reward, terminated, truncated, info
    
    def reset(self) -> Tuple[dict, dict]:
        obs, info = self.env.reset()
        self.window = [obs] * self.window_size
        obs = {key: np.stack([frame[key] for frame in self.window]) for key in self.window[0].keys()}
        return obs, info
    
class RecedingHorizon(gym.Wrapper):
    def __init__(self, env: FrankaBaseEnv, horizon_length: int):
        super().__init__(env)

        self.horizon_length = horizon_length

        self.action_space = stack_gym_space(self.env.action_space, self.horizon_length)

    def step(self, action: np.ndarray, **kwargs) -> Tuple[dict, float, bool, bool, dict]:
        obs = {}
        rewards = []
        terminated = False
        truncated = False
        info = {}

        if self.horizon_length == 1 and len(actions.shape) == 1:
            actions = actions[None]
        assert action.shape[0] >= self.horizon_length

        for i in range(self.horizon_length):
            obs, reward, terminated, truncated, info = self.env.step(action[i], **kwargs)

            rewards.append(reward)

            if terminated or truncated:
                break

        return obs, np.sum(rewards), terminated, truncated, info