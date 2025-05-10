import gym
import time
import numpy as np
import pinocchio as pin
from typing import Optional, Tuple
from scipy.spatial.transform import Rotation

from crisp_py.robot import Robot
from crisp_py.gripper.gripper import Gripper
from crisp_py.devices.camera import Camera
from crisp_py.envs.franka_env_config import FrankaEnvConfig

class FrankaBaseEnv(gym.Env):
    """Base class for Franka Gym Environment.
    This class serves as a base for creating specific Franka Gym environments.
    It cannot be used directly.
    """

    def __init__(self, env_config: Optional[FrankaEnvConfig] = None):
        """Initialize the Franka Gym Environment.

        Args:
            env_config (FrankaEnvConfig): Configuration for the environment.
        """
        super().__init__()
        self.env_config = FrankaEnvConfig() if env_config is None else env_config

        self.robot = Robot(namespace=self.env_config.namespace, robot_config=self.env_config.robot_config)
        self.gripper = Gripper(self.robot.node, 
                               namespace=self.env_config.namespace, 
                               gripper_config=self.env_config.gripper_config, 
                               spin_node=False)
        self.cameras = [
            Camera(self.robot.node, 
                   namespace=self.env_config.namespace, 
                   camera_config=camera_config, 
                   spin_node=False) for camera_config in self.env_config.camera_configs
        ]

        self.gripper_is_closing = False

        self.robot.wait_until_ready()
        self.gripper.wait_until_ready()
        for camera in self.cameras:
            camera.wait_until_ready()

        self.control_rate = self.robot.node.create_rate(self.env_config.control_frequency)

        self.observation_space = gym.spaces.Dict(
            {
                **{
                    f'{camera.camera_config.camera_name}_image': gym.spaces.Box(
                        low=np.zeros((*camera.camera_config.resolution, 3), dtype=np.uint8),
                        high=255 * np.ones((*camera.camera_config.resolution, 3), dtype=np.uint8),
                        dtype=np.uint8,
                    )
                    for camera in self.cameras
                },
                "proprio_joint": gym.spaces.Box(
                    low=np.ones((7, ), dtype=np.float32) * -np.pi,
                    high=np.ones((7, ), dtype=np.float32) * np.pi,
                    dtype=np.float32
                ),
                "proprio_cartesian": gym.spaces.Box(
                    low=-np.ones((6, ), dtype=np.float32),
                    high=np.ones((6, ), dtype=np.float32),
                    dtype=np.float32
                ),
                "proprio_gripper": gym.spaces.Box(
                    low=np.zeros((1, ), dtype=np.float32),
                    high=np.ones((1, ), dtype=np.float32),
                    dtype=np.float32
                ),
            }
        )

    def _get_obs(self) -> dict:
        """Get the current observation from the robot.

        Returns:
            dict: Current observation.
        """
        obs = {}
        for camera in self.cameras:
            obs[f'{camera.camera_config.camera_name}_image'] = camera.current_image
        obs["proprio_joint"] = self.robot.joint_values
        obs["proprio_cartesian"] = self.robot.end_effector_pose
        obs["proprio_gripper"] = (self.gripper.max_width - self.gripper.width) / (self.gripper.max_width - self.gripper.min_width)
        return obs
    
    def _set_gripper_action(self, action: float):
        if action >= self.env_config.gripper_threshold and self.gripper.is_open() and not self.gripper_is_closing:
            self.gripper.close()
            self.gripper_is_closing = True
        elif action < self.env_config.gripper_threshold and not self.gripper.is_open() and self.gripper_is_closing:
            self.gripper.open()
            self.gripper_is_closing = False
    
    def reset(self) -> Tuple[dict, dict]:
        """Reset the environment."""
        super().reset()

        return self._get_obs(), {}

    def close(self):
        """Close the environment."""
        self.robot.close()
        super().close()


class FrankaCartesianEnv(FrankaBaseEnv):
    """Franka Cartesian Environment.

    This class is a specific implementation of the Franka Gym Environment for Cartesian space control.
    """
    def __init__(self, env_config: Optional[FrankaEnvConfig] = None):
        """Initialize the Franka Cartesian Environment.

        Args:
            env_config (FrankaEnvConfig): Configuration for the environment.
        """
        super().__init__(env_config)

        self._min_z_height = 0.0

        self.action_space = gym.spaces.Box(
            low=np.concatenate([
                -np.ones((3, ), dtype=np.float32),
                -np.ones((3, ), dtype=np.float32) * np.pi,
                np.zeros((1, ), dtype=np.float32)
            ], axis=1),
            high=np.concatenate([
                np.ones((3, ), dtype=np.float32),
                np.ones((3, ), dtype=np.float32) * np.pi,
                np.ones((1, ), dtype=np.float32)
            ], axis=1),
            dtype=np.float32
        )

        self.robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

    def step(self, action: np.ndarray, block: bool = True) -> Tuple[dict, float, bool, bool, dict]:
        reward = 0.0
        terminated = False
        truncated = False
        info = {}

        assert action.shape == self.action_space.shape, f"Action shape {action.shape} does not match expected shape {self.action_space.shape}"
        assert self.action_space.contains(action), f"Action {action} is not in the action space {self.action_space}"

        target_pose = self.robot.target_pose
        current_position, current_orientation = target_pose.translation, Rotation.from_quat(pin.Quaternion(target_pose.rotation))
        translation, rotation = action[:3], Rotation.from_euler('xyz', action[3:6])

        target_position = current_position + translation
        target_position[2] = max(target_position[2], self._min_z_height)
        target_orientation = rotation * current_orientation

        target_pose = pin.SE3(quat=target_orientation.as_quat(), translation=target_position)
        self.robot.set_target(pose=target_pose)

        self._set_gripper_action(action[6])

        if block:
            self.control_rate.sleep()

        obs = self._get_obs()

        return obs, reward, terminated, truncated, info

class FrankaJointEnv(FrankaBaseEnv):
    """Franka Cartesian Environment.

    This class is a specific implementation of the Franka Gym Environment for Joint space control.
    """

    def __init__(self, env_config: Optional[FrankaEnvConfig] = None):
        """Initialize the Franka Joint Environment.

        Args:
            env_config (FrankaEnvConfig): Configuration for the environment.
        """
        super().__init__(env_config)

        self.action_space = gym.spaces.Box(
            low=np.concatenate([
                np.ones((7, ), dtype=np.float32) * -np.pi,
                np.zeros((1, ), dtype=np.float32)
            ], axis=1),
            high=np.concatenate([
                np.ones((7, ), dtype=np.float32) * np.pi,
                np.ones((1, ), dtype=np.float32)
            ], axis=1),
            dtype=np.float32
        )

        self.robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

    def step(self, action: np.ndarray, block: bool = True) -> Tuple[dict, float, bool, bool, dict]:
        reward = 0.0
        terminated = False
        truncated = False
        info = {}

        assert action.shape == self.action_space.shape, f"Action shape {action.shape} does not match expected shape {self.action_space.shape}"
        assert self.action_space.contains(action), f"Action {action} is not in the action space {self.action_space}"

        current_joint = self.robot.joint_values
        target_joint = (current_joint + action[:7] + np.pi) % (2 * np.pi) - np.pi

        self.robot.set_target_joint(target_joint)

        self._set_gripper_action(action[7])

        if block:
            self.control_rate.sleep()

        obs = self._get_obs()

        return obs, reward, terminated, truncated, info

