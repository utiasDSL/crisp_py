"""Simple example using dual arm setup.

Note: for a better dual arm teleop setup, it would be better to simply map the /left/joint_states
to the /right/joints_states. But this setup is a bit easier to understand and play with.
"""

# %%
from crisp_py.robot import Robot
from crisp_py.robot_config import FrankaConfig
import numpy as np

# %%
faster_publishing_config = FrankaConfig()
faster_publishing_config.publish_frequency = 200.0

left_arm = Robot(robot_config=faster_publishing_config, namespace="left")
right_arm = Robot(robot_config=faster_publishing_config, namespace="right")
left_arm.wait_until_ready()
right_arm.wait_until_ready()

# %%
left_arm.cartesian_controller_parameters_client.save_param_config(file_path="data.yaml")
# %%
left_arm.cartesian_controller_parameters_client.load_param_config(file_path="config/gravity_compensation.yaml")

# %%
print(left_arm.end_effector_pose)
print(right_arm.end_effector_pose)

# %%
print("Going to home position...")
left_arm.home()
right_arm.home()

# %%

left_arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")
right_arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")

# %%
def sync(left_arm, right_arm):
    right_arm.set_target(pose=left_arm.end_effector_pose)


right_arm.node.create_timer(1.0 / 100.0, lambda: sync(left_arm, right_arm))

# %%
max_time = 7.0
ctrl_freq = 50.0
rate = left_arm.node.create_rate(ctrl_freq)

t = 0.0
while t < max_time:
    x, y, z = left_arm.end_effector_pose.translation
    x += -np.sin(t) * 0.03
    left_arm.set_target(position=[x, y, z])
    rate.sleep()
    t += 1.0 / ctrl_freq

# %%

params = [
    ("task.k_pos_x", 0.0),
    ("task.k_pos_y", 0.0),
    ("task.k_pos_z", 0.0),
    ("task.k_rot_x", 0.0),
    ("task.k_rot_y", 0.0),
    ("task.k_rot_z", 0.0),
    ("nullspace.stiffness", 0.0),
]
left_arm.cartesian_controller_parameters_client.set_parameters(params)
