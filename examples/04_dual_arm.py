"""Simple example using dual arm setup.

Note: for a better dual arm teleop setup, it would be better to simply map the /left/joint_states
to the /right/joints_states. But this setup is a bit easier to understand and play with.
"""

# %%
from crisp_py.robot import Robot
from crisp_py.robot_config import FrankaConfig

# %%
faster_publishing_config = FrankaConfig()
faster_publishing_config.publish_frequency = 20.0

left_arm = Robot(robot_config=faster_publishing_config, namespace="left")
right_arm = Robot(robot_config=faster_publishing_config, namespace="right")
left_arm.wait_until_ready()
right_arm.wait_until_ready()

# %%
left_arm.cartesian_controller_parameters_client.load_param_config(
    file_path="config/control/gravity_compensation.yaml"
)
right_arm.cartesian_controller_parameters_client.load_param_config(
    file_path="config/control/joint_control.yaml"
)

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
    right_arm.set_target_joint(left_arm.joint_values)


right_arm.node.create_timer(1.0 / 100.0, lambda: sync(left_arm, right_arm))
