"""Simple example using dual arm setup.

Note: for a better dual arm teleop setup, check the next example.
"""

# %%
from crisp_py.robot import Robot
import numpy as np

# %%
left_arm = Robot(namespace="left")
right_arm = Robot(namespace="right")
left_arm.wait_until_ready()
right_arm.wait_until_ready()

# %%
print(left_arm.end_effector_pose)
print(right_arm.joint_values)

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


right_arm.node.create_timer(1.0 / 50.0, lambda: sync(left_arm, right_arm))

# %%
max_time = 7.0
ctrl_freq = 50.0
rate = left_arm.node.create_rate(ctrl_freq)

t = 0.0
while t < max_time:
    x, y, z = left_arm.end_effector_pose.translation
    x += -np.sin(t) * 0.1
    left_arm.set_target(position=[x, y, z])
    rate.sleep()
    t += 1.0 / ctrl_freq
