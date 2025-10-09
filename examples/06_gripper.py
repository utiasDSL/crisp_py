"""Simple example to control the gripper."""

# %%
import time


from crisp_py.gripper.gripper import make_gripper

gripper = make_gripper("gripper_franka")
gripper.wait_until_ready()

# %%
freq = 1.0
rate = gripper.node.create_rate(freq)
t = 0.0
while t < 10.0:
    print(gripper.value)
    print(gripper.torque)
    rate.sleep()
    t += 1.0 / freq

# %%
gripper.value

# Almost fully open
gripper.set_target(0.9)

time.sleep(3.0)

# Almost fully closed
gripper.set_target(0.1)

# %%
try:
    gripper.reboot()
except RuntimeError as e:
    print(e)

# %%
try:
    gripper.enable_torque()
except RuntimeError as e:
    print(e)

# %%
try:
    gripper.disable_torque()
except RuntimeError as e:
    print(e)
