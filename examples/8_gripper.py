"""Simple example to control the gripper."""

# %%
from crisp_py.gripper.gripper import Gripper

leader_gripper = Gripper(namespace="leader")
follower_gripper = Gripper(namespace="follower")
leader_gripper.wait_until_ready()
follower_gripper.wait_until_ready()

# %%
print(follower_gripper.width)
print(leader_gripper.width)


# %%
follower_gripper.set_target(leader_gripper.width)

# %%

freq = 50.0
rate = follower_gripper.node.create_rate(freq)
dt = 1.0 / freq

t = 0.0
while True: 
    follower_gripper.set_target(leader_gripper.width)
    rate.sleep()
    t += dt

