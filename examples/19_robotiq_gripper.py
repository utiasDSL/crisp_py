import time

from crisp_py.gripper import make_gripper


gripper = make_gripper("gripper_robotiq_2f85")

# %%

gripper.wait_until_ready()

# %%
gripper.open()
time.sleep(3.0)

gripper.close()
time.sleep(3.0)

gripper.set_target(0.5)
time.sleep(3.0)


gripper.shutdown()
