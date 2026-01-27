"""Simple example to control the gripper."""
import time

from crisp_py.gripper.gripper import make_gripper

gripper = make_gripper("gripper_franka")
gripper.config.max_delta = 0.15
gripper.wait_until_ready()

# Open and close the gripper
gripper.open()
time.sleep(2.0)
gripper.close()
time.sleep(2.0)
gripper.shutdown()
