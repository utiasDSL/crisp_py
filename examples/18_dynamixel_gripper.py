import time
from crisp_py.gripper.gripper import make_gripper

gripper = make_gripper("gripper_aloha")
gripper.wait_until_ready()

gripper.open()
time.sleep(3.0)

gripper.close()
time.sleep(3.0)

gripper.set_target(0.5)
time.sleep(3.0)


gripper.shutdown()
