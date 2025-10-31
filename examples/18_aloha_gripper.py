import time
from crisp_py.gripper.gripper import Gripper, GripperConfig

gripper_cfg = GripperConfig.from_yaml("/home/linusschwarz/crisp_configs/aloha-gripper.yaml")
gripper = Gripper(gripper_config=gripper_cfg)
gripper.wait_until_ready()

gripper.open()
time.sleep(3.0)

gripper.close()
time.sleep(3.0)

gripper.set_target(0.5)
time.sleep(3.0)


gripper.shutdown()
