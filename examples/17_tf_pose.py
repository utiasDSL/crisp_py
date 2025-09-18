from crisp_py.robot import Robot
from crisp_py.robot_config import RobotConfig
import time


config = RobotConfig(
    joint_names=[
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ],
    home_config=[0, -1.57, 1.57, 0, 1.57, 0, 0],
    target_frame="panda_hand_tcp",
    base_frame="panda_link0",
    use_tf_pose=True,
    tf_retrieve_rate=50.0,
)

robot = Robot(robot_config=config)

robot.wait_until_ready()


print(f"Current pose from TF: {robot.end_effector_pose}")
print(f"Current pose after move: {robot.end_effector_pose}")


