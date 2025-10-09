"""Example controlling the joints."""
import time

from crisp_py.robot import make_robot

robot = make_robot("fr3")
robot.wait_until_ready()

# %%
robot.controller_switcher_client.switch_controller("joint_impedance_controller")

# %%
q = robot.joint_values
q[0] += 0.2
robot.set_target_joint(q)

time.sleep(1.0)

robot.shutdown()
