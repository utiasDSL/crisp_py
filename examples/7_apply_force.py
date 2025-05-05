"""Simple script showing how to apply a force to the robot using the controller."""

# %%
import numpy as np

from crisp_py.robot import Robot

robot = Robot()
robot.wait_until_ready()

# %%
robot.home()
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

# %%
# Be sure that we are working in local coordinates
robot.cartesian_controller_parameters_client.set_parameters([("use_local_jacobian", True)])

# %%
# Push with one newton in the z-direction w.r.t. the end effector
robot.set_target_wrench(force=np.array([0.0, 0.0, 1.0]), torque=np.array([0.0, 0.0, 0.0]))
