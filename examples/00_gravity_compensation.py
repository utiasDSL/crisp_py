"""Try to follow a "figure eight" target on the yz plane."""

# %%
from crisp_py.robot import Robot

robot = Robot()
robot.wait_until_ready()

# %%
robot.cartesian_controller_parameters_client.load_param_config(
    file_path="config/control/gravity_compensation.yaml"
)
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
robot.shutdown()

