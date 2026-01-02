"""A simple example to demonstrate gravity compensation mode."""

# %%
from crisp_py.robot import make_robot

robot = make_robot("fr3")
robot.wait_until_ready()

# %%
robot.cartesian_controller_parameters_client.load_param_config(
    file_path="/home/linusschwarz/repos/crisp_configs/control/gravity_compensation.yaml"
)
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
input()
robot.shutdown()
