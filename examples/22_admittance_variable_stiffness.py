"""Example demonstrating variable admittance stiffness for the admittance controller.

This script shows how to change the admittance stiffness at runtime. The robot
maintains its position while the admittance compliance is changed from stiff
(resists external forces) to compliant (yields to external forces).

Requirements:
    - cartesian_admittance_controller must be loaded and available
    - variable_admittance_stiffness.enabled must be set to true
    - An F/T sensor must be publishing on ft_sensor_wrench topic
"""

from crisp_py.robot import make_robot

robot = make_robot("fr3")
robot.wait_until_ready()

# Switch to admittance controller
robot.controller_switcher_client.switch_controller("cartesian_admittance_controller")

# Load default admittance parameters
robot.admittance_controller_parameters_client.load_param_config(
    "config/control/default_admittance.yaml"
)

print("Admittance controller active. An F/T sensor is required.")
print("Push the robot gently to feel the compliance change.")
print()

# High admittance stiffness (stiff - resists external forces)
print("Setting HIGH admittance stiffness: translational=[500, 500, 500], rotational=[30, 30, 30]")
print("The robot should feel stiff and resist external forces.")
robot.set_admittance_stiffness(translational=[500.0, 500.0, 500.0], rotational=[30.0, 30.0, 30.0])
input("Push the robot. Press Enter for MEDIUM stiffness...")

# Medium admittance stiffness
print("Setting MEDIUM admittance stiffness: translational=[100, 100, 100], rotational=[10, 10, 10]")
print("The robot should be moderately compliant.")
robot.set_admittance_stiffness(translational=[100.0, 100.0, 100.0], rotational=[10.0, 10.0, 10.0])
input("Push the robot. Press Enter for LOW stiffness...")

# Low admittance stiffness (compliant - yields to external forces)
print("Setting LOW admittance stiffness: translational=[20, 20, 20], rotational=[2, 2, 2]")
print("The robot should feel very compliant and yield to pushes.")
robot.set_admittance_stiffness(translational=[20.0, 20.0, 20.0], rotational=[2.0, 2.0, 2.0])
input("Push the robot. Press Enter to exit...")

robot.shutdown()
