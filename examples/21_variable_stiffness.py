"""Example demonstrating runtime variable stiffness for the Cartesian impedance controller.

This script shows how to change the impedance stiffness at runtime using the
variable stiffness topic. The robot maintains its current position while the
stiffness is changed from high to medium to low.

Requirements:
    - variable_stiffness.enabled must be set to true in the controller parameters
    - The cartesian impedance controller must be active
"""

from crisp_py.robot import make_robot

robot = make_robot("fr3")
robot.wait_until_ready()

# Switch to cartesian impedance controller
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

# Enable variable stiffness on the controller
robot.cartesian_controller_parameters_client.set_parameters(
    [("variable_stiffness.enabled", True)]
)

print("Robot ready. Maintaining current position.")
print()

# High stiffness (default-like values)
print("Setting HIGH stiffness: translational=[600, 600, 600], rotational=[30, 30, 30]")
robot.set_stiffness(translational=[600.0, 600.0, 600.0], rotational=[30.0, 30.0, 30.0])
input("Press Enter to switch to MEDIUM stiffness...")

# Medium stiffness
print("Setting MEDIUM stiffness: translational=[200, 200, 200], rotational=[15, 15, 15]")
robot.set_stiffness(translational=[200.0, 200.0, 200.0], rotational=[15.0, 15.0, 15.0])
input("Press Enter to switch to LOW stiffness...")

# Low stiffness
print("Setting LOW stiffness: translational=[50, 50, 50], rotational=[5, 5, 5]")
robot.set_stiffness(translational=[50.0, 50.0, 50.0], rotational=[5.0, 5.0, 5.0])
input("Press Enter to exit...")

robot.shutdown()
