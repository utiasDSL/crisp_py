"""Example demonstrating runtime variable stiffness for the Cartesian impedance controller.

This script shows how to change the impedance stiffness at runtime using the
variable stiffness topic. The robot maintains its current position while the
stiffness is changed from high to medium to low.

Requirements:
    - The cartesian impedance controller must be active
"""

from crisp_py.robot import make_robot

robot = make_robot("fr3")
robot.wait_until_ready()

# Switch to cartesian impedance controller
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

print("Robot ready. Maintaining current position.")
print()

# High stiffness (current working values)
print("Setting HIGH stiffness: translational=[900, 900, 900], rotational=[45, 45, 45]")
robot.set_stiffness(translational=[900.0, 900.0, 900.0], rotational=[45.0, 45.0, 45.0])
input("Press Enter to switch to MEDIUM stiffness...")

# Medium stiffness
print("Setting MEDIUM stiffness: translational=[300, 300, 300], rotational=[15, 15, 15]")
robot.set_stiffness(translational=[300.0, 300.0, 300.0], rotational=[15.0, 15.0, 15.0])
input("Press Enter to switch to LOW stiffness...")

# Low stiffness
print("Setting LOW stiffness: translational=[50, 50, 50], rotational=[5, 5, 5]")
robot.set_stiffness(translational=[50.0, 50.0, 50.0], rotational=[5.0, 5.0, 5.0])
input("Press Enter to exit...")

robot.shutdown()
