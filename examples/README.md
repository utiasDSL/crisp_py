# crisp_py Examples

This folder contains example scripts demonstrating various features of crisp_py.

## Basic Robot Control

| Example | Description |
|---------|-------------|
| `00_gravity_compensation.py` | Demonstrates gravity compensation mode, allowing the robot to be moved freely by hand |
| ⭐ `01_figure_eight.py` | Makes the robot follow a figure-eight trajectory on the yz plane using cartesian impedance control |
| ⭐ `02_joint_control.py` | Shows how to control robot joints directly using joint impedance control |
| `03_getting_and_setting_parameters.py` | Demonstrates using `ParametersClient` to dynamically get, set, and list controller parameters |
| `04_dual_arm.py` | Sets up a dual-arm teleoperation system where the right arm mirrors the left arm's movements |
| `05_apply_force.py` | Shows how to apply a force/wrench to the robot using the cartesian impedance controller |

## Grippers

| Example | Description |
|---------|-------------|
| `06_franka_gripper.py` | Basic gripper control - opening and closing a Franka gripper |
| `18_dynamixel_gripper.py` | Controls an dynamixel gripper with open, close, and set target position |

## Cameras and Sensors

| Example | Description |
|---------|-------------|
| `07_camera_example_single_frame.py` | Captures a single frame from a camera and displays it using matplotlib |
| `08_sensor_example.py` | Reads data from a force/torque sensor and plots it over time |
| ⭐ `13_camera_example_stream.py` | Continuously streams camera images with live updates |

## Advanced Topics

| Example | Description |
|---------|-------------|
| `09_get_twist.py` | Shows how to get the end-effector twist (linear and angular velocity) while following a trajectory |
| `10_tf_pose.py` | Demonstrates using TF (transform frames) to get the robot's end-effector pose |
| `11_control_tests.py` | Compares different control modes (Operational Space Control vs Cartesian Impedance) on a figure-eight trajectory |
| ⭐ `14_viser_example.py` | Interactive 3D visualization using viser with transform controls for commanding the robot |
| `20_infinite_figure.py` | Runs a continuous figure-eight trajectory until interrupted with Ctrl+C |

## Robot-Specific Examples

| Example | Description |
|---------|-------------|
| `example_with_kinova.py` | Demonstrates circle trajectory control with a Kinova robot |
| `example_with_iiwa.py` | Demonstrates circle trajectory control with a KUKA iiwa robot |


## Getting Started

Most examples follow this general pattern:

```python
from crisp_py.robot import make_robot

# Create and initialize the robot
robot = make_robot("fr3")
robot.wait_until_ready()

# Switch to the desired controller
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

# Perform operations...

# Clean up
robot.shutdown()
```

For gripper examples:

```python
from crisp_py.gripper.gripper import make_gripper

gripper = make_gripper("gripper_franka")
gripper.wait_until_ready()

gripper.open()
gripper.close()

gripper.shutdown()
```
