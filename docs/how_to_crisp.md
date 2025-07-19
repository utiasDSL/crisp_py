## How to crisp

First import some necessary packages that we are going to use
```python
from crisp_py.robot import Robot
from crisp_py.robot_config import FrankaConfig, KinovaConfig, IiwaConfig
```

Start the ROS2 robot stack in a separate terminal. For example, launch one of the robots provided in the [crisp_controllers_demos](https://github.com/utiasDSL/crisp_controllers_demos).

Now you can start the robot and wait until it is ready (to be sure that we received the current state of the robot). There is a timeout, and it will fail if the robot is not available.
```python
robot = Robot(robot_config=FrankaConfig())  # Default is Franka
robot.wait_until_ready()
```

Now you can query the pose and joint values that have been received. We always save the latest.

```python
print(robot.end_effector_pose)
print(robot.joint_values)
```

Bring the robot to the homing position using the `joint_trajectory_controller` (the home config can be overwritten)
```python
robot.home()
```


To explicitly toggle a controller you can use the `controller_switcher_client` which is a client wrapper for the `ros2_control ControllerManager`.
```python
controllers = robot.controller_switcher_client.get_controller_list()
for controller in controllers:
    print(f"{controller.name}: {controller.state}")
```

This will display a list of available controllers and their state:
```python
cartesian_impedance_controller: inactive
gravity_compensation: inactive
joint_state_broadcaster: active
pose_broadcaster: active
joint_trajectory_controller: active
```

Finally you can switch controller. This call will make sure that all controllers that use commands interfaces are deactivated and your controller is ready to use:

```python
robot.reset_targets()
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
# robot.controller_switcher_client.switch_controller("gravity_compensation")
```

If you list the available controllers again you will see:
```python
cartesian_impedance_controller: active
gravity_compensation: inactive
joint_state_broadcaster: active
pose_broadcaster: active
joint_trajectory_controller: inactive
```

You can modify the parameters of the controller on the fly either on rqt or here. List the available parameters:
```python
robot.cartesian_controller_parameters_client.list_parameters()
```
Then modify them:
```python
params = [
    ("task.k_pos_x", 400.0),
    ("task.k_pos_y", 400.0),
    ("task.k_pos_z", 400.0),
    ("task.k_rot_x", 20.0),
    ("task.k_rot_y", 20.0),
    ("task.k_rot_z", 20.0),
    ("nullspace.stiffness", 5.0),
]

robot.cartesian_controller_parameters_client.set_parameters(params)
```
You can also load parameters from a yaml file with `robot.cartesian_controller_parameters_client.load_params(path)`
Check if the changes have been applied:

```python
param_names, _= zip(*params)
robot.cartesian_controller_parameters_client.get_parameters(param_names)
```

The `move_to` function will publish a pose to `/target_pose` while interpolating linearly to it. It is blocking.
```python
x, y, z = robot.end_effector_pose.translation
robot.move_to(position=[x-0.1, y, z], speed=0.15)
```

The `set_target` function will directly set the pose as a target and it will get publish (check the robot config to see the publish frequency)
```python
x, y, z = robot.end_effector_pose.translation
robot.set_target(position=[x+0.02, y, z])
```

The `set_target_joint` function is similar to the previous one but for the joint space. It will use the target joint to normalize the joints in the nullspace.
```python
q = robot.joint_values.copy()
q[0] += 0.5  # [rad]
robot.set_target_joint(q)
```
If you activate a "joint-only" controller that accepts a `target_joint` topic you can use `set_target_joint(q)` to control the robot.

Once you are done, you can shutdown the robot (or simply kill the terminal/process where you are working):
```python
robot.shutdown()
```
