## How to grasp

To grasp, make sure that you set the proper `min_value` and `max_value` in the config for the range in which the gripper joint moves.
All joint values are normalized between 0 (closed) and 1 (open).
To calibrate a gripper, use the script:
```bash
python scripts/gripper_manual_calibration.py --help
```
The script will guide you in calibration procedure.
[../media/gripper_calibration_output.png](../media/gripper_calibration_output.png)

After this, control a gripper as follows:
```python
from crisp_py.gripper import Gripper, GripperConfig

gripper_config = GripperConfig.from_yaml("path/to/yaml")
# or gripper_config = GripperConfig(min_value=x.x, max_value=x.x)

gripper = Gripper(config=gripper_config)
gripper.open()  # equivalent to gripper.set_target(1.0)
gripper.close()  # equivalent to gripper.set_target(0.0)
gripper.set_target(0.5)
gripper.shutdown()

```
