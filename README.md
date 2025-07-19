
<img src="media/crisp_py_logo.png" alt="CRISP PY Logo" />

[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)
![MIT Badge](https://img.shields.io/badge/MIT-License-blue?style=flat)
<a href="https://github.com/utiasDSL/crisp_py/actions/workflows/ruff_ci.yml"><img src="https://github.com/utiasDSL/crisp_py/actions/workflows/ruff_ci.yml/badge.svg"/></a>
<a href="https://github.com/utiasDSL/crisp_py/actions/workflows/pixi_ci.yml"><img src="https://github.com/utiasDSL/crisp_py/actions/workflows/pixi_ci.yml/badge.svg"/></a>

*crisp_py /krɪspi/*, a python package to interface with robots using [crisp_controllers](https://github.com/utiasDSL/crisp_controllers) or any ROS2 manipulator with a similar interface!
Set target poses and joints, reconfigure stiffness and other controller parameters dynamically, deactivate and activate `ros2_controllers` and more!

![crisp_py](https://github.com/user-attachments/assets/e4cbf5fd-6ba7-4d7c-917a-bbb78d79ab10)

## Getting started

### Intall from PyPi
> [!WARNING]
> 👷‍♂️ This is still work in progress...

Simply add it to your project with
```bash
pip install crisp-py
```
be sure that ROS2 is installed in your system. We recommend to use [robostack](https://robostack.github.io/) in combination with [pixi](https://pixi.sh/latest/).
You can check [crisp_gym](https://github.com/utiasDSL/crisp_gym) or this repo to see how to set it up.
Try importing `crisp_py` to check if everything is working.
```bash
python
import crisp_py  # If no print, everything is fine!
```

### Git installation with pixi

Clone this repo, install [pixi](https://pixi.sh/latest/) and create an environment to work with the robot:
```bash
pixi install
pixi shell -e humble  # or jazzy
```

> [!NOTE]
> This will activate an environment where `ros2` is sourced, so you are able to use the `roscli`, `rqt`, `rviz` and more!
> The default `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` are set to 100 and 0 respectively. If you want to override them, add a `scripts/personal_ros_env.sh` script
> to the project where you export this environment variables, e.g. `export ROS_DOMAIN_ID=42 && export ROS_LOCALHOST_ONLY=1`. The script will be sourced at activation and is ignored by git.
> More information on this topic can be found [here](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html).

> [!WARNING]
> For **multi-machine setups**, you recommend using a different RMW than FastDDS (for an easier configuration)
> We recommend using CycloneDDS (or [Zenoh RMW](https://github.com/ros2/rmw_zenoh/tree/rolling)). It is easy to configure, in particular
> for multi-machine setups.
> Check [Setup CycloneDDS](docs/setup_cyclonedds.md) to see how to setup a different RMW.

You are good to go!

You can check the examples to get an idea on how to use the code or check some of the guides:
- [How to crisp :robot:](docs/how_to_crisp.md)
- [How to grip :hand:](docs/how_to_grip.md)
