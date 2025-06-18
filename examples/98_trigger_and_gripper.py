"""Simple example to control the gripper."""

# %%
from crisp_py.gripper.gripper import Gripper, GripperConfig
from datetime import datetime
from pathlib import Path
import yaml

# %%

gripper_config = GripperConfig(0.0, 1.0, joint_state_topic="trigger_state_broadcaster/joint_states")
gripper = Gripper(gripper_config=gripper_config, namespace="gripper")
gripper.wait_until_ready()

# %%
gripper.disable_torque()

# %%
gripper.value

# %% CALIBRATION STEPS

print("--- Move the gripper to the closed position, then press enter. ---")
input()
min_value = gripper.raw_value

print("--- Now move the gripper to the open position, then press enter. ---")
input()
max_value = gripper.raw_value

# %%
print(f"Measured min_value is {min_value} and max_value is {max_value}")

# %% Saving to a valid config file
this_file_path = Path(__name__).parent.parent
config_file = f"gripper_config_{datetime.now().strftime('%Y-%m-%d-%H-%M')}.yaml"

print(f"Saving config file to: {this_file_path / 'config' / config_file}")

with open(this_file_path / "config" / config_file, "w") as file:
    gripper_config = {"min_value": min_value, "max_value": max_value}
    yaml.dump(gripper_config, file)

# %%
project_root_path = Path("/home/lsy_franka/repos/crisp_py")
gripper_config = None
with open(project_root_path / "config" / "gripper_with_trigger.yaml", "r") as file:
    gripper_config = yaml.safe_load(file)
    gripper_config = GripperConfig(
        min_value=gripper_config.get("min_value"), max_value=gripper_config.get("max_value")
    )

gripper_config.joint_state_topic = "gripper_state_broadcaster/joint_states"

project_root_path = Path("/home/lsy_franka/repos/crisp_py")
trigger_config = None
with open(project_root_path / "config" / "trigger.yaml", "r") as file:
    trigger_config = yaml.safe_load(file)
    trigger_config = GripperConfig(
        min_value=trigger_config.get("min_value"), max_value=trigger_config.get("max_value")
    )

trigger_config.joint_state_topic = "trigger_state_broadcaster/joint_states"
trigger_config.command_topic = "trigger_position_controller/commands"

gripper = Gripper(gripper_config=gripper_config, namespace="gripper")
gripper.wait_until_ready()

trigger = Gripper(node=gripper.node, gripper_config=trigger_config, namespace="gripper", index=1)
trigger.wait_until_ready()

trigger.value

# trigger.enable_torque()
# trigger.reboot()

trigger.set_target(0.0)

rate = gripper.node.create_rate(50)
while True:
    gripper.set_target(max(min(1.0, 1.0 - trigger.value), 0.0))
    rate.sleep()
