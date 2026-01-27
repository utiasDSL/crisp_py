from __future__ import annotations

import time
from typing import Literal

import numpy as np
import viser
from viser.extras import ViserUrdf
from scipy.spatial.transform import Rotation
from robot_descriptions.loaders.yourdfpy import load_robot_description

from crisp_py.robot import make_robot
from crisp_py.utils.geometry import Pose

# TODO: Set robot type here.
ROBOT_TYPE: Literal["fr3", "panda", "iiwa14"] = "fr3"

# ===

def get_description_name(robot_type: str) -> str:
    if robot_type in ["fr3", "panda"]:
        return "panda_description"
    elif robot_type == "iiwa":
        return "iiwa14_description"
    else:
        return f"{robot_type}_description"

def should_add_gripper_to_config(robot_type: str) -> bool:
    if robot_type in ["fr3", "panda"]:
        return True
    return False

robot = make_robot(ROBOT_TYPE)
robot.wait_until_ready()

robot.config.time_to_home = 2.0
robot.home()
start_pose = robot.end_effector_pose

robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
robot.cartesian_controller_parameters_client.load_param_config(
    file_path="config/control/default_cartesian_impedance.yaml"
)

load_meshes: bool = True
load_collision_meshes: bool = False

# Start viser server.
server = viser.ViserServer()

# Load URDF.
#
# This takes either a yourdfpy.URDF object or a path to a .urdf file.

urdf = load_robot_description(get_description_name(ROBOT_TYPE))
viser_urdf = ViserUrdf(
    server,
    urdf_or_path=urdf,
    load_meshes=load_meshes,
    load_collision_meshes=load_collision_meshes,
    collision_mesh_color_override=(1.0, 0.0, 0.0, 0.5),
)

# Add visibility checkboxes.
with server.gui.add_folder("Visibility"):
    show_meshes_cb = server.gui.add_checkbox(
        "Show meshes",
        viser_urdf.show_visual,
    )
    show_collision_meshes_cb = server.gui.add_checkbox(
        "Show collision meshes", viser_urdf.show_collision
    )

@show_meshes_cb.on_update
def _(_):
    viser_urdf.show_visual = show_meshes_cb.value

@show_collision_meshes_cb.on_update
def _(_):
    viser_urdf.show_collision = show_collision_meshes_cb.value

# Hide checkboxes if meshes are not loaded.
show_meshes_cb.visible = load_meshes
show_collision_meshes_cb.visible = load_collision_meshes

# Set initial robot configuration.
actuation = np.array([*robot.joint_values, 0.0]) if should_add_gripper_to_config(ROBOT_TYPE) else np.array(robot.joint_values)
viser_urdf.update_cfg(actuation)

# Create grid.
trimesh_scene = viser_urdf._urdf.scene or viser_urdf._urdf.collision_scene
server.scene.add_grid(
    "/grid",
    width=2,
    height=2,
    position=(
        0.0,
        0.0,
        # Get the minimum z value of the trimesh scene.
        trimesh_scene.bounds[0, 2] if trimesh_scene is not None else 0.0,
    ),
)
# Add interactive transform controls for the end effector.
transform_handle = server.scene.add_transform_controls(
    "/end_effector_target",
    position=start_pose.position,
    wxyz=start_pose.orientation.as_quat(scalar_first=True),
    scale=0.3,
    line_width=3.0,
)

# Add callback for when the transform handle is moved.
@transform_handle.on_update
def update_robot_target(handle: viser.TransformControlsEvent) -> None:
    rot = Rotation.from_quat(handle.target.wxyz, scalar_first=True)
    pose = Pose(position=handle.target.position, orientation=rot)
    robot.set_target(pose=pose)


# Sleep forever.
while True:
    actuation = np.array([*robot.joint_values, 0.0]) if should_add_gripper_to_config(ROBOT_TYPE) else np.array(robot.joint_values)
    viser_urdf.update_cfg(actuation)
    time.sleep(0.01)
