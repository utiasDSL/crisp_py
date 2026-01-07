from __future__ import annotations

from pathlib import Path
import time
from typing import Literal

import numpy as np
import viser
from viser.extras import ViserUrdf

from crisp_py.robot import make_robot
from crisp_py.utils.geometry import Pose

robot = make_robot("fr3")
robot.wait_until_ready()

robot.home()
pose_home = robot.end_effector_pose

robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
robot.cartesian_controller_parameters_client.load_param_config(
    file_path="config/control/default_cartesian_impedance.yaml"
)
current_position = robot.end_effector_pose.position.copy()
current_position[1] += 0.1
robot.set_target(position=current_position)


def main(
    robot_type: Literal["panda"] = "panda",
    load_meshes: bool = True,
    load_collision_meshes: bool = False,
) -> None:
    # Start viser server.
    server = viser.ViserServer()

    # Load URDF.
    #
    # This takes either a yourdfpy.URDF object or a path to a .urdf file.
    viser_urdf = ViserUrdf(
        server,
        urdf_or_path=Path("assets/fr3_franka_hand.urdf"),
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
    viser_urdf.update_cfg(np.array(robot.joint_values))

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
        position=pose_home.position,
        scale=0.3,
        line_width=3.0,
    )

    # Add callback for when the transform handle is moved.
    @transform_handle.on_update
    def update_robot_target(handle: viser.TransformControlsEvent) -> None:
        # Get target pose from the handle.
        pose = Pose(position=handle.target.position, orientation=pose_home.orientation)
        robot.set_target(pose=pose)


    # Sleep forever.
    while True:
        time.sleep(2.0)


if __name__ == "__main__":
    main()
