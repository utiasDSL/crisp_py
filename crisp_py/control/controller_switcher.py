"""Script to switch to a different ros2_controller."""

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


class ControllerSwitcherClient:
    """ControllerSwitcher class allows user to communicate with the controller_manager and manage controllers in an easy way."""

    def __init__(
        self,
        node: Node,
    ):
        """Initialize the ControllerSwitcher.

        Args:
            node (Node): Node used for the communication with the controller_manager.
        """
        self.node = node

        self.load_client = node.create_client(
            LoadController,
            "controller_manager/load_controller",
            callback_group=ReentrantCallbackGroup(),
        )
        self.configure_client = node.create_client(
            ConfigureController,
            "controller_manager/configure_controller",
            callback_group=ReentrantCallbackGroup(),
        )
        self.list_client = node.create_client(
            ListControllers,
            "controller_manager/list_controllers",
            callback_group=ReentrantCallbackGroup(),
        )
        self.switch_client = node.create_client(
            SwitchController,
            "controller_manager/switch_controller",
            callback_group=ReentrantCallbackGroup(),
        )

    def is_server_ready(self) -> bool:
        """Return True if all services are ready."""
        return (
            self.load_client.wait_for_service(timeout_sec=5.0)
            and self.configure_client.wait_for_service(timeout_sec=5.0)
            and self.list_client.wait_for_service(timeout_sec=5.0)
            and self.switch_client.wait_for_service(timeout_sec=5.0)
        )

    def get_controller_list(self) -> list:
        """Get a list of all controllers using a service."""
        if not self.list_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Timed out waiting for controller list service.")
        else:
            self.node.get_logger().debug("Got controller list service.")

        future = self.list_client.call_async(ListControllers.Request())

        while not future.done():
            self.node.get_logger().debug(
                "Waiting for controller list...", throttle_duration_sec=1.0
            )

        response = future.result()

        return response.controller

    def load_controller(self, controller_name: str) -> bool:
        """Load a controller using a service."""
        request = LoadController.Request()
        request.name = controller_name
        future = self.load_client.call_async(request)

        while not future.done():
            self.node.get_logger().debug(
                "Waiting for load controller answer...", throttle_duration_sec=1.0
            )
        response = future.result()

        return response.ok

    def configure_controller(self, controller_name: str) -> bool:
        """Configure a controller using a service."""
        request = ConfigureController.Request()
        request.name = controller_name
        future = self.configure_client.call_async(request)

        while not future.done():
            self.node.get_logger().debug(
                "Waiting for configure controller answer...", throttle_duration_sec=1.0
            )
        response = future.result()

        return response.ok

    def _switch_controller(self, to_deactivate: list[str], to_activate: list[str]) -> bool:
        """Switch to a different ros2_controller using a service."""
        request = SwitchController.Request()
        request.deactivate_controllers = to_deactivate
        request.activate_controllers = to_activate
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout = rclpy.duration.Duration(seconds=5).to_msg()

        future = self.switch_client.call_async(request)

        while not future.done():
            self.node.get_logger().debug(
                "Waiting for switch controller answer...", throttle_duration_sec=1.0
            )
        response = future.result()

        return response.ok

    def switch_controller(self, controller_name: str) -> bool | None:
        """Switch to a different ros2_controller that is already loaded using a service.

        First we request a list of current controllers.
        If the desired controller is not loaded, then we request to load it and configure it.
        Finally, we request to switch to the desired controller.

        Args:
            controller_name (str): Name of the controller to switch to.
        """
        controllers = self.get_controller_list()

        active_controllers = [
            controller.name for controller in controllers if controller.state == "active"
        ]
        inactive_controllers = [
            controller.name for controller in controllers if controller.state == "inactive"
        ]

        if controller_name in active_controllers:
            self.node.get_logger().debug(f"Controller {controller_name} is already active.")
            return True

        if controller_name not in inactive_controllers:
            ok = self.load_controller(controller_name)
            if not ok:
                self.node.get_logger().error(
                    f"Failed to load controller {controller_name}. Are you sure the controller exists?"
                )
                raise RuntimeError(f"Failed to load controller {controller_name}.")

            ok = self.configure_controller(controller_name)
            if not ok:
                self.node.get_logger().error(f"Failed to configure controller {controller_name}.")
                raise RuntimeError(f"Failed to configure controller {controller_name}.")

        to_deactivate = []
        for active_controller in active_controllers:
            if not active_controller.endswith("broadcaster"):  # Do not deactivate broadcasters
                to_deactivate.append(active_controller)

        to_activate = [controller_name]

        ok = self._switch_controller(to_deactivate, to_activate)

        if not ok:
            self.node.get_logger().error(f"Failed to switch to controller {controller_name}.")
            raise RuntimeError(f"Failed to switch to controller {controller_name}.")

        return True
