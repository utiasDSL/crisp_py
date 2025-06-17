"""A client for interacting with ROS2 parameter services.

This module provides a ParametersClient class that simplifies interaction with ROS2 parameter
services. It allows listing, getting, setting, saving, and loading parameters for a specified
ROS2 node.

The client supports operations like:
- Listing available parameters
- Getting parameter values
- Setting parameter values
- Saving parameters to YAML files
- Loading parameters from YAML files

Example:
    ```python
    import rclpy
    from rclpy.node import Node
    from parameters_client import ParametersClient

    node = Node("my_node")
    client = ParametersClient(node, "/target_node")
    client.wait_until_ready()

    # Get all parameters
    params = client.list_parameters()
    values = client.get_parameters(params)

    # Save to file
    client.save_param_config("params.yaml")
    ```
"""

import time
from pathlib import Path
from typing import Any

import yaml
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python


class ParametersClient:
    """This module provides a ParametersClient class that simplifies interaction with ROS2 parameter services. It allows listing, getting, setting, saving, and loading parameters for a specified ROS2 node."""

    def __init__(self, node: Node, target_node: str) -> None:
        """Initialize a ParametersClient to interact with a specific ROS2 node's parameters.

        Args:
            node (Node): The rclpy node that creates and manages this client.
            target_node (str): The fully-qualified name of the target node to communicate with.
        """
        self.node = node
        self.target_node = target_node
        self.list_params_client = self.node.create_client(
            ListParameters,
            f"{target_node}/list_parameters",
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_params_client = self.node.create_client(
            GetParameters, f"{target_node}/get_parameters", callback_group=ReentrantCallbackGroup()
        )

        self.set_parameters_client = self.node.create_client(
            SetParameters, f"{target_node}/set_parameters", callback_group=ReentrantCallbackGroup()
        )

    def wait_until_ready(self, timeout_sec: float = 5.0) -> None:
        """Wait until all parameter services are available or raise TimeoutError if they don't become ready in time.

        Args:
            timeout_sec (float): How many seconds to wait before giving up. Default is 5.0.

        Raises:
            TimeoutError: If the services are not ready within the timeout period.
        """
        t_start = time.time()
        while (
            not self.get_params_client.service_is_ready()
            and not self.list_params_client.service_is_ready()
            and not self.set_parameters_client.service_is_ready()
        ):
            time.sleep(0.01)
            if time.time() - t_start > timeout_sec:
                raise TimeoutError("Waited too long for parameter services to be available.")

    def list_parameters(self) -> list[str]:
        """Retrieve a list of parameter names from the target node.

        Returns:
            list[str]: A list of parameter names available on the target node.

        Raises:
            AssertionError: If the list_parameters service is not ready.
        """
        assert self.list_params_client.service_is_ready(), (
            f"Service for listing params is not ready, have you started the node {self.target_node}?"
        )
        response: ListParameters.Response = self.list_params_client.call(
            request=ListParameters.Request()
        )
        return [str(name) for name in response.result.names]

    def get_parameters(self, param_names: list[str]) -> list[Any]:
        """Get parameter values as Python-native types from the target node.

        Args:
            param_names (list[str]): A list of parameter names to retrieve.

        Returns:
            list[Any]: A list of values converted to Python-native types.
        """
        return [
            parameter_value_to_python(param_value)
            for param_value in self.get_parameter_values(param_names)
        ]

    def get_parameter_values(self, param_names: list[str]) -> list[ParameterValue]:
        """Get raw ParameterValue messages for the specified parameters from the target node.

        Args:
            param_names (list[str]): Names of the parameters to fetch.

        Returns:
            list[ParameterValue]: A list of ParameterValue objects corresponding to the requested parameters.

        Raises:
            AssertionError: If the get_parameters service is not ready.
        """
        assert self.get_params_client.service_is_ready(), (
            f"Service for getting params is not ready, have you started the node {self.target_node}?"
        )
        request = GetParameters.Request()
        request.names = param_names

        response: GetParameters.Response = self.get_params_client.call(request)
        return list(response.values)

    def set_parameters(self, params: list[tuple[str, Any]]) -> None:
        """Set parameters on the target node with new values.

        This function first ensures the parameters exist, then attempts to set them. It raises
        an error if any parameter fails to set.

        Args:
            params: List of tuples of param names to change and the new param values

        Raises:
            AssertionError: If the number of names and values do not match or the service is not ready.
            ValueError: If any of the parameters do not currently exist on the target node.
            RuntimeError: If setting any parameter fails or no results are returned.
        """
        assert self.set_parameters_client.service_is_ready(), (
            f"Service for setting params is not ready, have you started the node {self.target_node}?"
        )
        param_names, new_param_values = zip(*params)
        current_parameters = self.get_parameters(list(param_names))
        if None in current_parameters:
            raise ValueError(
                f"One of the passed elements in the array of params does not exist: {[(name, value) for name, value in zip(param_names, current_parameters)]}"
            )

        updated_params = []
        for param_name, new_param_value in zip(param_names, new_param_values):
            updated_param = Parameter(name=param_name, value=new_param_value).to_parameter_msg()
            updated_params.append(updated_param)

        request = SetParameters.Request()
        request.parameters = updated_params
        response = self.set_parameters_client.call(request=request)

        succesful_results = [param_result.successful for param_result in response.results]
        if False in succesful_results:
            raise RuntimeError(
                f"One or multiple parameters could not be set: {[(name, result.successful, result.reason) for name, result in zip(param_names, response.results)]}"
            )
        if not len(succesful_results):
            raise RuntimeError("No results from setting parameters...")

    def save_param_config(self, file_path: str = "data.yaml") -> None:
        """Save the current parameter values to a YAML file.

        Args:
            file_path (str, optional): Path where to save the YAML file. Defaults to "data.yaml".
        """
        param_names = self.list_parameters()
        param_values = self.get_parameters(param_names)
        params_dict: dict[str, Any] = {
            param_name: param_value for param_name, param_value in zip(param_names, param_values)
        }
        with open(file_path, "w") as outfile:
            yaml.safe_dump(params_dict, outfile)

    def load_param_config(self, file_path: str | Path) -> None:
        """Load parameter values from a YAML file and set them on the target node.

        Args:
            file_path (str | Path): Path to the YAML file containing parameter values.
        """
        params_dict = {}
        with open(file_path, "r") as input_file:
            params_dict: dict = yaml.safe_load(input_file)
        params: list = [(name, value) for name, value in params_dict.items()]
        self.set_parameters(params)
