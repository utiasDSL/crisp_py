"""Example using the ParametersClient to get parameters from a node."""

# %%
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from crisp_py.control.parameters_client import ParametersClient

# %%
rclpy.init()

# %%
node = rclpy.create_node(node_name="test_node", namespace="right", parameter_overrides=[])

# %%


def spin_node(node: rclpy.Node):
    """Simple function to spin a node."""
    if not rclpy.ok():
        rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)


threading.Thread(target=spin_node, args=(node,), daemon=True).start()

# %%
# Create a client to change parameters. Note that this client can be found in the Robot() class as well.
# But you can create clients for other target_nodes that allow dynamic reconfiguration (like the crisp_controllers)
# The ParametersClient is a wrapper containing multiple service clients to get, set and list the parameters.
client = ParametersClient(node=node, target_node="cartesian_impedance_controller")

# %%
# Check if the client is ready to receive requests.
try:
    client.wait_until_ready()
    print("You are ready to change params.")
except TimeoutError:
    print(
        "Not able to wait until ready. Have you spawned the controllers? Have you started the robot?"
    )

# %%
# Check parameters that we could change dynamically.
print(client.list_parameters())

# %%
# Create a list of parameters to modify.
params = [("task.k_pos_x", 300.0)]

# %%
# Set the parameters. If the parameter names do not exist, this will raise an error
client.set_parameters(params)
