"""Example using the ParametersClient to get parameters from a node."""

# %%
import threading
import rclpy
from crisp_py.control.parameters_client import ParametersClient
from rclpy.executors import MultiThreadedExecutor

# %%
rclpy.init()

# %%
node = rclpy.create_node(node_name="test_node", namespace="right")

# %%


def spin_node(node):
    if not rclpy.ok():
        rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)


threading.Thread(target=spin_node, args=(node,), daemon=True).start()

# %%
client = ParametersClient(node=node, target_node="cartesian_impedance_controller")

# %%
try:
    client.wait_until_ready()
    print("You are ready to change params.")
except TimeoutError:
    print("Not able to wait until ready. Have you spawned the controllers? Have you started the robot?")

# %%

print(client.list_parameters())

# %%
parameter_names = ["task.k_pos_x"]
parameter_values = [300.0]

# %%
print(client.get_parameters(parameter_names))

# %%
client.set_parameters(parameter_names, parameter_values)

# %%
print(client.get_parameters(parameter_names))
