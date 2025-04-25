"""Get and set parameters."""

from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters

# %%
from crisp_py.robot import Robot

robot = Robot(namespace="right")
robot.wait_until_ready()

# %%
response = robot.list_params_client.call(ListParameters.Request())
names = response.result.names

# %%
print(names)

# %%
request = GetParameters.Request()
request.names = ["task.k_rot_x"]
response = robot.get_params_client.call(request)

# %%
print(response.values[0].double_value)

# %%
request = SetParameters.Request()
