"""Get and set parameters."""

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
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
# Here are some desired params to set (change to your willing)
params = [
    ("task.k_pos_x", 1000.0, ParameterType.PARAMETER_DOUBLE),
    ("task.k_pos_y", 1000.0, ParameterType.PARAMETER_DOUBLE),
    ("task.k_pos_z", 1000.0, ParameterType.PARAMETER_DOUBLE),
    ("task.k_rot_x", 60.0, ParameterType.PARAMETER_DOUBLE),
    ("task.k_rot_y", 60.0, ParameterType.PARAMETER_DOUBLE),
    ("task.k_rot_z", 60.0, ParameterType.PARAMETER_DOUBLE),
]

# %%
# Check the current values of the params
request = GetParameters.Request()

request.names = [name for name, *_ in params]
response = robot.get_params_client.call(request)

# %%
print(ParameterType.PARAMETER_DOUBLE == response.values[0].type)
print(response.values[0].double_value)

for i, (name, *_) in enumerate(params):
    print(f"{name} has the value {response.values[i].double_value}")

# %%
request = SetParameters.Request()
request.parameters = []
for param_name, param_value, param_type in params:
    request.parameters += [
        Parameter(
            name=param_name,
            value=ParameterValue(type=param_type, double_value=param_value),
        )
    ]
response = robot.set_parameters_client.call(request)

# %%
robot.controller_switcher_client.switch_controller(
    controller_name="cartesian_impedance_controller"
)
