import matplotlib.pyplot as plt
import numpy as np

from crisp_py.robot import Robot

arm = Robot(namespace="")
arm.wait_until_ready()

# %%
print(arm.end_effector_pose)
print(arm.joint_values)

# %%
print("Going to home position...")
arm.home()
homing_pose = arm.end_effector_pose.copy()

# params = [
#     ("task.k_pos_x", 0.0),
#     ("task.k_pos_y", 0.0),
#     ("task.k_pos_z", 0.0),
#     ("task.k_rot_x", 0.0),
#     ("task.k_rot_y", 0.0),
#     ("task.k_rot_z", 0.0),
#     ("nullspace.stiffness", 5.0),
#     ("nullspace.weights.fr3_joint1.value", 5.0),
#     ("nullspace.weights.fr3_joint2.value", 5.0),
#     ("nullspace.weights.fr3_joint3.value", 5.0),
#     ("nullspace.weights.fr3_joint4.value", 2.0),
#     ("nullspace.weights.fr3_joint5.value", 2.0),
#     ("nullspace.weights.fr3_joint6.value", 1.0),
#     ("nullspace.weights.fr3_joint7.value", 1.0),
#     ("nullspace.projector_type", "none"),  # Do not project joint torques in nullspace, simply let them go through
# ]

# arm.cartesian_controller_parameters_client.set_parameters(params)
# arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")

# set controller type for velocity
arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")
arm.cartesian_controller_parameters_client.load_param_config(
    file_path="config/control/joint_velocity_control.yaml"
    #file_path="config/control/default_operational_space_controller.yaml"
)

ee_poses = []
target_poses = []
ts = []
ctrl_freq = 20
v = 0.1
w = 0.25

print("Starting to move...")
t = 0.0
max_time = 3
#target_pose = left_arm.end_effector_pose.copy()
#rate = arm.node.create_rate(ctrl_freq)

# while t < max_time:
    
#     vel = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

#    # arm.set_target_joint_velocity(vel)

#     #left_arm.set_target(pose=target_pose)

#     rate.sleep()

#     #ee_poses.append(left_arm.end_effector_pose.copy())
#     #target_poses.append(left_arm._target_pose.copy())
#     ts.append(t)

#     t += 1.0 / ctrl_freq

def mv(vel):
    '''Move in the x direction'''
    arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")

    t = 0.0
    max_time = 2
    rate = arm.node.create_rate(ctrl_freq)

    while t < max_time:
        
        velo = np.array(vel)

        arm.get_vel(velo)

        rate.sleep()
        ts.append(t)

        t += 1.0 / ctrl_freq

mv([v,0,0,0,0,0])

print("Going back home.")
arm.home()

print("moving again!")
mv([0,v,0,0,0,0])

print("Going back home.")
arm.home()

print("moving again")
mv([0,0,v,0,0,0])

print("Going back home.")
arm.home()
# %%
arm.shutdown()