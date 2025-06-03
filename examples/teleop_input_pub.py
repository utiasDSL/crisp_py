#!/usr/bin/env python3

"""
To run this script, see README.md. 
"""

import os
import rclpy
from rclpy.node import Node
import numpy as np

'''from franka_gripper.msg import GraspActionGoal, MoveActionGoal
#replace: ???
from franka_msgs.action import Grasp'''
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray, Float64
from crisp_py.robot import Robot

# from ruamel.yaml import YAML


class TeleopController(Node):
    """Controller class for PS4 teleoperation of Franka robot"""
    
    def __init__(self, command_topic='command', arm=None):
        self.vel_scale = 0.125
        self.yaw_scale = 5
        self.vel_vec = np.zeros(6)
        hz = 100 # init rate frequency
        self.arm = arm
        arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")
        arm.cartesian_controller_parameters_client.load_param_config(
            file_path="config/control/joint_velocity_control.yaml"
            #file_path="config/control/default_operational_space_controller.yaml"
        )
        
        # Initialize ROS node and load config
        #self.node = rclpy.create_node('teleop_controller')
        super().__init__('teleop_controller')
        
        #rclpy.init_node('teleop_pub', anonymous=True)
        #self.config = self.load_config()
        self.hz = self.declare_parameter('/teleop/hz', hz)

        self.command_topic = command_topic
        
        # Initialize publishers
        self.setup_publishers()
        '''self.setup_gripper()'''
        self.rate = arm.node.create_rate(hz) #prev self.hz parameter
        #self.timer_ = node.create_timer(self.hz, self.timer_callback)

        self.create_timer(0.01, self.publish_velocity)
        self.create_timer(0.01, self.record_keypress)

        # Initialize data recording
        self.time_start = self.get_clock().now().nanoseconds * 1e-9
        self.key_press_list = []
        self.get_logger().info("Teleoperation started: 3D position control via PS4 controller")

        # Button state tracking to avoid repeated commands
        self.prev_grasp_button = 0
        self.prev_release_button = 0

    """Don't need this hopefully"""

    # def load_config(self):
    #     """Load configuration from YAML file"""
    #     file_path = os.path.dirname(os.path.abspath(__file__))
    #     config_path = os.path.join(file_path, '../../../config/config.yml')
        
    #     yaml = YAML()
    #     yaml.preserve_quotes = True
        
    #     with open(config_path, 'r') as file:
    #         return yaml.load(file)

    def setup_publishers(self):
        """Initialize ROS publishers"""
        #self.vel_publisher = rclpy.Publisher('/teleop_custom', Float64MultiArray, queue_size=1)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/cartesian_velocity_controller/commands', 10)
        # Use queue_size=1 for gripper to avoid buffering commands
        '''self.grasp_publisher = self.create_publisher(GraspActionGoal, '/franka_gripper/grasp/goal', queue_size=1)'''
        
        # Wait for velocity controller to be ready
        wait_topic = f"/custom_joint1_velocity_controller/{self.command_topic}"
        self.arm.wait_until_ready()
        
        # Setup joystick subscriber
        #rclpy.Subscriber('/joy', Joy, self.joy_callback)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def setup_gripper(self):
        """Initialize gripper command messages"""
        self.grasp_msg = GraspActionGoal()
        self.grasp_msg.goal.epsilon.inner = 0.01
        self.grasp_msg.goal.epsilon.outer = 0.01
        self.grasp_msg.goal.force = 0.001
        self.grasp_msg.goal.speed = 0.05
        
        # Gripper widths
        self.grasp_width = 0.012  # container
        self.release_width = 0.08
        
        # Try to get actual gripper state if possible (implementation depends on your setup)
        # For now, we'll use a flag and assume the gripper starts open
        self.grasp_state = False
        
        self.get_logger().info("Gripper initialized. Assuming starting position is OPEN.")
        self.get_logger().info("Press L2 to close gripper, R2 to open gripper.")

    def joy_callback(self, data: Joy):
        """Process joystick input"""
        # Linear velocities
        left_stick_x = data.axes[1]  # Left stick vertical, x direction
        left_stick_y = data.axes[0]  # Left stick horizontal, y direction
        arrow_up_down = data.axes[-1]  # up and down buttons, z direction 
        
        # Angular velocities
        right_stick_x = -data.axes[2]  # Right stick horizontal, roll
        right_stick_y = data.axes[3]  # Right stick vertical, pitch
        
        # Yaw control
        triangle = data.buttons[3]  # triangle button, positive yaw
        cross = data.buttons[0]     # cross button, negative yaw
        yaw = (triangle - cross) * self.yaw_scale
        
        # Gripper control - detect button state changes
        grasp_button = data.axes[4]  # L2 button
        release_button = data.axes[5]  # R2 button
        
        # Emergency stop
        stop_button = float(data.buttons[2]) # Square button
        
        # Update velocity command
        self.vel_vec = (1.0 - stop_button) * np.array([
            left_stick_x, left_stick_y, arrow_up_down,
            right_stick_x, right_stick_y, yaw
        ]) * self.vel_scale
        
        # Handle gripper commands with improved logic
        '''self.handle_gripper(grasp_button, release_button)'''
        
        # Update previous button states
        self.prev_grasp_button = grasp_button
        self.prev_release_button = release_button

    def handle_gripper(self, grasp_button, release_button):
        """Handle gripper open/close commands with improved logic"""
        # Check for button press changes to avoid repeated commands
        grasp_pressed = grasp_button < -0.9 and self.prev_grasp_button >= -0.9
        release_pressed = release_button < -0.9 and self.prev_release_button >= -0.9
        
        if grasp_pressed:
            # Button to close the gripper is pressed
            self.get_logger().info("Closing gripper to width %.3f", self.grasp_width)
            self.grasp_msg.goal.width = self.grasp_width
            self.grasp_publisher.publish(self.grasp_msg)
            self.grasp_state = True
        elif release_pressed:
            # Button to open the gripper is pressed
            rclpy.loginfo("Opening gripper to width %.3f", self.release_width)
            self.grasp_msg.goal.width = self.release_width
            self.grasp_publisher.publish(self.grasp_msg)
            self.grasp_state = False

    def publish_velocity(self):
        """Publish velocity command"""
        #msg = Float64MultiArray()
        #print("pubbing vel")
        #msg = TwistStamped()
        #msg.header.stamp = self.get_clock().now().to_msg()
        # msg.twist.linear.x = float(self.vel_vec[0])
        # msg.twist.linear.y = float(self.vel_vec[1])
        # msg.twist.linear.z = float(self.vel_vec[2])
        # msg.twist.angular.x = float(self.vel_vec[3])
        # msg.twist.angular.y = float(self.vel_vec[4])
        # msg.twist.angular.z = float(self.vel_vec[5])
        msg = Float64MultiArray()
        msg.data = self.vel_vec.tolist()
        self.cmd_pub.publish(msg)

    def record_keypress(self):
        """Record keypress data"""
        time_now = self.get_clock().now().nanoseconds * 1e-9
        self.key_press_list.append(
            f'{[self.vel_vec.tolist()], [time_now - self.time_start], [self.time_start]}'
        )

    def shutdown(self):
        """Clean shutdown procedure"""
        # Stop the robot
        self.get_logger().info("Shutting down robot")
        # msg = TwistStamped()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # # Could put this in a stop_msg variable
        # msg.twist.linear.x = 0.0
        # msg.twist.linear.y = 0.0
        # msg.twist.linear.z = 0.0
        # msg.twist.angular.x = 0.0
        # msg.twist.angular.y = 0.0
        # msg.twist.angular.z = 0.0

        msg = Float64MultiArray()
        msg.data = [0.0] * 6
        self.cmd_pub.publish(msg)
        
        # Save recorded data if needed
        # if hasattr(self, 'config') and self.config['CBF']['Saving_data']['is_save_data']:
        #     save_path = self.config['CBF']['save_path']
        #     with open(save_path, 'a') as f:
        #         for keypress in self.key_press_list:
        #             f.write(f'{keypress}\n')

def main(args=None): #previously run(self)
    """Main control loop"""
    #self.get_logger().info("Teleoperation: 3D position end-effector\nps4 CONTROLLER")
    
    rclpy.init(args=args)
    arm = Robot(namespace="") # is this the right place for this
    controller = TeleopController(command_topic='command',arm=arm)
    print("hello")
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


    # try:
    #     while not rclpy.is_shutdown():
    #         self.publish_velocity()
    #         self.record_keypress()
    #         self.rate.sleep()
            
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     self.shutdown()


if __name__ == '__main__':
    main()
    
    # try:
    #     # command_topic = 'command'
    #     command_topic = 'uncertified_command'
    #     controller = TeleopController(command_topic=command_topic)
    #     controller.run()
    # except KeyboardInterrupt:
    #     pass
