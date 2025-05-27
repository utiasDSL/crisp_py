#!/usr/bin/env python3

"""
To run this script, see README.md. 
"""

import os
import rclpy
from rclpy.node import Node
import numpy as np

from franka_gripper.msg import GraspActionGoal, MoveActionGoal
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray, Float64
# from ruamel.yaml import YAML


class TeleopController(Node):
    """Controller class for PS4 teleoperation of Franka robot"""
    
    def __init__(self, command_topic='command'):
        self.vel_scale = 0.1
        self.yaw_scale = 2.0
        self.vel_vec = np.zeros(6)
        
        # Initialize ROS node and load config
        rclpy.init()
        node = TeleopController()
        #rclpy.init_node('teleop_pub', anonymous=True)
        #self.config = self.load_config()
        self.hz = self.declare_parameter('/teleop/hz', default=100)

        self.command_topic = command_topic
        
        # Initialize publishers
        self.setup_publishers()
        self.setup_gripper()
        #self.rate = rclpy.Rate(self.hz)
        self.timer_ = node.create_timer(self.hz, self.timer_callback)

        # Initialize data recording
        self.time_start = rclpy.get_time()
        self.key_press_list = []
        
        # Button state tracking to avoid repeated commands
        self.prev_grasp_button = 0
        self.prev_release_button = 0

    def load_config(self):
        """Load configuration from YAML file"""
        file_path = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(file_path, '../../../config/config.yml')
        
        yaml = YAML()
        yaml.preserve_quotes = True
        
        with open(config_path, 'r') as file:
            return yaml.load(file)

    def setup_publishers(self):
        """Initialize ROS publishers"""
        #self.vel_publisher = rclpy.Publisher('/teleop_custom', Float64MultiArray, queue_size=1)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cartesian_velocity_controller/commands', 10)
        # Use queue_size=1 for gripper to avoid buffering commands
        self.grasp_publisher = rclpy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1)
        
        # Wait for velocity controller to be ready
        wait_topic = f"/custom_joint1_velocity_controller/{self.command_topic}"
        rclpy.wait_for_message(wait_topic, Float64)
        
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
        
        rclpy.loginfo("Gripper initialized. Assuming starting position is OPEN.")
        rclpy.loginfo("Press L2 to close gripper, R2 to open gripper.")

    def joy_callback(self, data: Joy):
        """Process joystick input"""
        # Linear velocities
        left_stick_x = data.axes[1]  # Left stick vertical, x direction
        left_stick_y = data.axes[0]  # Left stick horizontal, y direction
        arrow_up_down = data.axes[-1]  # up and down buttons, z direction 
        
        # Angular velocities
        right_stick_x = -data.axes[3]  # Right stick horizontal, roll
        right_stick_y = data.axes[4]  # Right stick vertical, pitch
        
        # Yaw control
        triangle = data.buttons[2]  # triangle button, positive yaw
        cross = data.buttons[0]     # cross button, negative yaw
        yaw = (triangle - cross) * self.yaw_scale
        
        # Gripper control - detect button state changes
        grasp_button = data.axes[-3]  # L2 button
        release_button = data.axes[2]  # R2 button
        
        # Emergency stop
        stop_button = float(data.buttons[3])
        
        # Update velocity command
        self.vel_vec = (1.0 - stop_button) * np.array([
            left_stick_x, left_stick_y, arrow_up_down,
            right_stick_x, right_stick_y, yaw
        ]) * self.vel_scale
        
        # Handle gripper commands with improved logic
        self.handle_gripper(grasp_button, release_button)
        
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
            rclpy.loginfo("Closing gripper to width %.3f", self.grasp_width)
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
        msg  TwistedStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = self.vel_vec.tolist()
        self.vel_publisher.publish(msg)

    def record_keypress(self):
        """Record keypress data"""
        time_now = rclpy.get_time()
        self.key_press_list.append(
            f'{[self.vel_vec.tolist()], [time_now - self.time_start], [self.time_start]}'
        )

    def shutdown(self):
        """Clean shutdown procedure"""
        # Stop the robot
        msg = Float64MultiArray()
        msg.data = [0.0] * 6
        self.vel_publisher.publish(msg)
        
        # Save recorded data if needed
        # if hasattr(self, 'config') and self.config['CBF']['Saving_data']['is_save_data']:
        #     save_path = self.config['CBF']['save_path']
        #     with open(save_path, 'a') as f:
        #         for keypress in self.key_press_list:
        #             f.write(f'{keypress}\n')

    def run(self):
        """Main control loop"""
        rclpy.loginfo("Teleoperation: 3D position end-effector\nps4 CONTROLLER")
        
        try:
            while not rclpy.is_shutdown():
                self.publish_velocity()
                self.record_keypress()
                self.rate.sleep()
                
        except rclpy.ROSInterruptException:
            pass
        finally:
            self.shutdown()


if __name__ == '__main__':
    try:
        # command_topic = 'command'
        command_topic = 'uncertified_command'
        controller = TeleopController(command_topic=command_topic)
        controller.run()
    except rclpy.ROSInterruptException:
        pass
