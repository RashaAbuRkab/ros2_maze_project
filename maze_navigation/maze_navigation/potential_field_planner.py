#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# Hint: You may need TwistStamped instead of Twist for ROS 2 Jazzy with Gazebo Sim
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

class PotentialFieldPlanner(Node):
    def __init__(self):
        super().__init__('potential_field_planner')
        
        # Declare parameters
        self.declare_parameter('goal_x', 9.0)
        self.declare_parameter('goal_y', 9.0)
        self.declare_parameter('k_att', 1.0)      # Attractive gain
        self.declare_parameter('k_rep', 150.0)    # Repulsive gain
        self.declare_parameter('d_obs', 1.2)      # Distance of influence (meters)
        self.declare_parameter('max_linear_vel', 0.22)
        self.declare_parameter('max_angular_vel', 1.5)
        
        # Get parameters
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.k_att = self.get_parameter('k_att').value
        self.k_rep = self.get_parameter('k_rep').value
        self.d_obs = self.get_parameter('d_obs').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.scan_data = None
        self.scan_angles = None
        
        # TODO: Initialize Publishers and Subscribers
        # self.cmd_vel_pub = ...
        # self.odom_sub = ...
        # self.scan_sub = ...
        
        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"Planner initialized. Navigating to Goal: ({self.goal_x}, {self.goal_y})")

    def euler_from_quaternion(self, q):
        """Convert quaternion to euler yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

    def scan_callback(self, msg):
        # Convert tuple to numpy array for easier math
        self.scan_data = np.array(msg.ranges)
        
        # TODO: Handle invalid LiDAR readings (inf, nan, 0.0)
        # Hint: Set invalid readings to a large safe distance
        
        if self.scan_angles is None:
            # Generate the angle for each beam relative to the robot's heading
            self.scan_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

    def control_loop(self):
        """
        Main control loop executed at 10Hz.
        TODO: Implement the Potential Field Algorithm here.
        """
        
        # 1. Calculate Attractive Force (Pull towards goal)
        
        # 2. Calculate Repulsive Force (Push away from obstacles)
        
        # 3. Calculate Total Force and Desired Heading
        
        # 4. Convert to Velocity Commands and Publish
        pass

def main(args=None):
    rclpy.init(args=args)
    planner = PotentialFieldPlanner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
