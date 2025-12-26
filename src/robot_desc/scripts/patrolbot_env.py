#!/usr/bin/env python3
import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker

# NOTE: We assume you have these messages since you have the turtlebot3_msgs package.
# If this fails, we can switch to standard messages.
try:
    from turtlebot3_msgs.srv import Dqn
except ImportError:
    print("ERROR: turtlebot3_msgs not found. Please install them or ask for a version without them.")

class RLEnvironment(Node):
    def __init__(self):
        super().__init__('rl_environment')

        # --- CONFIGURATION FOR PATROL ROBOT ---
        self.goal_pose_x = 2.0  # Fixed Goal X
        self.goal_pose_y = 0.0  # Fixed Goal Y
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.robot_pose_theta = 0.0

        # RL parameters
        self.done = False
        self.succeed = False
        self.fail = False
        self.local_step = 0
        self.max_step = 600

        # Discrete angular actions (Left/Right turns)
        self.angular_vel = [0.6, 0.3, 0.0, -0.3, -0.6]

        # Laser scan settings (UPDATED FOR YOUR ROBOT)
        self.front_ranges = []
        self.front_angles = []
        self.max_beams = 24
        self.lidar_max_range = 6.0  # Changed from 3.5 to match your URDF

        # Collision settings (UPDATED)
        # 0.30 is safer for your robot size (Turtlebot was 0.15)
        self.collision_dist = 0.30 

        # ROS Setup
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, 'goal_marker', 10)
        
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_sub_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_sub_callback, 10)

        # STANDARD GAZEBO RESET SERVICE (Replaces custom TurtleBot services)
        self.reset_client = self.create_client(Empty, '/reset_simulation')

        # Services provided to the Agent (The PPO Brain talks to these)
        self.reset_environment_srv = self.create_service(Dqn, 'reset_environment', self.handle_reset_environment)
        self.rl_agent_interface_srv = self.create_service(Dqn, 'rl_agent_interface', self.handle_rl_agent_interface)

        # Initialization
        self.goal_distance = 0.0
        self.goal_angle = 0.0
        self.init_goal_distance = 0.0
        
        self.get_logger().info("Patrol Robot Environment Node Started")

    # ------------------------
    # MOVEMENT
    # ------------------------
    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(np.clip(linear_x, -0.4, 0.4))
        msg.angular.z = float(np.clip(angular_z, -1.0, 1.0))
        self.cmd_vel_pub.publish(msg)
        self.last_cmd_linear_x = msg.linear.x
        self.last_cmd_angular_z = msg.angular.z

    def stop_robot(self):
        self.publish_twist(0.0, 0.0)

    # ------------------------
    # SERVICE HANDLERS (The Bridge to PPO)
    # ------------------------
    def handle_reset_environment(self, request, response):
        """Called by PPO agent at the start of every episode"""
        self.get_logger().info("Resetting Environment...")
        
        # 1. Reset Gazebo
        req = Empty.Request()
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /reset_simulation service...')
        self.reset_client.call_async(req)

        # 2. Reset Variables
        self.done = False
        self.succeed = False
        self.fail = False
        self.local_step = 0
        self.publish_goal_marker()

        # 3. Calculate initial state
        state = self.calculate_state()
        self.init_goal_distance = state[0] * 5.0 # Un-normalize for storage
        response.state = state
        return response

    def handle_rl_agent_interface(self, request, response):
        """Called by PPO agent every single step"""
        # 1. Convert Action (0-4) to Velocity
        angular_z = float(self.angular_vel[request.action])
        
        # Simple Linear Logic: Move fast if goal is straight, slow if turning
        linear_x = 0.2 # Base speed
        if abs(angular_z) < 0.1: linear_x = 0.4 # Faster if straight

        self.publish_twist(linear_x, angular_z)

        # 2. Calculate State & Reward
        state = self.calculate_state()
        reward = self.calculate_reward()
        
        response.state = state
        response.reward = reward
        response.done = self.done
        
        if self.done:
            self.stop_robot()
            # Reset flags for next run
            self.done = False
            self.succeed = False
            self.fail = False

        return response

    # ------------------------
    # SENSORS
    # ------------------------
    def odom_sub_callback(self, msg):
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        _, _, self.robot_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        # Calculate Goal Info
        dx = self.goal_pose_x - self.robot_pose_x
        dy = self.goal_pose_y - self.robot_pose_y
        self.goal_distance = math.hypot(dx, dy)
        
        path_theta = math.atan2(dy, dx)
        goal_angle = path_theta - self.robot_pose_theta
        self.goal_angle = (goal_angle + math.pi) % (2 * math.pi) - math.pi

    def scan_sub_callback(self, scan):
        # Handle Lidar Data
        ranges = np.array(scan.ranges, dtype=np.float32)
        # Replace inf with max range (6.0 for your robot)
        ranges = np.where(np.isinf(ranges), self.lidar_max_range, ranges)
        ranges = np.where(np.isnan(ranges), self.lidar_max_range, ranges)

        self.min_obstacle_distance = float(np.min(ranges)) if ranges.size > 0 else self.lidar_max_range

        # Filter for front beams (+/- 45 degrees)
        self.front_ranges = []
        num_ranges = len(ranges)
        # Approximate indices for +/- 45 degrees
        for i, r in enumerate(ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if -0.78 < angle < 0.78: # +/- 45 degrees in radians
                self.front_ranges.append(float(r))

    # ------------------------
    # STATE & REWARD
    # ------------------------
    def calculate_state(self):
        self.local_step += 1

        # Check Termination
        if self.min_obstacle_distance < self.collision_dist:
            self.get_logger().info('CRASH!')
            self.fail = True
            self.done = True
        elif self.goal_distance < 0.25:
            self.get_logger().info('GOAL REACHED!')
            self.succeed = True
            self.done = True
        elif self.local_step >= self.max_step:
            self.get_logger().info('TIMEOUT')
            self.done = True

        # Prepare Lidar State (24 beams)
        front = list(self.front_ranges)
        # Resize to exactly self.max_beams
        if len(front) > self.max_beams:
            step = len(front) // self.max_beams
            front = front[::step][:self.max_beams]
        while len(front) < self.max_beams:
            front.append(self.lidar_max_range)

        # State Vector: [Goal Dist, Goal Angle, ...Lidar Beams...]
        state = [self.goal_distance / 5.0, self.goal_angle / math.pi] + [r for r in front]
        return state

    def calculate_reward(self):
        if self.fail: return -20.0
        if self.succeed: return 100.0

        # Reward for getting closer to goal
        reward = (self.init_goal_distance - self.goal_distance) * 2.0
        return float(reward)

    # ------------------------
    # UTILS
    # ------------------------
    def publish_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_pose_x
        marker.pose.position.y = self.goal_pose_y
        marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2
        marker.color.a = 1.0; marker.color.g = 1.0
        self.marker_pub.publish(marker)

    def euler_from_quaternion(self, quat):
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x*x + y*y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    env_node = RLEnvironment()
    rclpy.spin(env_node)
    env_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()