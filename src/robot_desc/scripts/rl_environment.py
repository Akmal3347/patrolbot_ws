#!/usr/bin/env python3
import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from turtlebot3_msgs.srv import Goal, Dqn

ROS_DISTRO = os.environ.get('ROS_DISTRO')


class RLEnvironment(Node):
    def __init__(self):
        super().__init__('rl_environment')

        # Robot and goal states
        self.goal_pose_x = 2.0
        self.goal_pose_y = 2.0
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.robot_pose_theta = 0.0

        # RL parameters
        self.done = False
        self.succeed = False
        self.fail = False
        self.local_step = 0
        self.max_step = 600

        # Discrete angular actions (reduced magnitudes)
        # safer turns: ±0.6, ±0.3 rad/s
        self.angular_vel = [0.6, 0.3, 0.0, -0.3, -0.6]

        # Laser scan / beams
        self.front_ranges = []
        self.front_angles = []
        self.max_beams = 24
        self.min_obstacle_distance = 3.5

        # Last command (for reward shaping)
        self.last_cmd_linear_x = 0.0
        self.last_cmd_angular_z = 0.0

        # Goal distance history
        self.init_goal_distance = 0.0
        self.prev_goal_distance = 0.0

        # Collision / reward hyperparams (tunable)
        self.collision_penalty = -10.0    # softened from -50
        self.success_reward = 100.0
        self.min_safe_dist = 0.5          # used to scale speed
        self.obstacle_reward_scale = 0.2  # factor to tune obstacle penalty magnitude

        # ROS QoS
        qos_group = 10
        self.use_twist = True if ROS_DISTRO == 'humble' else False
        if self.use_twist:
            self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_group)
        else:
            self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', qos_group)

        # Publishers & Subscribers
        self.marker_pub = self.create_publisher(Marker, 'goal_marker', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_sub_callback, qos_group)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_sub_callback, qos_group)

        # Service clients
        self.clients_callback_group = MutuallyExclusiveCallbackGroup()
        self.initialize_environment_client = self.create_client(
            Goal, 'initialize_env', callback_group=self.clients_callback_group
        )
        self.task_succeed_client = self.create_client(
            Goal, 'task_succeed', callback_group=self.clients_callback_group
        )
        self.task_failed_client = self.create_client(
            Goal, 'task_failed', callback_group=self.clients_callback_group
        )

        # Services
        self.make_environment_srv = self.create_service(Empty, 'make_environment', self.handle_make_environment)
        self.reset_environment_srv = self.create_service(Dqn, 'reset_environment', self.handle_reset_environment)
        self.rl_agent_interface_srv = self.create_service(Dqn, 'rl_agent_interface', self.handle_rl_agent_interface)

        # Safety defaults
        self.goal_distance = math.hypot(self.goal_pose_x - self.robot_pose_x,
                                        self.goal_pose_y - self.robot_pose_y)
        self.goal_angle = 0.0

    # ------------------------
    # Twist helper
    # ------------------------
    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        """Publish velocity command continuously."""
        self.last_cmd_linear_x = float(linear_x)
        self.last_cmd_angular_z = float(angular_z)

        # clamp for safety
        max_linear = 0.4
        linear_x = float(np.clip(linear_x, -max_linear, max_linear))
        angular_z = float(np.clip(angular_z, -1.0, 1.0))

        if self.use_twist:
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.cmd_vel_pub.publish(msg)
        else:
            ts = TwistStamped()
            ts.header.stamp = self.get_clock().now().to_msg()
            ts.twist.linear.x = linear_x
            ts.twist.angular.z = angular_z
            self.cmd_vel_pub.publish(ts)

    def stop_robot(self):
        """Stop robot immediately."""
        self.publish_twist(0.0, 0.0)
        self.last_cmd_linear_x = 0.0
        self.last_cmd_angular_z = 0.0

    # ------------------------
    # Goal marker
    # ------------------------
    def publish_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_pose_x
        marker.pose.position.y = self.goal_pose_y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    # ------------------------
    # Service handlers
    # ------------------------
    def handle_make_environment(self, request, response):
        self.get_logger().info("make_environment service called")
        self.make_environment()
        return response

    def handle_reset_environment(self, request, response):
        self.get_logger().info("reset_environment service called")
        self.make_environment()
        state = self.calculate_state()
        self.init_goal_distance = state[0] if len(state) > 0 else 0.0
        self.prev_goal_distance = self.init_goal_distance
        response.state = state
        return response

    def handle_rl_agent_interface(self, request, response):
        """Main interface for agent actions."""
        # Linear forward velocity base and cap
        base_linear = 0.2
        max_linear = 0.4
        angle_scale = max(0.0, 1 - abs(getattr(self, 'goal_angle', 0.0)) / math.pi)
        linear_x = base_linear + (max_linear - base_linear) * angle_scale
        linear_x = float(np.clip(linear_x, 0.0, max_linear))

        # Slow down near obstacles: compute obstacle_scale in [0,1]
        front_min = min(self.front_ranges) if self.front_ranges else 3.5
        # If front_min >= min_safe_dist -> obstacle_scale = 1.0 (no slow down)
        if front_min >= self.min_safe_dist:
            obstacle_scale = 1.0
        else:
            # linearly scale 0 when extremely close -> 1 when at min_safe_dist
            obstacle_scale = float(np.clip((front_min - 0.1) / (self.min_safe_dist - 0.1), 0.0, 1.0))

        linear_x *= obstacle_scale
        linear_x = float(np.clip(linear_x, 0.0, max_linear))  # final clamp

        # Angular velocity from discrete action (bounded)
        angular_z = float(self.angular_vel[request.action])

        # Publish velocities
        self.publish_twist(linear_x, angular_z)

        # Compute state, reward, done
        state = self.calculate_state()
        reward = self.calculate_reward()
        done = self.done

        response.state = state
        response.reward = reward
        response.done = done

        if done:
            # robot already stopped in calculate_state
            # reset ephemeral flags after giving the terminal transition to agent
            self.done = False
            self.succeed = False
            self.fail = False

        return response

    # ------------------------
    # Subscribers
    # ------------------------
    def odom_sub_callback(self, msg):
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        _, _, self.robot_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        goal_distance = math.hypot(self.goal_pose_x - self.robot_pose_x,
                                   self.goal_pose_y - self.robot_pose_y)
        path_theta = math.atan2(self.goal_pose_y - self.robot_pose_y,
                                self.goal_pose_x - self.robot_pose_x)
        goal_angle = path_theta - self.robot_pose_theta
        # normalize to [-pi, pi]
        goal_angle = (goal_angle + math.pi) % (2 * math.pi) - math.pi

        self.goal_distance = goal_distance
        self.goal_angle = goal_angle

    def scan_sub_callback(self, scan):
        ranges = np.array(scan.ranges, dtype=np.float32)
        # replace inf/nan with a large finite distance
        ranges = np.where(np.isinf(ranges), 3.5, ranges)
        ranges = np.where(np.isnan(ranges), 3.5, ranges)

        self.front_ranges = []
        self.front_angles = []
        for i, distance in enumerate(ranges):
            angle = scan.angle_min + i * scan.angle_increment
            ang = ((angle + math.pi) % (2*math.pi)) - math.pi
            # keep beams within +-45 degrees
            if -math.pi/4 <= ang <= math.pi/4:
                self.front_ranges.append(float(distance))
                self.front_angles.append(float(ang))

        self.min_obstacle_distance = float(np.min(ranges)) if ranges.size > 0 else 3.5

    # ------------------------
    # Environment functions
    # ------------------------
    def make_environment(self):
        while not self.initialize_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for initialize_env service...')
        future = self.initialize_environment_client.call_async(Goal.Request())
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.goal_pose_x = result.pose_x
        self.goal_pose_y = result.pose_y
        self.get_logger().info(f"Goal initialized at [{self.goal_pose_x}, {self.goal_pose_y}]")
        self.publish_goal_marker()

    def call_task_succeed(self):
        while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for task_succeed service...')
        future = self.task_succeed_client.call_async(Goal.Request())
        rclpy.spin_until_future_complete(self, future)

    def call_task_failed(self):
        while not self.task_failed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for task_failed service...')
        future = self.task_failed_client.call_async(Goal.Request())
        rclpy.spin_until_future_complete(self, future)

    # ------------------------
    # RL calculations
    # ------------------------
    def calculate_state(self):
        self.goal_distance = getattr(self, 'goal_distance', math.hypot(self.goal_pose_x - self.robot_pose_x,
                                                                       self.goal_pose_y - self.robot_pose_y))
        self.local_step += 1

        # Episode termination: collision, success, or timeout
        if getattr(self, 'min_obstacle_distance', 3.5) < 0.15:
            self.get_logger().info('Collision happened')
            self.fail = True
            self.done = True
            self.stop_robot()
            self.local_step = 0
            self.call_task_failed()
        elif self.goal_distance < 0.20:
            self.get_logger().info('Goal Reached')
            self.succeed = True
            self.done = True
            self.stop_robot()
            self.local_step = 0
            self.call_task_succeed()
        elif self.local_step >= self.max_step:
            self.get_logger().info('Time out!')
            self.fail = True
            self.done = True
            self.stop_robot()
            self.local_step = 0
            self.call_task_failed()

        # Fixed-length front beams (pad with large distance)
        front = list(self.front_ranges) if self.front_ranges else []
        if len(front) >= self.max_beams:
            front = front[:self.max_beams]
        else:
            front = front + [3.5] * (self.max_beams - len(front))

        state = [self.goal_distance / 5.0, getattr(self, 'goal_angle', 0.0) / math.pi] + [float(r) for r in front]
        return state

    def calculate_reward(self):
        # Terminal rewards first
        if self.fail:
            return float(self.collision_penalty)
        if self.succeed:
            return float(self.success_reward)

        # Dense shaping rewards
        goal_distance = getattr(self, 'goal_distance', math.hypot(self.goal_pose_x - self.robot_pose_x,
                                                                  self.goal_pose_y - self.robot_pose_y))
        prev = getattr(self, 'prev_goal_distance', goal_distance)
        progress = max(0.0, prev - goal_distance)
        self.prev_goal_distance = goal_distance
        progress_reward = 10.0 * progress   # reduced multiplier (was 10.0)

        yaw_reward = 2.0 * (1.0 - (2.0 * abs(self.goal_angle)/math.pi))

        vel = getattr(self, 'last_cmd_linear_x', 0.0)
        velocity_reward = 0.5 * vel / 0.5  # normalized with max_linear

        standing_penalty = -0.02 if vel < 0.03 and abs(getattr(self, 'last_cmd_angular_z', 0.0)) < 0.05 else 0.0
        spin_penalty = -0.001 * abs(getattr(self, 'last_cmd_angular_z', 0.0))

        obstacle_reward = self.compute_weighted_obstacle_reward()
        # scale obstacle_reward to small magnitude so it doesn't dominate
        obstacle_reward = float(np.clip(obstacle_reward * self.obstacle_reward_scale, -1.0, 0.0))

        reward = 0.5 * yaw_reward + 1.0 * progress_reward + 0.2 * velocity_reward + 0.4 * obstacle_reward + standing_penalty + spin_penalty
        return float(reward)

    def compute_weighted_obstacle_reward(self):
        """Return a bounded negative value in [-1, 0]. More negative when obstacles are closer in front."""
        if not self.front_ranges or not self.front_angles:
            return 0.0
        front_ranges = np.array(self.front_ranges)
        front_angles = np.array(self.front_angles)
        # consider beams within a 'close' threshold (0.5 m)
        close_mask = front_ranges <= 0.5
        if not np.any(close_mask):
            return 0.0

        close_ranges = front_ranges[close_mask]
        close_angles = front_angles[close_mask]

        # compute simple proximity penalty per beam: 1.0 when distance==0.1, 0.0 when distance>=0.5
        safe = np.clip((close_ranges - 0.1) / (0.5 - 0.1), 1e-6, 1.0)
        proximity = 1.0 - safe  # 1.0 when very close, 0 when at threshold

        # directional weighting (front-centered)
        weights = self.compute_directional_weights(close_angles, max_weight=10.0)
        weighted = np.sum(weights * proximity) / np.sum(weights + 1e-8)

        # map weighted in [0,1] to penalty in [-1,0]
        return -float(np.clip(weighted, 0.0, 1.0))

    def compute_directional_weights(self, relative_angles, max_weight=10.0):
        power = 6
        raw_weights = (np.cos(relative_angles))**power + 0.1
        max_raw = np.max(raw_weights) if np.max(raw_weights) != 0 else 1.0
        scaled_weights = raw_weights * (max_weight / max_raw)
        normalized_weights = scaled_weights / (np.sum(scaled_weights) + 1e-8)
        return normalized_weights

    # ------------------------
    # Quaternion -> Euler
    # ------------------------
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
    rl_env = RLEnvironment()
    try:
        while rclpy.ok():
            rclpy.spin_once(rl_env, timeout_sec=0.02)
    except KeyboardInterrupt:
        pass
    finally:
        rl_env.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
