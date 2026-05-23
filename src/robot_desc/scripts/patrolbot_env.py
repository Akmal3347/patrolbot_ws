#!/usr/bin/env python3
import os
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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

        # Discrete angular actions
        self.angular_vel = [0.6, 0.3, 0.0, -0.3, -0.6]

        # Lidar configuration
        # FIX: Use full 360° scan sampled at max_beams evenly-spaced points.
        # Previously only ±45° was used, leaving the robot blind to side and
        # rear obstacles. The state_size (26) stays unchanged because we still
        # pass exactly max_beams values — but they now cover the full circle.
        self.max_beams = 24
        self.lidar_max_range = 6.0   # matches URDF: <max>6.0</max> in the lidar sensor block
        self.full_ranges = []        # raw 360° scan (used for collision detection)
        self.sampled_ranges = []     # max_beams evenly-sampled values (used in state)
        self.min_obstacle_distance = self.lidar_max_range

        # Last command (for reward shaping)
        self.last_cmd_linear_x = 0.0
        self.last_cmd_angular_z = 0.0

        # Goal distance tracking (raw metres)
        self.init_goal_distance = 0.0
        self.prev_goal_distance = 0.0

        # Reward / collision hyperparams
        self.collision_penalty = -50.0
        self.success_reward = 100.0
        self.min_safe_dist = 0.35
        self.obstacle_reward_scale = 0.4  # increased from 0.2 — stronger avoidance signal
        self.collision_threshold = 0.18 # metres

        # Fresh scan gate for safe resets
        self._scan_received = False

        # ROS QoS
        qos = 10
        self.use_twist = (ROS_DISTRO == 'humble')
        if self.use_twist:
            self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        else:
            self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', qos)

        self.marker_pub = self.create_publisher(Marker, 'goal_marker', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_sub_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_sub_callback, qos)

        # Service clients — ReentrantCallbackGroup allows blocking .call() inside callbacks
        self.clients_callback_group = ReentrantCallbackGroup()
        self.initialize_environment_client = self.create_client(
            Goal, 'initialize_env', callback_group=self.clients_callback_group)
        self.task_succeed_client = self.create_client(
            Goal, 'task_succeed', callback_group=self.clients_callback_group)
        self.task_failed_client = self.create_client(
            Goal, 'task_failed', callback_group=self.clients_callback_group)

        # Services provided to the PPO agent
        self.make_environment_srv = self.create_service(
            Empty, 'make_environment', self.handle_make_environment,
            callback_group=self.clients_callback_group)
        self.reset_environment_srv = self.create_service(
            Dqn, 'reset_environment', self.handle_reset_environment,
            callback_group=self.clients_callback_group)
        self.rl_agent_interface_srv = self.create_service(
            Dqn, 'rl_agent_interface', self.handle_rl_agent_interface,
            callback_group=self.clients_callback_group)

        self.goal_distance = math.hypot(self.goal_pose_x, self.goal_pose_y)
        self.goal_angle = 0.0

    # ------------------------------------------------------------------ #
    # Velocity publishing
    # ------------------------------------------------------------------ #
    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        self.last_cmd_linear_x = float(linear_x)
        self.last_cmd_angular_z = float(angular_z)
        linear_x  = float(np.clip(linear_x,  -0.4, 0.4))
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
        self.publish_twist(0.0, 0.0)

    # ------------------------------------------------------------------ #
    # Goal marker
    # ------------------------------------------------------------------ #
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
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    # ------------------------------------------------------------------ #
    # Fresh scan gate
    # ------------------------------------------------------------------ #
    def _wait_for_fresh_scan(self, timeout=2.0):
        self._scan_received = False
        start = time.time()
        while not self._scan_received and (time.time() - start) < timeout:
            time.sleep(0.05)

    # ------------------------------------------------------------------ #
    # Front obstacle distance (for speed heuristic only)
    # ------------------------------------------------------------------ #
    def _front_obstacle_distance(self):
        """Minimum distance in the forward ±60° arc only.
        Used by the speed heuristic so the robot does not slow down
        because of pillars to its side or behind it."""
        if not self.sampled_ranges:
            return self.lidar_max_range
        n = len(self.sampled_ranges)
        angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
        front_mask = np.abs(angles) < math.radians(60)
        front_ranges = np.array(self.sampled_ranges)[front_mask]
        if len(front_ranges) == 0:
            return self.lidar_max_range
        return float(np.min(front_ranges))

    # ------------------------------------------------------------------ #
    # Service handlers
    # ------------------------------------------------------------------ #
    def handle_make_environment(self, request, response):
        self.get_logger().info("make_environment called")
        self.make_environment()
        return response

    def handle_reset_environment(self, request, response):
        self.get_logger().info("reset_environment called")
        self.make_environment()

        # Wait for fresh sensor data instead of blind sleep
        self._wait_for_fresh_scan(timeout=2.0)
        time.sleep(0.3)  # short extra settle time for odom

        # Reset episode state cleanly
        self.local_step = 0
        self.done = False
        self.succeed = False
        self.fail = False

        # FIX: Store raw metres, not the normalised state value.
        # Previously state[0] = goal_distance/5.0 was stored as prev_goal_distance,
        # then progress = prev - goal_distance was computed in raw metres → wrong scale.
        self.init_goal_distance = self.goal_distance   # raw metres
        self.prev_goal_distance = self.goal_distance   # raw metres

        # Use pure observation — no termination checks on reset
        state = self._build_state_vector()
        response.state = state
        return response

    def handle_rl_agent_interface(self, request, response):
        """Apply agent action, step simulation, return (state, reward, done)."""
        # --- Linear velocity: heuristic based on heading error and obstacles ---
        base_linear = 0.2
        max_linear  = 0.4
        angle_scale = max(0.0, 1.0 - abs(self.goal_angle) / math.pi)
        linear_x = base_linear + (max_linear - base_linear) * angle_scale

        # Scale down near obstacles (front sector only for this heuristic)
        # FIX: Use front ±60° arc instead of global min_obstacle_distance.
        # The global min includes side/rear beams, which caused the robot to
        # crawl when passing between pillars even though the path ahead was clear.
        front_min = self._front_obstacle_distance()
        if front_min < self.min_safe_dist:
            obstacle_scale = float(np.clip(
                (front_min - 0.1) / (self.min_safe_dist - 0.1), 0.4, 1.0))
            linear_x *= obstacle_scale

        linear_x = float(np.clip(linear_x, 0.0, max_linear))

        # Angular velocity from discrete PPO action
        angular_z = float(self.angular_vel[request.action])

        self.publish_twist(linear_x, angular_z)
        self.get_logger().debug(
            f"Step {self.local_step} | lin={linear_x:.2f} ang={angular_z:.2f} "
            f"| nearest={self.min_obstacle_distance:.2f}m")

        # Wait for Gazebo physics to propagate the command.
        # Note: time.sleep inside a callback thread is acceptable when using
        # MultiThreadedExecutor (other threads remain active), but keep it short.
        time.sleep(0.1)

        state  = self.calculate_state()
        reward = self.calculate_reward()
        done   = self.done

        response.state  = state
        response.reward = reward
        response.done   = done

        if done:
            self.done    = False
            self.succeed = False
            self.fail    = False

        return response

    # ------------------------------------------------------------------ #
    # Sensor callbacks
    # ------------------------------------------------------------------ #
    def odom_sub_callback(self, msg):
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        _, _, self.robot_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        self.goal_distance = math.hypot(
            self.goal_pose_x - self.robot_pose_x,
            self.goal_pose_y - self.robot_pose_y)

        path_theta = math.atan2(
            self.goal_pose_y - self.robot_pose_y,
            self.goal_pose_x - self.robot_pose_x)
        goal_angle = path_theta - self.robot_pose_theta
        self.goal_angle = (goal_angle + math.pi) % (2 * math.pi) - math.pi

    def scan_sub_callback(self, scan):
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.where(np.isinf(ranges), self.lidar_max_range, ranges)
        ranges = np.where(np.isnan(ranges), self.lidar_max_range, ranges)
        ranges = np.clip(ranges, 0.0, self.lidar_max_range)

        self.full_ranges = ranges.tolist()

        # Global minimum across all 360° — used for collision detection
        self.min_obstacle_distance = float(np.min(ranges)) if len(ranges) > 0 else self.lidar_max_range

        # FIX: Sample max_beams evenly from the full 360° scan.
        # This gives the agent full situational awareness (front, sides, rear)
        # without changing state_size. Previously only ±45° was visible.
        n = len(ranges)
        if n >= self.max_beams:
            indices = np.linspace(0, n - 1, self.max_beams, dtype=int)
            self.sampled_ranges = ranges[indices].tolist()
        else:
            # Pad if fewer beams than expected (shouldn't happen with 360-sample lidar)
            pad = [self.lidar_max_range] * (self.max_beams - n)
            self.sampled_ranges = ranges.tolist() + pad

        self._scan_received = True

    # ------------------------------------------------------------------ #
    # Environment reset
    # ------------------------------------------------------------------ #
    def make_environment(self):
        while not self.initialize_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for initialize_env service...')
        req = Goal.Request()
        result = self.initialize_environment_client.call(req)
        self.goal_pose_x = result.pose_x
        self.goal_pose_y = result.pose_y
        self.get_logger().info(f"Goal set to [{self.goal_pose_x:.2f}, {self.goal_pose_y:.2f}]")
        self.publish_goal_marker()

    def call_task_succeed(self):
        while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for task_succeed service...')
        self.task_succeed_client.call(Goal.Request())

    def call_task_failed(self):
        while not self.task_failed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for task_failed service...')
        self.task_failed_client.call(Goal.Request())

    # ------------------------------------------------------------------ #
    # RL calculations
    # ------------------------------------------------------------------ #
    def _build_state_vector(self):
        """Pure observation — no side effects."""
        beams = list(self.sampled_ranges) if self.sampled_ranges else [self.lidar_max_range] * self.max_beams
        if len(beams) > self.max_beams:
            beams = beams[:self.max_beams]
        else:
            beams += [self.lidar_max_range] * (self.max_beams - len(beams))

        normalized_beams = [float(r) / self.lidar_max_range for r in beams]

        state = [
            self.goal_distance / 5.0,          # normalized distance
            self.goal_angle / math.pi           # normalized angle [-1, 1]
        ] + normalized_beams                   # 24 beams, each in [0, 1]

        return state

    def calculate_state(self):
        """Step logic: increment counter, check termination, return state."""
        self.local_step += 1

        # Termination checks
        if self.min_obstacle_distance < self.collision_threshold:
            self.get_logger().info(f'Collision! dist={self.min_obstacle_distance:.3f}m')
            self.fail = self.done = True
            self.stop_robot()
            self.local_step = 0
            self.call_task_failed()
        elif self.goal_distance < 0.20:
            self.get_logger().info('Goal reached!')
            self.succeed = self.done = True
            self.stop_robot()
            self.local_step = 0
            self.call_task_succeed()
        elif self.local_step >= self.max_step:
            self.get_logger().info('Timeout.')
            self.done = True
            self.stop_robot()
            self.local_step = 0
            self.call_task_failed()

        return self._build_state_vector()

    def calculate_reward(self):
        if self.fail:
            return float(self.collision_penalty)
        if self.succeed:
            return float(self.success_reward)

        # Progress reward — allow negative so drifting away is penalized
        # FIX: prev_goal_distance is now stored in raw metres (fixed in reset),
        # so this subtraction is correctly scaled.
        progress = self.prev_goal_distance - self.goal_distance
        self.prev_goal_distance = self.goal_distance
        progress_reward = 10.0 * progress

        # Heading alignment reward
        yaw_reward = 2.0 * (1.0 - 2.0 * abs(self.goal_angle) / math.pi)

        # Forward velocity reward
        vel = self.last_cmd_linear_x
        velocity_reward = 0.5 * vel / 0.4

        # Penalties
        standing_penalty = -0.02 if vel < 0.03 and abs(self.last_cmd_angular_z) < 0.05 else 0.0
        spin_penalty = -0.05 * abs(self.last_cmd_angular_z)
        time_penalty = -0.03  # small per-step cost to encourage reaching goal quickly

        # Obstacle proximity penalty (uses sampled beams, consistent with state)
        obstacle_reward = self.compute_weighted_obstacle_reward()
        obstacle_reward = float(np.clip(obstacle_reward * self.obstacle_reward_scale, -1.0, 0.0))

        reward = (0.1  * yaw_reward
                + 1.0  * progress_reward
                + 0.2  * velocity_reward
                + 0.4  * obstacle_reward
                +        standing_penalty
                +        spin_penalty
                +        time_penalty)
        return float(reward)

    def compute_weighted_obstacle_reward(self):
        """Bounded penalty in [-1, 0]. More negative when obstacles are close."""
        if not self.sampled_ranges:
            return 0.0

        ranges = np.array(self.sampled_ranges)
        n = len(ranges)

        # Build angles corresponding to evenly-sampled beams across 360°
        angles = np.linspace(-math.pi, math.pi, n, endpoint=False)

        # FIX: Increased warning zone from 0.5m to 0.8m.
        close_mask = ranges <= 0.8
        if not np.any(close_mask):
            return 0.0

        close_ranges = ranges[close_mask]
        close_angles = angles[close_mask]

        safe = np.clip((close_ranges - 0.1) / (0.8 - 0.1), 1e-6, 1.0)
        proximity = 1.0 - safe

        weights = self.compute_directional_weights(close_angles)
        weighted = np.sum(weights * proximity) / np.sum(weights + 1e-8)
        return -float(np.clip(weighted, 0.0, 1.0))

    def compute_directional_weights(self, relative_angles, max_weight=10.0):
        """Weight beams by how directly forward they are.
        FIX: Clip negative cosines to zero so rear-hemisphere obstacles
        do not drive the avoidance penalty. Previously cos(180°)^6 = 1.0
        gave rear beams the same weight as forward beams, causing the robot
        to keep turning away from pillars it had already passed."""
        raw = np.clip(np.cos(relative_angles), 0.0, 1.0) ** 2 + 0.05
        max_raw = np.max(raw) if np.max(raw) != 0 else 1.0
        scaled = raw * (max_weight / max_raw)
        return scaled / (np.sum(scaled) + 1e-8)

    # ------------------------------------------------------------------ #
    # Quaternion → Euler
    # ------------------------------------------------------------------ #
    def euler_from_quaternion(self, quat):
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = (math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1
                 else math.asin(sinp))
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    rl_env = RLEnvironment()
    executor = MultiThreadedExecutor()
    executor.add_node(rl_env)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rl_env.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()