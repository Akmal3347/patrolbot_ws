#!/usr/bin/env python3
import rclpy
import random
import math
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from turtlebot3_msgs.srv import Goal
from gazebo_msgs.srv import SpawnEntity, SetEntityState


class PatrolGoalHandler(Node):
    def __init__(self):
        super().__init__('patrol_goal_handler')

        # --- Robot configuration ---
        self.robot_name = 'patrolbot_robot_final'
        self.entity_name = 'goal_sphere'

        # Fixed start position
        # FIX: Changed from (−2.0, 0.0) to (−2.0, 0.6).
        # The original y=0.0 aimed the robot directly at pillar P2 (x=−1.08, y≈0)
        # causing guaranteed collision every episode in ~44 steps with no learning signal.
        # At y=0.6 the horizontal corridor to the goal has ≥0.28m clearance from
        # every pillar surface — the robot can drive straight and reach the goal.
        self.start_x = -2.0
        self.start_y =  0.0   # kept at 0; goal_handler reset always overrides this

        # --- Goal curriculum (SUCCESS-RATE BASED, not episode-count based) ---
        # Phase 1 — FIXED goal: stays until the robot proves it can navigate.
        #   Switches to Phase 2 only when BOTH conditions are met:
        #     a) At least MIN_EPISODES_BEFORE_SWITCH episodes have been played.
        #     b) Success rate over the last WINDOW episodes >= SUCCESS_RATE_THRESHOLD.
        # Phase 2 — RANDOM goal: goal randomised within a ring around the start.
        self.MIN_EPISODES_BEFORE_SWITCH = 300   # never switch before this many episodes
        self.SUCCESS_RATE_THRESHOLD     = 0.50  # need 50% success rate over last 50 eps
        self.WINDOW                     = 50    # rolling window size for rate calculation
        self.GOAL_RADIUS_MIN = 1.5
        self.GOAL_RADIUS_MAX = 3.5

        # Fixed goal used in phase 1
        # FIX: Changed from (2.0, 0.0) to (1.8, 0.6).
        # (2.0, 0.0) required passing through pillars P2, P1, and P9 (all on x-axis).
        # (1.8, 0.6) sits at the far end of the clear y=0.6 corridor:
        #   clearance to P7 (y=1.077): 0.48m from path → 0.28m from pillar surface ✓
        #   clearance to P2 (y=−0.003): 0.60m → 0.40m surface clearance ✓
        #   clearance to P1, P9 (y≈0): 0.60m → 0.40m surface clearance ✓
        #   clearance to P5 (y=1.086): 0.49m → 0.29m surface clearance ✓
        # The robot can drive straight from spawn to goal with no obstacles in the way.
        self.fixed_goal_x = 1.8
        self.fixed_goal_y = 0.6

        # Current episode goal (updated each episode)
        self.goal_x = self.fixed_goal_x
        self.goal_y = self.fixed_goal_y

        # Rolling outcome history: True = success, False = fail/timeout
        self.outcome_history = []
        self.phase = 'fixed'   # FORCE Phase 1: 'fixed'

        self.cb_group = ReentrantCallbackGroup()

        # Services this node provides
        self.srv         = self.create_service(Goal, 'initialize_env', self.init_callback,    callback_group=self.cb_group)
        self.success_srv = self.create_service(Goal, 'task_succeed',   self.success_callback, callback_group=self.cb_group)
        self.fail_srv    = self.create_service(Goal, 'task_failed',    self.fail_callback,    callback_group=self.cb_group)

        # Clients this node calls
        self.spawn_client     = self.create_client(SpawnEntity,    '/spawn_entity',              callback_group=self.cb_group)
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state',   callback_group=self.cb_group)

        self.spawned        = False
        self.total_episodes = 0
        self.success_count  = 0

        self.get_logger().info(
            f'PatrolGoalHandler ready. PHASE 1 ONLY MODE. Fixed goal at ({self.fixed_goal_x}, {self.fixed_goal_y}).'
        )

    # ------------------------------------------------------------------ #
    # Service callbacks
    # ------------------------------------------------------------------ #
    def init_callback(self, request, response):
        self.total_episodes += 1

        # Pick goal for this episode
        self.goal_x, self.goal_y = self._sample_goal()

        self.get_logger().info(
            f"Ep {self.total_episodes}: goal=({self.goal_x:.2f}, {self.goal_y:.2f}) "
            f"[phase={self.phase}]")

        # Reset robot position (blocking)
        if not self.reset_robot_position():
            self.get_logger().error("Robot reset FAILED — check entity name and Gazebo state.")

        # Spawn or teleport goal marker
        if not self.spawned:
            self.spawn_goal_initially(self.goal_x, self.goal_y)
            self.spawned = True
        else:
            # FIX: Use blocking call so the goal is at the new position before
            # the env node reads it. Previously call_async was fire-and-forget,
            # causing stale goal observations on the first step of each episode.
            self.move_goal_teleport(self.goal_x, self.goal_y)

        response.pose_x = float(self.goal_x)
        response.pose_y = float(self.goal_y)
        return response

    def success_callback(self, request, response):
        self.success_count += 1
        self.outcome_history.append(True)
        self._trim_history()
        self._check_curriculum_graduation()
        self._print_stats("SUCCESS")
        return response

    def fail_callback(self, request, response):
        self.outcome_history.append(False)
        self._trim_history()
        self._check_curriculum_graduation()
        self._print_stats("FAILED")
        return response

    # ------------------------------------------------------------------ #
    # Curriculum logic
    # ------------------------------------------------------------------ #
    def _trim_history(self):
        """Keep only the last WINDOW outcomes."""
        if len(self.outcome_history) > self.WINDOW:
            self.outcome_history = self.outcome_history[-self.WINDOW:]

    def _rolling_success_rate(self):
        if not self.outcome_history:
            return 0.0
        return sum(self.outcome_history) / len(self.outcome_history)

    def _check_curriculum_graduation(self):
        """
        FIXED Phase 1 Only: Graduation logic commented out to stay in Phase 1.
        """
        # if self.phase == 'random':
        #     return  # already graduated

        # rate = self._rolling_success_rate()
        # if (self.total_episodes >= self.MIN_EPISODES_BEFORE_SWITCH
        #         and len(self.outcome_history) >= self.WINDOW
        #         and rate >= self.SUCCESS_RATE_THRESHOLD):
        #     self.phase = 'random'
        #     self.get_logger().info(
        #         f"*** CURRICULUM GRADUATED to RANDOM goals at episode "
        #         f"{self.total_episodes} (success rate {rate*100:.1f}% over last {self.WINDOW} eps) ***"
        #     )
        pass

    def _sample_goal(self):
        """
        FORCE Phase 1: Always return the fixed goal.
        """
        if self.phase == 'fixed':
            return self.fixed_goal_x, self.fixed_goal_y

        # Randomized Phase 2 logic commented out
        # for _ in range(50):
        #     r     = random.uniform(self.GOAL_RADIUS_MIN, self.GOAL_RADIUS_MAX)
        #     theta = random.uniform(-math.pi, math.pi)
        #     gx = self.start_x + r * math.cos(theta)
        #     gy = self.start_y + r * math.sin(theta)
        #     if math.hypot(gx - self.start_x, gy - self.start_y) >= self.GOAL_RADIUS_MIN:
        #         return round(gx, 3), round(gy, 3)

        return self.fixed_goal_x, self.fixed_goal_y

    # ------------------------------------------------------------------ #
    # Robot reset
    # ------------------------------------------------------------------ #
    def reset_robot_position(self):
        """Blocking robot reset. Faces robot directly toward the fixed goal."""
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /gazebo/set_entity_state...')

        req = SetEntityState.Request()
        req.state.name = self.robot_name

        # In phase 'fixed': spawn at exact start position facing the goal (yaw=0).
        if self.phase == 'fixed':
            req.state.pose.position.x = float(self.start_x)
            req.state.pose.position.y = 0.6        # clear corridor
            req.state.pose.position.z = 0.0
            req.state.pose.orientation.z = 0.0    # face forward (+X toward goal)
            req.state.pose.orientation.w = 1.0
        
        # Phase 2 logic commented out (never reached as phase is locked to 'fixed')
        # else:
        #     side = random.choice([-1, 1])
        #     req.state.pose.position.x = float(self.start_x)
        #     req.state.pose.position.y = side * 0.5
        #     req.state.pose.position.z = 0.15
        #     # ±45° yaw for randomised starts in phase 2
        #     req.state.pose.orientation.z = side * 0.383
        #     req.state.pose.orientation.w = 0.924

        req.state.twist.linear.x  = 0.0
        req.state.twist.angular.z = 0.0

        result = self.set_state_client.call(req)
        return result.success

    # ------------------------------------------------------------------ #
    # Goal entity management
    # ------------------------------------------------------------------ #
    def move_goal_teleport(self, x, y):
        """
        FIX: Blocking call so the goal is guaranteed to have moved before
        init_callback returns and the env node reads the new goal position.
        """
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /gazebo/set_entity_state...')

        req = SetEntityState.Request()
        req.state.name = self.entity_name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = 0.15
        req.state.pose.orientation.w = 1.0
        self.set_state_client.call(req)

    def spawn_goal_initially(self, x, y):
        """Spawn the visual goal marker for the first episode."""
        req = SpawnEntity.Request()
        req.name = self.entity_name
        req.xml = (
            '<?xml version="1.0" ?>'
            '<sdf version="1.5"><model name="goal_sphere"><static>true</static>'
            '<link name="link"><visual name="visual"><geometry>'
            '<sphere><radius>0.2</radius></sphere></geometry>'
            '<material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse>'
            '</material></visual></link></model></sdf>'
        )
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = 0.05
        # Fire-and-forget is acceptable here — this only runs once at startup
        self.spawn_client.call_async(req)

    # ------------------------------------------------------------------ #
    # Stats
    # ------------------------------------------------------------------ #
    def _print_stats(self, status):
        rate = (self.success_count / self.total_episodes * 100.0
                if self.total_episodes > 0 else 0.0)
        rolling = self._rolling_success_rate() * 100.0
        self.get_logger().info(
            f"[{status}] Ep {self.total_episodes} | "
            f"Successes: {self.success_count} | Overall: {rate:.1f}% | "
            f"Rolling({self.WINDOW}): {rolling:.1f}% | Phase: {self.phase}"
        )


def main():
    rclpy.init()
    node = PatrolGoalHandler()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()