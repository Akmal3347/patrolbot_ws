#!/usr/bin/env python3
import os
import sys
import time
import threading
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from turtlebot3_msgs.srv import Dqn

import tensorflow as tf
from tensorflow.keras.layers import Dense, Input
from tensorflow.keras.models import Model

# Hide GPU warnings
try:
    tf.config.set_visible_devices([], 'GPU')
except Exception:
    pass


class TestAgent(Node):
    def __init__(self, stage_num, load_episode):
        super().__init__('test_agent')

        self.stage = int(stage_num)
        self.load_episode = int(load_episode)
        self.state_size = 26   # 2 (goal dist + angle) + 24 lidar beams
        self.action_size = 5

        # Path to your fully converged weights
        self.model_dir_path = os.path.expanduser('~/patrolbot_ws1/src/robot_desc/saved_model_ppo')

        # Build ONLY the Actor (no Critic needed for pure testing)
        self.actor = self.build_actor()

        if self.load_episode > 0:
            actor_path = os.path.join(self.model_dir_path, f"actor_ep{self.load_episode}.weights.h5")
            if os.path.exists(actor_path):
                # Build models with dummy input so weights can be loaded
                dummy = np.zeros((1, self.state_size), dtype=np.float32)
                self.actor(dummy, training=False)
                self.actor.load_weights(actor_path)
                self.get_logger().info(f"SUCCESS: Loaded optimized Actor weights from episode {self.load_episode}")
            else:
                self.get_logger().error(f"Weight file not found: {actor_path}")
                sys.exit()
        else:
            self.get_logger().error("Please specify an episode to load (e.g., 450)!")
            sys.exit()

        # ROS2 clients
        self.rl_agent_interface_client = self.create_client(Dqn, 'rl_agent_interface')
        self.reset_environment_client = self.create_client(Dqn, 'reset_environment')

    # ------------------------------------------------------------------ #
    # Model building
    # ------------------------------------------------------------------ #
    def build_actor(self):
        inputs = Input(shape=(self.state_size,), dtype=tf.float32)
        x = Dense(256, activation='relu')(inputs)
        x = Dense(256, activation='relu')(x)
        outputs = Dense(self.action_size, activation='softmax', dtype=tf.float32)(x)
        return Model(inputs, outputs)

    # ------------------------------------------------------------------ #
    # Testing Core
    # ------------------------------------------------------------------ #
    def get_action(self, state):
        state_tensor = tf.convert_to_tensor(state, dtype=tf.float32)
        probs = self.actor(state_tensor, training=False).numpy()[0]
        
        # PURE EXPLOITATION: Pick the action with the absolute highest probability.
        # Zero randomness, zero exploration.
        action = int(np.argmax(probs))
        return action

    # ------------------------------------------------------------------ #
    # ROS2 environment interface
    # ------------------------------------------------------------------ #
    def _wait_for_future(self, future, timeout=30.0):
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                raise TimeoutError("ROS2 service call timed out after 30s")
            time.sleep(0.005)
        return future.result()

    def reset_environment(self):
        while not self.reset_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for reset_environment service...")
        future = self.reset_environment_client.call_async(Dqn.Request())
        result = self._wait_for_future(future)
        state = np.array(result.state, dtype=np.float32)
        return np.reshape(state, (1, self.state_size))

    def step(self, action):
        req = Dqn.Request()
        req.action = int(action)
        while not self.rl_agent_interface_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for rl_agent_interface service...")
        future = self.rl_agent_interface_client.call_async(req)
        result = self._wait_for_future(future)
        next_state = np.reshape(np.array(result.state, dtype=np.float32), (1, self.state_size))
        return next_state, float(result.reward), bool(result.done)

    # ------------------------------------------------------------------ #
    # Main Testing Loop
    # ------------------------------------------------------------------ #
    def process(self):
        self.get_logger().info("Waiting 5s for environment to initialize...")
        time.sleep(5)

        while not self.reset_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for reset_environment service...")
        self.get_logger().info("Services ready. Starting PURE TESTING MODE.")

        test_run = 1

        # Infinite loop for continuous testing
        while rclpy.ok():
            state = self.reset_environment()
            score = 0.0
            steps = 0

            while True:
                action = self.get_action(state)
                next_state, reward, done = self.step(action)
                score += reward
                steps += 1
                state = next_state

                if done:
                    print(f"Test Run {test_run:4d} | Score: {score:8.2f} | Steps: {steps:3d}")
                    test_run += 1
                    break


def main(argv=None):
    if argv is None:
        argv = sys.argv
    
    # Kept argument parsing identical to your launch files to prevent crashing
    stage_num             = argv[1] if len(argv) > 1 else '1'
    max_training_episodes = argv[2] if len(argv) > 2 else '1000' # Ignored in testing
    load_episode          = argv[3] if len(argv) > 3 else '450'  # Defaults to 450

    rclpy.init(args=argv)
    test_agent = TestAgent(stage_num, load_episode)

    # Spin the node in a background daemon thread
    executor = MultiThreadedExecutor()
    executor.add_node(test_agent)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        test_agent.process()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        test_agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()