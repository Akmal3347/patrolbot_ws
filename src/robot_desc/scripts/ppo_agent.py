#!/usr/bin/env python3
import datetime
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
from tensorflow.keras.optimizers import Adam

try:
    tf.config.set_visible_devices([], 'GPU')
except Exception:
    pass

LOGGING = True
current_time = datetime.datetime.now().strftime('[%m%d-%H:%M]')


class PPOAgent(Node):
    def __init__(self, stage_num, max_training_episodes, load_episode):
        super().__init__('ppo_agent')

        self.stage = int(stage_num)
        self.max_training_episodes = int(max_training_episodes)
        self.load_episode = int(load_episode)
        self.state_size = 26   # 2 (goal dist + angle) + 24 lidar beams
        self.action_size = 5

        # PPO Hyperparameters
        self.gamma = 0.99
        self.lam = 0.95
        self.clip_ratio = 0.2
        self.learning_rate = 3e-4
        self.train_mode = True
        self.minibatch_size = 64
        self.update_epochs = 10
        self.entropy_coef = 0.05
        self.value_coef = 0.5
        self.max_grad_norm = 0.5

        # Per-episode replay buffer
        self.buffer_states = []
        self.buffer_actions = []
        self.buffer_logp = []
        self.buffer_rewards = []
        self.buffer_values = []
        self.buffer_dones = []

        # FIX: Force the path to your permanent src directory so weights aren't deleted on build
        self.model_dir_path = os.path.expanduser('~/patrolbot_ws1/src/robot_desc/saved_model_ppo')
        os.makedirs(self.model_dir_path, exist_ok=True)

        # FIX: Force the log path to your permanent src directory so TensorBoard logs aren't deleted
        self.log_dir = os.path.join(
            os.path.expanduser('~/patrolbot_ws1/src/robot_desc'),
            'ppo_logs', 'run_' + current_time
        )
        self.writer = tf.summary.create_file_writer(self.log_dir)
        self.get_logger().info(f"TensorBoard logs: {self.log_dir}")

        self.actor = self.build_actor()
        self.critic = self.build_critic()

        if self.load_episode > 0:
            actor_path = os.path.join(self.model_dir_path, f"actor_ep{self.load_episode}.weights.h5")
            critic_path = os.path.join(self.model_dir_path, f"critic_ep{self.load_episode}.weights.h5")
            if os.path.exists(actor_path) and os.path.exists(critic_path):
                # Build models with dummy input so weights can be loaded
                dummy = np.zeros((1, self.state_size), dtype=np.float32)
                self.actor(dummy, training=False)
                self.critic(dummy, training=False)
                self.actor.load_weights(actor_path)
                self.critic.load_weights(critic_path)
                self.get_logger().info(f"Loaded weights from episode {self.load_episode}")
            else:
                self.get_logger().error(f"Weight files not found for episode {self.load_episode}. Starting fresh.")
                self.load_episode = 0

        # ROS2 clients
        self.rl_agent_interface_client = self.create_client(Dqn, 'rl_agent_interface')
        self.make_environment_client = self.create_client(Empty, 'make_environment')
        self.reset_environment_client = self.create_client(Dqn, 'reset_environment')

        self.action_pub = self.create_publisher(Float32MultiArray, '/get_action', 10)
        self.result_pub = self.create_publisher(Float32MultiArray, '/result', 10)

    # ------------------------------------------------------------------ #
    # Model building
    # ------------------------------------------------------------------ #
    def build_actor(self):
        inputs = Input(shape=(self.state_size,), dtype=tf.float32)
        x = Dense(256, activation='relu')(inputs)
        x = Dense(256, activation='relu')(x)
        outputs = Dense(self.action_size, activation='softmax', dtype=tf.float32)(x)
        model = Model(inputs, outputs)
        model.compile(optimizer=Adam(learning_rate=self.learning_rate))
        return model

    def build_critic(self):
        inputs = Input(shape=(self.state_size,), dtype=tf.float32)
        x = Dense(256, activation='relu')(inputs)
        x = Dense(256, activation='relu')(x)
        outputs = Dense(1, activation='linear', dtype=tf.float32)(x)
        model = Model(inputs, outputs)
        model.compile(optimizer=Adam(learning_rate=self.learning_rate))
        return model

    # ------------------------------------------------------------------ #
    # PPO core
    # ------------------------------------------------------------------ #
    def get_action(self, state):
        """
        FIX: Use model() directly instead of model.predict() — 5-10x faster
        for single-sample inference (no Keras graph-tracing overhead per call).
        """
        state_tensor = tf.convert_to_tensor(state, dtype=tf.float32)
        probs = self.actor(state_tensor, training=False).numpy()[0].astype(np.float64)
        probs = np.clip(probs, 1e-10, 1.0)
        probs /= probs.sum()
        action = np.random.choice(self.action_size, p=probs)
        logp = float(np.log(probs[action]))
        value = float(self.critic(state_tensor, training=False).numpy()[0][0])
        return action, logp, value

    def compute_gae(self, rewards, values, dones, last_value=0.0):
        """
        FIX: Accept last_value argument.
        - Pass last_value=0.0      for true terminal episodes (collision / success).
        - Pass last_value=V(s_T)   for timeout episodes so the critic's estimate
          of future return is not incorrectly zeroed out.
        Without this fix PPO produces a large negative advantage for the last
        several steps of every timed-out episode, discouraging long navigation.
        """
        T = len(rewards)
        advantages = np.zeros(T, dtype=np.float32)
        last_gae = 0.0
        for t in reversed(range(T)):
            # FIX: use last_value for the step after the final timestep
            next_value = values[t + 1] if t + 1 < T else last_value
            mask = 1.0 - float(dones[t])
            delta = float(rewards[t]) + self.gamma * next_value * mask - float(values[t])
            last_gae = delta + self.gamma * self.lam * mask * last_gae
            advantages[t] = last_gae
        returns = advantages + values
        return advantages.astype(np.float32), returns.astype(np.float32)

    def ppo_update(self, states, actions, logp_old, returns, advantages, episode_num):
        states = tf.convert_to_tensor(states, dtype=tf.float32)
        actions = tf.convert_to_tensor(actions, dtype=tf.int32)
        logp_old = tf.convert_to_tensor(logp_old, dtype=tf.float32)
        returns = tf.convert_to_tensor(returns, dtype=tf.float32)
        advantages = tf.convert_to_tensor(advantages, dtype=tf.float32)
        advantages = (advantages - tf.reduce_mean(advantages)) / (tf.math.reduce_std(advantages) + 1e-8)

        dataset_size = states.shape[0]
        indices = np.arange(dataset_size)

        total_actor_loss = 0.0
        total_critic_loss = 0.0
        total_entropy = 0.0
        num_updates = 0

        for _ in range(self.update_epochs):
            np.random.shuffle(indices)
            for start in range(0, dataset_size, self.minibatch_size):
                mb_idx = indices[start:start + self.minibatch_size]

                mb_states = tf.gather(states, mb_idx)
                mb_actions = tf.gather(actions, mb_idx)
                mb_logp_old = tf.gather(logp_old, mb_idx)
                mb_returns = tf.gather(returns, mb_idx)
                mb_adv = tf.gather(advantages, mb_idx)

                with tf.GradientTape() as tape_a, tf.GradientTape() as tape_c:
                    probs = self.actor(mb_states, training=True)
                    one_hot = tf.one_hot(mb_actions, self.action_size)
                    selected_probs = tf.reduce_sum(probs * one_hot, axis=1)
                    logp = tf.math.log(selected_probs + 1e-10)

                    ratio = tf.exp(logp - mb_logp_old)
                    unclipped = ratio * mb_adv
                    clipped = tf.clip_by_value(ratio, 1 - self.clip_ratio, 1 + self.clip_ratio) * mb_adv
                    actor_loss = -tf.reduce_mean(tf.minimum(unclipped, clipped))

                    entropy = -tf.reduce_mean(tf.reduce_sum(probs * tf.math.log(probs + 1e-10), axis=1))
                    actor_loss = actor_loss - self.entropy_coef * entropy

                    values_pred = tf.squeeze(self.critic(mb_states, training=True), axis=1)
                    critic_loss = tf.reduce_mean(tf.square(mb_returns - values_pred)) * self.value_coef

                grads_a = tape_a.gradient(actor_loss, self.actor.trainable_variables)
                grads_c = tape_c.gradient(critic_loss, self.critic.trainable_variables)
                grads_a, _ = tf.clip_by_global_norm(grads_a, self.max_grad_norm)
                grads_c, _ = tf.clip_by_global_norm(grads_c, self.max_grad_norm)
                self.actor.optimizer.apply_gradients(zip(grads_a, self.actor.trainable_variables))
                self.critic.optimizer.apply_gradients(zip(grads_c, self.critic.trainable_variables))

                total_actor_loss += actor_loss.numpy()
                total_critic_loss += critic_loss.numpy()
                total_entropy += entropy.numpy()
                num_updates += 1

        denom = max(num_updates, 1)
        with self.writer.as_default():
            tf.summary.scalar('Loss/Actor_Loss', total_actor_loss / denom, step=episode_num)
            tf.summary.scalar('Loss/Critic_Loss', total_critic_loss / denom, step=episode_num)
            tf.summary.scalar('Loss/Entropy', total_entropy / denom, step=episode_num)

    # ------------------------------------------------------------------ #
    # ROS2 environment interface
    # FIX: Use _wait_for_future() instead of rclpy.spin_until_future_complete()
    # because the node is now spun by a background executor thread.
    # Calling spin_until_future_complete() from the main thread while another
    # thread is also spinning the same node causes a deadlock.
    # ------------------------------------------------------------------ #
    def _wait_for_future(self, future, timeout=30.0):
        """Poll a future without re-entering the executor."""
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
    # Main PPO training loop
    # ------------------------------------------------------------------ #
    def process(self):
        self.get_logger().info("Waiting 5s for environment to initialize...")
        time.sleep(5)

        # FIX: Just wait for services to be available, don't double-initialize.
        # reset_environment() already calls make_environment() internally,
        # so calling env_make() here would create a phantom episode in the
        # goal_handler's stats and double-reset the robot.
        while not self.reset_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for reset_environment service...")
        self.get_logger().info("Services ready. Starting training.")

        # Terminal reward values — used to decide whether to bootstrap last_value
        TERMINAL_REWARDS = {100.0, -50.0}  # success, collision

        start_ep = self.load_episode + 1
        end_ep = self.load_episode + self.max_training_episodes + 1

        for episode in range(start_ep, end_ep):
            state = self.reset_environment()
            score = 0.0

            self.buffer_states.clear()
            self.buffer_actions.clear()
            self.buffer_logp.clear()
            self.buffer_rewards.clear()
            self.buffer_values.clear()
            self.buffer_dones.clear()

            while True:
                action, logp, value = self.get_action(state)
                next_state, reward, done = self.step(action)
                score += reward

                self.buffer_states.append(state[0].astype(np.float32))
                self.buffer_actions.append(int(action))
                self.buffer_logp.append(float(logp))
                self.buffer_rewards.append(float(reward))
                self.buffer_values.append(float(value))
                self.buffer_dones.append(bool(done))

                state = next_state

                msg = Float32MultiArray()
                msg.data = [float(action), float(score), float(reward)]
                self.action_pub.publish(msg)

                if done:
                    # -------------------------------------------------- #
                    # FIX: Determine whether this is a true terminal state
                    # (collision / success → last_value = 0) or a timeout
                    # (robot still alive → bootstrap from critic estimate).
                    # -------------------------------------------------- #
                    last_reward = self.buffer_rewards[-1]
                    is_true_terminal = last_reward in TERMINAL_REWARDS

                    if is_true_terminal:
                        last_value = 0.0
                    else:
                        # Timeout: ask the critic what the last state is worth
                        last_state_tensor = tf.convert_to_tensor(next_state, dtype=tf.float32)
                        last_value = float(self.critic(last_state_tensor, training=False).numpy()[0][0])

                    values_np = np.array(self.buffer_values, dtype=np.float32)

                    # FIX: For truncated (timeout) episodes, unmask the last step
                    # so the bootstrap value actually propagates into the advantage.
                    # Without this, dones[-1]=True causes mask=0 which zeroes out
                    # last_value, making the bootstrap completely ineffective.
                    buffer_dones_for_gae = np.array(self.buffer_dones, dtype=np.float32)
                    if not is_true_terminal:
                        buffer_dones_for_gae[-1] = 0.0

                    advantages, returns = self.compute_gae(
                        np.array(self.buffer_rewards, dtype=np.float32),
                        values_np,
                        buffer_dones_for_gae,
                        last_value=last_value
                    )

                    self.ppo_update(
                        np.array(self.buffer_states, dtype=np.float32),
                        np.array(self.buffer_actions, dtype=np.int32),
                        np.array(self.buffer_logp, dtype=np.float32),
                        returns,
                        advantages,
                        episode
                    )

                    with self.writer.as_default():
                        tf.summary.scalar('Performance/Score', score, step=episode)
                        tf.summary.scalar('Performance/Return_Mean', float(np.mean(returns)), step=episode)
                        tf.summary.scalar('Performance/Episode_Length', len(self.buffer_rewards), step=episode)
                        self.writer.flush()

                    msg = Float32MultiArray()
                    msg.data = [float(score), float(np.mean(returns))]
                    self.result_pub.publish(msg)
                    print(f"Episode {episode:4d} | Score: {score:8.2f} | Steps: {len(self.buffer_rewards):3d} | "
                          f"Last-val: {last_value:.3f} | Terminal: {is_true_terminal}")

                    if episode % 50 == 0:
                        actor_path = os.path.join(self.model_dir_path, f"actor_ep{episode}.weights.h5")
                        critic_path = os.path.join(self.model_dir_path, f"critic_ep{episode}.weights.h5")
                        self.actor.save_weights(actor_path)
                        self.critic.save_weights(critic_path)
                        self.get_logger().info(f"Saved weights at episode {episode}")
                    break


def main(argv=None):
    if argv is None:
        argv = sys.argv
    stage_num             = argv[1] if len(argv) > 1 else '1'
    max_training_episodes = argv[2] if len(argv) > 2 else '1000'
    load_episode          = argv[3] if len(argv) > 3 else '0'

    rclpy.init(args=argv)
    ppo_agent = PPOAgent(stage_num, max_training_episodes, load_episode)

    # FIX: Spin the node in a background daemon thread so ROS2 callbacks
    # are always processed while process() runs in the main thread.
    # This prevents the deadlock that occurs when spin_until_future_complete
    # is called without an active executor on the node.
    executor = MultiThreadedExecutor()
    executor.add_node(ppo_agent)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        ppo_agent.process()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        ppo_agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()