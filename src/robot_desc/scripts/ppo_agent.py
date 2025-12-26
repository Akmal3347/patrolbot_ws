import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import tensorflow as tf
from tensorflow.keras.layers import Dense, Input
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import Adam

import datetime
import sys
import time
import numpy as np
import rclpy # Import ROS *after* TensorFlow

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from turtlebot3_msgs.srv import Dqn



# Disable GPU if causing issues, otherwise optional
try:
    tf.config.set_visible_devices([], 'GPU')
except Exception:
    pass

class PPOAgent(Node):
    def __init__(self, stage_num, max_training_episodes):
        super().__init__('ppo_agent')

        # Basic RL settings
        self.stage = int(stage_num)
        self.max_training_episodes = int(max_training_episodes)
        self.state_size = 26
        self.action_size = 5

        # PPO Hyperparameters
        self.gamma = 0.99
        self.lam = 0.95
        self.clip_ratio = 0.2
        self.learning_rate = 3e-4
        self.train_mode = True
        self.minibatch_size = 64
        self.update_epochs = 10
        self.entropy_coef = 0.01
        self.value_coef = 0.5
        self.max_grad_norm = 0.5

        # Replay buffer
        self.buffer_states = []
        self.buffer_actions = []
        self.buffer_logp = []
        self.buffer_rewards = []
        self.buffer_values = []
        self.buffer_dones = []

        # Model saving path
        self.model_dir_path = os.path.join(os.getcwd(), 'saved_models')
        os.makedirs(self.model_dir_path, exist_ok=True)

        # Actor-Critic models
        self.actor = self.build_actor()
        self.critic = self.build_critic()

        # ROS2 communication
        self.rl_agent_interface_client = self.create_client(Dqn, 'rl_agent_interface')
        self.reset_environment_client = self.create_client(Dqn, 'reset_environment')
        
        # Removed 'make_environment' client (not needed for custom robot)

        self.action_pub = self.create_publisher(Float32MultiArray, '/get_action', 10)
        self.result_pub = self.create_publisher(Float32MultiArray, '/result', 10)

        self.get_logger().info("PPO Agent Initialized. Models created.")

    # -------------------------- Model creation -------------------------- #
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

    # -------------------------- PPO Core -------------------------- #
    def get_action(self, state):
        probs = self.actor.predict(state, verbose=0)[0].astype(np.float64)
        probs = np.clip(probs, 1e-10, 1.0)
        probs = probs / probs.sum()
        action = np.random.choice(self.action_size, p=probs)
        logp = float(np.log(probs[action]))
        value = float(self.critic.predict(state, verbose=0)[0][0])
        return action, logp, value

    def compute_gae(self, rewards, values, dones):
        T = len(rewards)
        advantages = np.zeros(T, dtype=np.float32)
        last_gae = 0.0
        for t in reversed(range(T)):
            next_value = values[t + 1] if t + 1 < T else 0.0
            mask = 1.0 - float(dones[t])
            delta = float(rewards[t]) + self.gamma * next_value * mask - float(values[t])
            last_gae = delta + self.gamma * self.lam * mask * last_gae
            advantages[t] = last_gae
        returns = advantages + values
        return advantages.astype(np.float32), returns.astype(np.float32)

    def ppo_update(self, states, actions, logp_old, returns, advantages):
        states = tf.convert_to_tensor(states, dtype=tf.float32)
        actions = tf.convert_to_tensor(actions, dtype=tf.int32)
        logp_old = tf.convert_to_tensor(logp_old, dtype=tf.float32)
        returns = tf.convert_to_tensor(returns, dtype=tf.float32)
        advantages = tf.convert_to_tensor(advantages, dtype=tf.float32)
        
        # Normalize advantages
        advantages = (advantages - tf.reduce_mean(advantages)) / (tf.math.reduce_std(advantages) + 1e-8)

        dataset_size = states.shape[0]
        indices = np.arange(dataset_size)

        for _ in range(self.update_epochs):
            np.random.shuffle(indices)
            for start in range(0, dataset_size, self.minibatch_size):
                end = start + self.minibatch_size
                mb_idx = indices[start:end]

                mb_states = tf.gather(states, mb_idx)
                mb_actions = tf.gather(actions, mb_idx)
                mb_logp_old = tf.gather(logp_old, mb_idx)
                mb_returns = tf.gather(returns, mb_idx)
                mb_adv = tf.gather(advantages, mb_idx)

                with tf.GradientTape() as tape_a, tf.GradientTape() as tape_c:
                    probs = self.actor(mb_states, training=True)
                    one_hot_actions = tf.one_hot(mb_actions, self.action_size)
                    selected_probs = tf.reduce_sum(probs * one_hot_actions, axis=1)
                    logp = tf.math.log(selected_probs + 1e-10)

                    ratio = tf.exp(logp - mb_logp_old)
                    unclipped = ratio * mb_adv
                    clipped = tf.clip_by_value(ratio, 1.0 - self.clip_ratio, 1.0 + self.clip_ratio) * mb_adv
                    actor_loss = -tf.reduce_mean(tf.minimum(unclipped, clipped))
                    
                    entropy = -tf.reduce_mean(tf.reduce_sum(probs * tf.math.log(probs + 1e-10), axis=1))
                    actor_loss = actor_loss - self.entropy_coef * entropy

                    values_pred = tf.squeeze(self.critic(mb_states, training=True), axis=1)
                    critic_loss = tf.reduce_mean(tf.square(mb_returns - values_pred)) * self.value_coef
                    total_loss = actor_loss + critic_loss

                grads_a = tape_a.gradient(actor_loss, self.actor.trainable_variables)
                grads_c = tape_c.gradient(critic_loss, self.critic.trainable_variables)
                
                grads_a, _ = tf.clip_by_global_norm(grads_a, self.max_grad_norm)
                grads_c, _ = tf.clip_by_global_norm(grads_c, self.max_grad_norm)

                self.actor.optimizer.apply_gradients(zip(grads_a, self.actor.trainable_variables))
                self.critic.optimizer.apply_gradients(zip(grads_c, self.critic.trainable_variables))

    # -------------------------- ROS2 Env -------------------------- #
    def reset_environment(self):
        while not self.reset_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for reset_environment service...")
        future = self.reset_environment_client.call_async(Dqn.Request())
        rclpy.spin_until_future_complete(self, future)
        state = np.array(future.result().state, dtype=np.float32)
        return np.reshape(state, (1, self.state_size))

    def step(self, action):
        req = Dqn.Request()
        req.action = int(action)
        while not self.rl_agent_interface_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for rl_agent_interface service...")
        future = self.rl_agent_interface_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        next_state = np.reshape(np.array(result.state, dtype=np.float32), (1, self.state_size))
        return next_state, float(result.reward), bool(result.done)

    # -------------------------- Main Loop -------------------------- #
    def process(self):
        self.get_logger().info("Waiting 3 seconds for connection...")
        time.sleep(3)

        for episode in range(1, self.max_training_episodes + 1):
            state = self.reset_environment()
            score = 0.0
            
            self.buffer_states.clear(); self.buffer_actions.clear(); self.buffer_logp.clear()
            self.buffer_rewards.clear(); self.buffer_values.clear(); self.buffer_dones.clear()

            while True:
                action, logp, value = self.get_action(state)
                next_state, reward, done = self.step(action)
                score += reward

                self.buffer_states.append(state[0])
                self.buffer_actions.append(int(action))
                self.buffer_logp.append(float(logp))
                self.buffer_rewards.append(float(reward))
                self.buffer_values.append(float(value))
                self.buffer_dones.append(bool(done))

                state = next_state

                if done:
                    values_np = np.array(self.buffer_values, dtype=np.float32)
                    advantages, returns = self.compute_gae(
                        np.array(self.buffer_rewards, dtype=np.float32),
                        values_np,
                        np.array(self.buffer_dones, dtype=np.float32)
                    )

                    self.ppo_update(
                        np.array(self.buffer_states, dtype=np.float32),
                        np.array(self.buffer_actions, dtype=np.int32),
                        np.array(self.buffer_logp, dtype=np.float32),
                        returns,
                        advantages
                    )

                    print(f"Episode {episode} | Score: {score:.2f}")

                    if episode % 50 == 0:
                        self.actor.save(os.path.join(self.model_dir_path, f"actor_ep{episode}.h5"))
                    break

def main(argv=None):
    if argv is None:
        argv = sys.argv
    stage_num = '1'
    max_training_episodes = '3000'

    rclpy.init(args=argv)
    ppo_agent = PPOAgent(stage_num, max_training_episodes)
    try:
        ppo_agent.process()
    finally:
        ppo_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()