#!/usr/bin/env python3
import rospy
from stable_baselines3 import PPO
from destination import Destination

if __name__ == "__main__":
    rospy.init_node("rl_robot_train")

    # Create the environment
    env = Destination()

    # Wait for LIDAR data to be available
    rospy.loginfo("Waiting for LIDAR data...")
    while env.laser_data is None:
        rospy.sleep(0.1)

    # Initialize the PPO model
    model = PPO("MlpPolicy", env, verbose=1)

    # Train the model
    model.learn(total_timesteps=100000)

    # Save the model
    model.save("rl_robot_ppo")

    # Define the model with proper TensorBoard logging
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="/home/aqil/dodo_ws/ppo_logs/")
