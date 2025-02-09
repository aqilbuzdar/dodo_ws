#!/usr/bin/env python3
import rospy
import gym
from gym import spaces
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Destination(gym.Env):
    def __init__(self):
        super(Destination, self).__init__()

        # Define action and observation space
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)  # [linear_velocity, angular_velocity]
        self.observation_space = spaces.Box(low=0, high=10, shape=(10,), dtype=np.float32)  # 10 LIDAR readings

        # ROS publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # Initialize variables
        self.laser_data = None
        self.goal_position = np.array([5.0, 5.0])  # Example goal position
        self.robot_position = np.array([0.0, 0.0])  # Example robot position

    def laser_callback(self, data):
        # Callback to get LIDAR data
        rospy.loginfo("Received LIDAR data")
        self.laser_data = np.array(data.ranges[:10])  # Use first 10 LIDAR readings

    def reset(self):
        # Reset the environment
        rospy.loginfo("Resetting environment")
        self.robot_position = np.array([0.0, 0.0])  # Reset robot position
        self.laser_data = None  # Reset LIDAR data
        rospy.sleep(2.0)  # Wait for LIDAR data to be received
        return self._get_obs()

    def step(self, action):
        # Execute one time step in the environment
        self._take_action(action)
        obs = self._get_obs()
        reward = self._get_reward()
        done = self._is_done()
        info = {}

        return obs, reward, done, info

    def _take_action(self, action):
        # Check for NaN values in actions
        if np.any(np.isnan(action)):
            rospy.logwarn("NaN values detected in actions")
            action = np.zeros(2)  # Replace NaN actions with zeros

        # Send velocity commands to the robot
        cmd_vel = Twist()
        cmd_vel.linear.x = action[0]  # Linear velocity
        cmd_vel.angular.z = action[1]  # Angular velocity
        rospy.loginfo(f"Action: linear={cmd_vel.linear.x}, angular={cmd_vel.angular.z}")
        self.cmd_vel_pub.publish(cmd_vel)

    def _get_obs(self):
        # Get observation (LIDAR data and robot position)
        if self.laser_data is None:
            rospy.logwarn("No LIDAR data received yet")
            return np.zeros(10)
        
        # Check for NaN values in LIDAR data
        if np.any(np.isnan(self.laser_data)):
            rospy.logwarn("NaN values detected in LIDAR data")
            return np.zeros(10)
        
        # Normalize LIDAR data to ensure it's within the observation space bounds
        obs = np.clip(self.laser_data, 0, 10)
        return obs

    def _get_reward(self):
        # Calculate reward
        distance_to_goal = np.linalg.norm(self.goal_position - self.robot_position)
        reward = -distance_to_goal  # Reward is negative distance to goal

        # Add penalty for collisions
        if self.laser_data is not None and np.any(self.laser_data < 0.5):  # If any obstacle is too close
            reward -= 10

        # Check for NaN values
        if np.isnan(reward):
            rospy.logwarn("NaN value detected in reward")
            reward = 0.0

        return reward

    def _is_done(self):
        # Check if the episode is done
        distance_to_goal = np.linalg.norm(self.goal_position - self.robot_position)
        if distance_to_goal < 0.5:  # If robot is close to the goal
            return True
        if self.laser_data is not None and np.any(self.laser_data < 0.3):  # If robot collides with an obstacle
            return True
        return False