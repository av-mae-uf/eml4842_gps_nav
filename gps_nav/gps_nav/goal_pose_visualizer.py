#!/usr/bin/python3

# Python Imports
import math
import numpy as np

# ROS2 Imports
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

# Custom Imports
from gps_nav_interfaces.msg import CurrentGoalPose

class GoalPoseVisualizer(Node):

    def __init__(self):
        super().__init__('goal_pose_visualizer')

        self.subscription = self.create_subscription(
            CurrentGoalPose, 'current_goal_pose', self.goal_pose_callback, 1)

        # prepare to publish the 'closest_point' topic for rviz
        self.publisher_closest_point = self.create_publisher(
            PoseStamped, 'closest_point_to_rviz', 1)

        # prepare to publish the 'look_ahead_pose' topic for rviz
        self.publisher_look_ahead_pose = self.create_publisher(
            PoseStamped, 'look_ahead_pose_to_rviz', 1)

    def goal_pose_callback(self, msg):

         # publish the 'closet_point' topic to rviz
        closest_pose = PoseStamped()
        closest_pose.header.frame_id = 'utm'
        closest_pose.header.stamp = msg.closest_pose.header.stamp
        closest_pose.pose.position.x = msg.closest_pose.pose.position.x
        closest_pose.pose.position.y = msg.closest_pose.pose.position.y
        closest_pose.pose.position.z = msg.closest_pose.pose.position.z
        closest_pose.pose.orientation.w = 1.0
        closest_pose.pose.orientation.x = 0.0
        closest_pose.pose.orientation.y = 0.0
        closest_pose.pose.orientation.z = 0.0
        self.publisher_closest_point.publish(closest_pose)

        # publish the 'look_ahead_pose' topic to rviz
        look_ahead_pose = PoseStamped()
        look_ahead_pose.header.frame_id = 'utm'
        look_ahead_pose.header.stamp = msg.current_goal_pose.header.stamp
        look_ahead_pose.pose.position.x = msg.current_goal_pose.pose.position.x
        look_ahead_pose.pose.position.y = msg.current_goal_pose.pose.position.y
        look_ahead_pose.pose.position.z = msg.current_goal_pose.pose.position.z
        look_ahead_pose.pose.orientation = msg.current_goal_pose.pose.orientation

        self.publisher_look_ahead_pose.publish(look_ahead_pose)


def main(args=None):
    rclpy.init(args=args)

    goal_pose_visualizer = GoalPoseVisualizer()

    rclpy.spin(goal_pose_visualizer)

    goal_pose_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()