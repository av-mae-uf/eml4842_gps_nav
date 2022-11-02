from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

config_dir = get_package_share_directory("gps_nav")

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_nav',
            executable='route_pose_visualizer',
            name='route_pose_visualizer',
            output='screen'
        ),
        Node(
            package='gps_nav',
            executable='goal_pose_visualizer',
            name='goal_pose_visualizer',
            output='screen'
        ),
        Node(
            package='rviz2',
            namespace='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=["-d", config_dir + "/rviz/gps_nav.rviz"]
        )
    ])