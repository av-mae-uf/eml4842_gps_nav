import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
config_dir = get_package_share_directory("gps_nav")

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_nav',
            #namespace='gps_nav',
            executable='route_pose_provider',
            name='route_pose_provider',
            parameters = [
                {'want_loop': True},
                {'state_defs': '{0:\'OFF\', 1:\'ON\', 2:\'OUTSIDE\', 3:\'ENTRY_EXTENSION_PT\', 4:\'EXIT_EXTENSION_PT\', 5:\'EXIT_TURN_PT\', 6:\'START\', 7:\'END\', 8:\'UTURN_PT1\', 9:\'UTURN_PT2\', 10:\'UTURN_PT3\', 11:\'CORNER\', 12:\'END_EXTENSION\'}'},
                {'pose_filename': 'stockpile/bandshell_1.txt'}
            ]
        ),
        Node(
            package='gps_nav',
            #namespace='gps_nav',
            executable='vehicle_simulator',
            name='vehicle_simulator',
            output='screen',
            parameters = [
                {'starting_position': [368964.0, 3280351.0, 0.0]},  # UTM coords + 10 E from path start
                {'starting_ang_deg': 193.0},
            ] 
        ),
    ])