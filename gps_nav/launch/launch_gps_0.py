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
            executable='goal_pose_creator',
            name='goal_pose_creator',
            output='screen',
            parameters = [
                {'send_to_rviz': True}
            ]
        ),
        Node(
            package='gps_nav',
            #namespace='gps_nav',
            executable='vehicle_controller',
            name='vehicle_controller',
            output='screen' 
        ),

        Node(
            package='gps_nav',
            #namespace='gps_nav',
            executable='vehicle_simulator',
            name='vehicle_simulator',
            output='screen',
            parameters = [
                {'starting_position': [368964.0, 3280351.0, 0.0]},  # UTM coords + 10 E from path start
                {'starting_ang_deg': 193.0}
            ] 
        ),

        Node(
            package='gps_nav',
            #namespace='gps_nav',
            executable='motion_spec_provider',
            name='motion_spec_provider',
            output='screen',
            parameters = [
                {'look_ahead_dist': 7.0},
                {'speed': 0.2}
            ] 
        ),

        Node(
            package='tf2_ros',
            #namespace='tf2_ros',
            executable='static_transform_publisher',
            name='cdc_tf',
            arguments=["0", "0", "0", "0", "0", "0", "map", "my_frame"]
        ),

        Node(
            package='rviz2',
            namespace='rviz2',
            executable='rviz2',
            name='rviz2',
                #arguments=["-d ~/dev_ws_2022/src/gps_nav/rviz/bandshell_1_rviz.rviz"]
                arguments=["-d", config_dir + "/rviz/bandshell_1_rviz.rviz"]

        )
    ])