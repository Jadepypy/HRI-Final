from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Use your working stereo node
        Node(
            package='game_nodes',
            executable='oakd_stereo',
            name='oakd_stereo',
            output='screen'
        ),

        # Obstacle detector
        Node(
            package='game_nodes',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen'
        ),

        # Navigator
        Node(
            package='game_nodes',
            executable='navigator',
            name='navigator',
            output='screen'
        ),
    ])
