from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Launch OAK-D stereo pipeline
    oak_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('game_nodes'),
                'launch',
                'oakd_stereo.launch.py'
            )
        )
    )

    # Obstacle detector node
    obstacle_detector = Node(
        package='game_nodes',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen'
    )

    # Navigator node (ENABLED)
    navigator = Node(
        package='game_nodes',
        executable='navigator',
        name='navigator',
        output='screen'
    )

    # RViz2 for visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        oak_launch,
        obstacle_detector,
        navigator,
        rviz
    ])
