from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import os

def generate_launch_description():

    config_path = "/home/ubuntu/game_ws/src/game_nodes/config/oakd_pro.json"

    container = ComposableNodeContainer(
        name='oak_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[],
        output='screen'
    )

    load_nodes = LoadComposableNodes(
        target_container='oak_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depthai_ros_driver',
                plugin='depthai_ros_driver::Camera',
                name='oak',
                parameters=[config_path]
            )
        ]
    )

    return LaunchDescription([container, load_nodes])
