from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    oak_node = ComposableNode(
        package='depthai_ros_driver',
        plugin='depthai_ros_driver::Camera',
        name='oak',
        parameters=[{'pipeline_type': 'RGBD'}],
    )

    container = ComposableNodeContainer(
        name='oak_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[oak_node],
        output='screen',
    )

    return LaunchDescription([container])
