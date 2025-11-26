import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    depthai_examples_dir = get_package_share_directory('depthai_examples')
    depthai_launch_dir = os.path.join(depthai_examples_dir, 'launch')

    camera_model = LaunchConfiguration('camera_model', default='OAK-D')
    tf_prefix = LaunchConfiguration('tf_prefix', default='oak')
    base_frame = LaunchConfiguration('base_frame', default='oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame', default='oak-d-base-frame')

    mode = LaunchConfiguration('mode', default='depth')
    lrcheck = LaunchConfiguration('lrcheck', default=True)
    extended = LaunchConfiguration('extended', default=False)
    subpixel = LaunchConfiguration('subpixel', default=True)
    confidence = LaunchConfiguration('confidence', default=200)
    monoResolution = LaunchConfiguration('monoResolution', default='720p')

    stereo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(depthai_launch_dir, 'stereo.launch.py')),
        launch_arguments={
            'camera_model': camera_model,
            'tf_prefix': tf_prefix,
            'base_frame': base_frame,
            'parent_frame': parent_frame,
            'mode': mode,
            'lrcheck': lrcheck,
            'extended': extended,
            'subpixel': subpixel,
            'confidence': confidence,
            'monoResolution': monoResolution,
        }.items()
    )

    obstacle_detector = Node(
        package='game_nodes',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen'
    )

    navigator = Node(
        package='game_nodes',
        executable='navigator',
        name='navigator',
        output='screen'
    )

    return LaunchDescription([
        stereo_launch,
        obstacle_detector,
        navigator
    ])
