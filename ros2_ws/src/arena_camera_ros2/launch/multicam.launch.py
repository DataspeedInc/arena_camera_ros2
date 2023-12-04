import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    multicam_node = Node(
        package='arena_camera_ros2',
        executable='arena_camera_ros2_multicam',
        name='arena_camera_multicam',
        output='screen',
        parameters=[
            LaunchConfiguration('param_file')
        ]
    )

    low_res_image_rectification = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('arena_camera_ros2'), 'launch', 'image_rectification.launch.py'
        )]),
        launch_arguments={'ns': 'low_res_camera'}.items()
    )

    high_res_image_rectification = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('arena_camera_ros2'), 'launch', 'image_rectification.launch.py'
        )]),
        launch_arguments={'ns': 'high_res_camera'}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('param_file', default_value=os.path.join(get_package_share_directory('arena_camera_ros2'), 'config', 'multicam.yaml'), description='Full path to configuration parameter file'),
        multicam_node,
        low_res_image_rectification,
        high_res_image_rectification
    ])