import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_color_node',
            namespace=LaunchConfiguration('ns'),
            remappings=[
                ('image', 'image_raw'),
                ('image_rect', 'image_rect_color')
            ],
        ),
        ComposableNode(
            package='arena_camera_ros2',
            plugin='arena_camera_ros2::ArenaCameraNode',
            name='camera_driver_node',
            namespace=LaunchConfiguration('ns'),
            parameters=[
                LaunchConfiguration('param_file')
            ]
        )
    ]

    camera_container = ComposableNodeContainer(
        name='image_proc_container',
        namespace=LaunchConfiguration('ns'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument('param_file', default_value=os.path.join(get_package_share_directory('arena_camera_ros2'), 'config', 'single_cam.yaml'), description='Full path to configuration parameter file'),
        DeclareLaunchArgument(name='ns', default_value='', description='Namespace of the raw camera topics'),
        camera_container
    ])