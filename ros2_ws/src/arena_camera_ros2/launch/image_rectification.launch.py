from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


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
        )
    ]

    image_processing_container = ComposableNodeContainer(
        name='image_proc_container',
        namespace=LaunchConfiguration('ns'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='ns', default_value='', description='Namespace of the raw camera topics'),
        image_processing_container
    ])