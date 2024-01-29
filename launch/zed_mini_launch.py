from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set the path to the ZED config file
    config = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'zedm.yaml'
    )

    return LaunchDescription([
        Node(
            package='zed_wrapper',
            executable='zed_wrapper_node',
            name='zed_wrapper_node',
            output='screen',
            parameters=[config],
        ),
    ])
