from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_integration_suite',  # Replace with the name of your package
            executable='zed_mini_pointcloud_publisher_node',  # Replace with the name of your executable
            name='zed_mini_pointcloud_publisher_node',
            output='screen',
            # Add any necessary parameters here
            parameters=[
                {'frame_id': 'zed_mini_frame'},
            ],
        ),
    ])
