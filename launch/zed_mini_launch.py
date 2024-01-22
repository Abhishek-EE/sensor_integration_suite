from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',  # Replace with the name of your package
            executable='zed_mini_point_cloud_publisher',  # Replace with the name of your executable
            name='zed_mini_point_cloud_publisher',
            output='screen',
            # Add any necessary parameters here
            # parameters=[
            #     {'param_name': 'param_value'},
            # ],
        ),
    ])
