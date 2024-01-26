from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def generate_launch_description():
    # Load the frames.yaml file
    # config_directory = os.path.join(get_package_share_directory('sensor_integration_suite'), 'config')
    config_directory = os.path.join(get_package_share_directory('sensor_integration_suite'))
    frames_yaml = os.path.join(config_directory, 'frames.yaml')
    # with open(frames_yaml, 'r') as file:
    #     frames_config = yaml.safe_load(file)

    return LaunchDescription([
        Node(
            package='sensor_integration_suite',
            executable='frame_broadcaster',
            name='frame_broadcaster_node',
            parameters=[frames_yaml],
        ),
        Node(
            package='sensor_integration_suite',
            executable='lidar_publisher_node',  # Replace with your actual executable name
            name='lidar_publisher_node_hz',
            parameters=[{'lidar_uri': '/dev/ttyUSB0', 'topic_name': '/lidar/points_hz','frame_id':'lidar_frame_hz'}],
            output='screen'
        ),
        # Launch the second LidarPublisherNode
        Node(
            package='sensor_integration_suite',
            executable='lidar_publisher_node',  # Replace with your actual executable name
            name='lidar_publisher_node_vt',
            parameters=[{'lidar_uri': '/dev/ttyUSB1', 'topic_name': '/lidar/points_vt','frame_id':'lidar_frame_vt'}],
            output='screen'
        ),
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
