from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the first LidarPublisherNode
        Node(
            package='sensor_integration_suite',
            executable='lidar_publisher_node',  # Replace with your actual executable name
            name='lidar_publisher_node_hz',
            parameters=[{'lidar_uri': '/dev/ttyUSB0', 
                         'topic_name': '/lidar/points_hz',
                         'frame_id':'horizontal_laser_link'}],
            output='screen'
        ),
        # Launch the second LidarPublisherNode
        Node(
            package='sensor_integration_suite',
            executable='lidar_publisher_node',  # Replace with your actual executable name
            name='lidar_publisher_node_vt',
            parameters=[{'lidar_uri': '/dev/ttyUSB1', 
                         'topic_name': '/lidar/points_vt',
                         'frame_id':'vertical_laser_link'}],
            output='screen'
        )
    ])