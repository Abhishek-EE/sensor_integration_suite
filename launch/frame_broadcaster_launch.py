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
            parameters=[{'frames_yaml':frames_yaml}],
        )
    ])