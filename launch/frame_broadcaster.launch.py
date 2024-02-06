from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import yaml
import os

def generate_launch_description():
    # Load the YAML file with transform configurations
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')
    config_directory = os.path.join(get_package_share_directory('sensor_integration_suite'))
    yaml_file = os.path.join(config_directory, 'frames.yaml')
    
    with open(yaml_file, 'r') as file:
        transforms_config = yaml.safe_load(file)['transforms']

    # Create a list of Node actions from the loaded configurations
    transform_publishers = []
    for transform in transforms_config:
        translation = transform['translation']
        rotation = transform['rotation']
        parent_frame = transform['parent_frame']
        child_frame = transform['child_frame']

        arguments = [
            str(translation[0]), str(translation[1]), str(translation[2]),
            str(rotation[0]), str(rotation[1]), str(rotation[2]),
            parent_frame, child_frame
        ]

        transform_publishers.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=arguments
            )
        )
    transform_publishers.append(use_sim_time_arg)

    return LaunchDescription(transform_publishers)
