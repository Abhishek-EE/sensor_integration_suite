from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/frame_broadcaster_launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/dual_lidar_launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/zed_mini_launch.py'])
        ),
        Node(
            package='sensor_integration_suite',
            executable='pointcloud_fusion_node',  # Replace with your actual executable name
            name='pointcloud_fusion_node',
            parameters=[{'zed_cloud_topic_name': '/zed/zed_node/point_cloud/cloud_registered', 
                         'vertical_pc_topic_name': '/lidar/points_vt',
                         'horizontal_pc_topic_name':'/lidar/points_hz'}],
            output='screen'
        ),

    ])
