from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time')
    camera_model_arg = LaunchConfiguration('camera_model', default='zedm')
    launch_file_dir = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'launch'
    )
    sensor_integration_dir = os.path.join(
        get_package_share_directory('sensor_integration_suite'),
        'launch'
    )
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/frame_broadcaster.launch.py'])
        # ),
        use_sim_time_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([sensor_integration_dir, '/dual_lidar.launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/zed_camera.launch.py']),
            launch_arguments={'camera_model': camera_model_arg,
                              'use_sim_time':LaunchConfiguration('use_sim_time')}.items()
        ),
        Node(
            package='sensor_integration_suite',
            executable='pointcloud_fusion_node',  # Replace with your actual executable name
            name='pointcloud_fusion_node',
            parameters=[{'zed_cloud_topic_name': '/zed/zed_node/point_cloud/cloud_registered', 
                         'vertical_pc_topic_name': '/lidar/points_vt',
                         'horizontal_pc_topic_name':'/lidar/points_hz',
                         'use_sim_time':LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

    ])
