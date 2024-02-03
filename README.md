# Sensor Integration Suite

The Sensor Integration Suite is a comprehensive ROS package designed for integrating and processing data from multiple sensors. It leverages data from LiDAR devices and ZED stereo cameras to provide a fused point cloud, which can be utilized for various applications such as 3D mapping, navigation, and object detection. It is specifically designed to read data from LD06 LIDAR

## Features

- Publishing LiDAR data as ROS PointCloud2 messages.
- Fusion of point clouds from ZED cameras and LiDAR sensors.
- Integration with ROS tf2 for managing transformations between frames.
- Compatibility with the `cartographer_ros` for SLAM (Simultaneous Localization and Mapping).

## Prerequisites

- ROS 2 (tested on Foxy)
- PCL (Point Cloud Library)
- CUDA (for GPU-accelerated processing)
- `zed_ros2_wrapper` package

## Installation

### Sensor Integration Suite

1. Clone the repository into your ROS workspace's `src` directory:

```bash
cd ~/ros_workspace/src
git clone git@github.com:Abhishek-EE/sensor_integration_suite.git
```
2. Navigate back to your ROS workspace and build the package
```bash
cd ~/ros_workspace
colcon build --packages-select sensor_integration_suite
```
3. The `zed_ros2_wrapper` is required for integrating the ZED stereo cameras. Follow the instructions provided by the package repository to install it.
```bash
cd ~/ros_workspace/src
git clone <zed-ros-wrapper-repo-url> zed_ros2_wrapper
cd ~/ros_workspace
colcon build --packages-select zed_ros2_wrapper
source install/setup.bash
```

## Usage
Ensure all necessary ROS packages are sourced before running any nodes. Then you can use the existing launch files to launch the system.

### Frame broadcasted by other directories
If all the required frames are being broadcasted independently of the system then you can simply launch the sensor_integration_suite.launch.py like this
```bash
ros2 launch sensor_integration_suite sensor_integration_suite.launch.py
```
### Frame not broadcasted by other systems
Here first you need to edit the config/frame.yaml to add all the desired frames and then launch the frame_broadcaster.launch.py
```bash
ros2 launch sensor_integration_suite frame_broadcaster.launch.py
```
And then launch the sensor_integration_suite.launch.py
```bash
ros2 launch sensor_integration_suite sensor_integration_suite.launch.py
```

## System Architecture 

The system architecture is designed to allow easy integration and synchronization between different sensor inputs. The main components are:

- `zed/zed_node`: Provides the ZED camera data streams including stereo images, depth data, and point clouds.
- `lidar_publisher_node_hz` and `lidar_publisher_node_vt`: Publish LiDAR data from horizontal and vertical LiDARs, respectively.
- `pointcloud_fusion_node`: Fuses point clouds from ZED and LiDAR sources into a single coherent point cloud.
- `cartographer_node`: Consumes the fused point cloud for SLAM to create a map and estimate the robot's pose.
- `tf`: The Transform Library in ROS, which keeps track of frame transformations and is critical for maintaining the spatial relations between different sensors.


Below is the topic diagram for cartographer integration
![Screenshot from 2024-02-01 14-58-05](https://github.com/Abhishek-EE/sensor_integration_suite/assets/59440480/1c9e76c7-9a55-4de2-b34b-0abc0846dd54)

## Contributing
Please follow the standard GitHub pull request process to make contributions to the Sensor Integration Suite.

## License
Please refer to the LICENSE file for information on the licensing of this package.

## Acknowledgments
This suite is the result of collaborative efforts. We would like to thank the maintainers of the zed_ros2_wrapper, cartographer_ros, and PCL for their valuable contributions to the ROS ecosystem.
