cmake_minimum_required(VERSION 3.5)
project(sensor_integration_suite)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-deprecated-copy -Wno-unused-parameter -Wno-ignored-qualifiers)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(CUDA REQUIRED)


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/lidarkit.cpp
  src/util.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  geometry_msgs
  PCL
)

# Declare C++ executable nodes
add_executable(lidar_publisher_node src/lidar_publisher_node.cpp)
ament_target_dependencies(lidar_publisher_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(lidar_publisher_node ${PROJECT_NAME})

#lidar_test_node
add_executable(lidar_test_node src/lidartest.cpp)
ament_target_dependencies(lidar_test_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(lidar_test_node ${PROJECT_NAME})

#pointcloud_fusion_node
add_executable(pointcloud_fusion_node src/pointcloud_fusion_node.cpp)
ament_target_dependencies(pointcloud_fusion_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  PCL
)
target_link_libraries(pointcloud_fusion_node ${PROJECT_NAME} ${PCL_LIBRARIES})


install(TARGETS
  lidar_publisher_node
  lidar_test_node
  pointcloud_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
