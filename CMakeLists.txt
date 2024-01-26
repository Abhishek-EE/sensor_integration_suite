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
find_package(ZED 4 REQUIRED) # Replace '3' with your ZED SDK version
find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)


include_directories(
  include
  ${ZED_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

# include(FetchContent)

# FetchContent_Declare(
#   yaml-cpp
#   GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
#   GIT_TAG <tag_name> # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
# )
# FetchContent_GetProperties(yaml-cpp)

# if(NOT yaml-cpp_POPULATED)
#   message(STATUS "Fetching yaml-cpp...")
#   FetchContent_Populate(yaml-cpp)
#   add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
# endif()

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

add_executable(zed_mini_pointcloud_publisher_node src/zed_mini_pointcloud_publisher_node.cpp)
ament_target_dependencies(zed_mini_pointcloud_publisher_node
  rclcpp
  sensor_msgs
  ZED
  CUDA
)
target_link_libraries(zed_mini_pointcloud_publisher_node ${PROJECT_NAME} ${ZED_LIBRARIES} ${CUDA_LIBRARIES})

add_executable(lidar_test_node src/lidartest.cpp)
ament_target_dependencies(lidar_test_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(lidar_test_node ${PROJECT_NAME})

add_executable(frame_broadcaster_node src/frame_broadcaster.cpp)
ament_target_dependencies(frame_broadcaster_node
  rclcpp
  tf2_ros
  tf2
  geometry_msgs
)
target_link_libraries(frame_broadcaster_node ${PROJECT_NAME})

install(TARGETS
  lidar_publisher_node
  lidar_test_node
  zed_mini_pointcloud_publisher_node
  frame_broadcaster_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
