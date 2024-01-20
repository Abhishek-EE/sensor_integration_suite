cmake_minimum_required(VERSION 3.5)
project(sensor_integration_suite)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/lidarkit.cpp
  src/util.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
)

# Declare C++ executable nodes
add_executable(lidar_publisher_node src/lidar_publisher_node.cpp)
ament_target_dependencies(lidar_publisher_node rclcpp sensor_msgs)
target_link_libraries(lidar_publisher_node ${PROJECT_NAME})

add_executable(lidar_test_node src/lidartest.cpp)
ament_target_dependencies(lidar_test_node rclcpp sensor_msgs)
target_link_libraries(lidar_test_node ${PROJECT_NAME})

install(TARGETS
  lidar_publisher_node
  lidar_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

