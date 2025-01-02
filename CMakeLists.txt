cmake_minimum_required(VERSION 3.8)
project(ros2_3d_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)


add_executable(cloud_separation src/cloud_separation.cpp)
add_executable(publish_trajectory src/trajectory.cpp)
add_executable(verify_line_of_sight src/verify_line_of_sight.cpp)

ament_target_dependencies(publish_trajectory rclcpp visualization_msgs)

ament_target_dependencies(
  cloud_separation
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  visualization_msgs
)

ament_target_dependencies(
  verify_line_of_sight
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  visualization_msgs
)

install(TARGETS
  cloud_separation
  publish_trajectory
  verify_line_of_sight
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/3d_real_time.py
  scripts/line_of_sight.py
  scripts/trajectory.py
  ros2_3d_interface/utilities/camera.py
  ros2_3d_interface/utilities/shape.py
  ros2_3d_interface/utilities/utils.py
  ros2_3d_interface/utilities/viewer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  models
  ros2_3d_interface
  DESTINATION share/${PROJECT_NAME}
)

ament_package()