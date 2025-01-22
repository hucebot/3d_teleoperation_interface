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

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)

install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/3d_interface/3d_viewer.py
  
  scripts/tools/streamdeck.py
  scripts/tools/trajectory_bridge.py
  scripts/tools/screen_recording.py
  scripts/tools/dummy_trajectory.py

  scripts/teleoperation/network_window.py
  scripts/teleoperation/robot_window.py
  scripts/teleoperation/stream_window.py
  scripts/teleoperation/insta_360_controller.py

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