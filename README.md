# 3d_teleoperation_interface
ROS2 package to visualize robot information

ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true

ros2 run ros2_3d_interface 3d_interface
