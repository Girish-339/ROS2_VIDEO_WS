#!/bin/bash
set -e

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source /ros2_video_ws/install/setup.bash

# Start nodes in background
ros2 run video_pub_sub camera_publisher &       # Camera node
ros2 run video_pub_sub ros2_to_gstreamer &      # GStreamer bridge
ros2 run yolo_detector yolo_node &             # YOLO detector

# Wait forever so container stays alive
wait
