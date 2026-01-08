# # Base image
# FROM ros:humble-ros-base

# # Set working directory
# WORKDIR /ros2_video_ws

# # System dependencies
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     libunwind8 \
#     ros-humble-cv-bridge \
#     ros-humble-sensor-msgs \
#     python3-pip \
#     python3-colcon-common-extensions \
#     python3-opencv \
#     python3-gi \
#     python3-gst-1.0 \
#     gstreamer1.0-plugins-base \
#     gstreamer1.0-plugins-good \
#     gstreamer1.0-plugins-bad \
#     gstreamer1.0-plugins-ugly \
#     gstreamer1.0-libav \
#     gstreamer1.0-tools \
#     gstreamer1.0-nice \
#     libgstreamer1.0-dev \
#     libgstreamer-plugins-base1.0-dev \
#     libgstreamer-plugins-bad1.0-dev \
#     && rm -rf /var/lib/apt/lists/*

# # Preload libunwind and OpenCV RTSP options
# ENV LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libunwind.so.8
# ENV OPENCV_FFMPEG_CAPTURE_OPTIONS="rtsp_transport;tcp|buffer_size;10485760|stimeout;5000000|fflags;discardcorrupt"

# # Copy workspace and build
# COPY . .
# RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# # Source ROS2 workspace
# RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# RUN echo "source /ros2_video_ws/install/setup.bash" >> ~/.bashrc

# # Entrypoint for ROS2 commands
# COPY docker/entrypoint.sh /entrypoint.sh
# RUN chmod +x /entrypoint.sh
# ENTRYPOINT ["/entrypoint.sh"]
# CMD ["bash"]
# -----------------------------
# Dockerfile for ROS2 + GStreamer + MediaMTX pipeline
# -----------------------------

# Base ROS2 Humble image
# FROM ros:humble-ros-base

# # Set working directory
# WORKDIR /ros2_video_ws

# # -----------------------------
# # Install system dependencies
# # -----------------------------
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     libunwind8 \
#     ros-humble-cv-bridge \
#     ros-humble-sensor-msgs \
#     python3-pip \
#     python3-colcon-common-extensions \
#     python3-opencv \
#     python3-gi \
#     python3-gst-1.0 \
#     gstreamer1.0-plugins-base \
#     gstreamer1.0-plugins-good \
#     gstreamer1.0-plugins-bad \
#     gstreamer1.0-plugins-ugly \
#     gstreamer1.0-libav \
#     gstreamer1.0-tools \
#     gstreamer1.0-nice \
#     libgstreamer1.0-dev \
#     libgstreamer-plugins-base1.0-dev \
#     libgstreamer-plugins-bad1.0-dev \
#     ffmpeg \
#     && rm -rf /var/lib/apt/lists/*

# # -----------------------------
# # Preload libunwind (fix OpenCV+GStreamer crash)
# # -----------------------------
# ENV LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libunwind.so.8

# # -----------------------------
# # OpenCV FFMPEG RTSP options
# # -----------------------------
# ENV OPENCV_FFMPEG_CAPTURE_OPTIONS="rtsp_transport;tcp|buffer_size;10485760|stimeout;5000000|fflags;discardcorrupt"

# # -----------------------------
# # Copy ROS2 workspace and build
# # -----------------------------
# COPY . .
# RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# # -----------------------------
# # Source ROS2 workspace automatically
# # -----------------------------
# RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# RUN echo "source /ros2_video_ws/install/setup.bash" >> ~/.bashrc

# # -----------------------------
# # Entrypoint script
# # -----------------------------
# COPY docker/entrypoint.sh /entrypoint.sh
# RUN chmod +x /entrypoint.sh
# ENTRYPOINT ["/entrypoint.sh"]

# # Default command
# CMD ["bash"]
FROM ros:humble-ros-base

# Set working directory
WORKDIR /ros2_video_ws

# -----------------------------
# Install system dependencies
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    libunwind8 \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    python3-pip \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-gi \
    python3-gst-1.0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-nice \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Preload libunwind (fix OpenCV+GStreamer crash)
# -----------------------------
ENV LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libunwind.so.8
ENV YOLO_CONFIG_DIR=/tmp/Ultralytics


# -----------------------------
# OpenCV FFMPEG RTSP options
# -----------------------------
ENV OPENCV_FFMPEG_CAPTURE_OPTIONS="rtsp_transport;tcp|buffer_size;10485760|stimeout;5000000|fflags;discardcorrupt"

# -----------------------------
# Copy ROS2 workspace
# -----------------------------
COPY . .

# -----------------------------
# Install Python dependencies
# -----------------------------
RUN pip3 install --upgrade pip
RUN pip3 install 'numpy<2' torch torchvision ultralytics opencv-python

# -----------------------------
# Build ROS2 workspace
# -----------------------------
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# -----------------------------
# Source ROS2 workspace automatically
# -----------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_video_ws/install/setup.bash" >> ~/.bashrc

# -----------------------------
# Entrypoint script
# -----------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Default command: run bash if needed
CMD ["bash"]
