FROM ros:humble
ENV ROS_DISTRO humble

ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="Europe/Paris"

LABEL maintainer="clemente.donoso@inria.fr"

RUN apt-get update && apt-get upgrade -y

###### Install Python Dependencies
RUN apt-get install -y \
    python-is-python3 \
    python3-gst-1.0 \
    python3-pip

RUN pip install\
    v4l2py \
    opencv-contrib-python \
    pyserial \
    scipy \
    pyside6 \
    pyglet \
    moderngl \
    moderngl-window \
    pyglm \
    glfw \
    pillow \
    pygame \
    pynput \
    pyrr \
    tqdm

RUN pip install --upgrade numpy==1.23.5

###### Install ROS Dependencies
RUN apt install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-imu-plugin \
    ros-${ROS_DISTRO}-librealsense2*

####### Install ROS2 Realsense
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src
RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
WORKDIR /ros2_ws
RUN rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y

###### Install Dependencies
RUN  apt install -y \
    gstreamer1.0-plugins-good \ 
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-rtp \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    net-tools \
    ntpdate \
    v4l-utils \
    terminator \
    libx11-xcb1 libxcb-util1 libxcb-render0 libxcb-shape0 \
    libxcb-xfixes0 libxcb-keysyms1 libxcb-image0 libxcb-randr0 \
    libxcb-xtest0 libxcb-cursor0 xvfb \
    python3-rosdep