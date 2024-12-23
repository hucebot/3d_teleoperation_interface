FROM nvidia/opengl:1.2-glvnd-devel-ubuntu22.04
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
    tqdm \
    urdfpy

RUN pip install --upgrade numpy==1.23.5

###### Install ROS2
RUN apt-get install -y \
    curl \
    gnupg2 \
    lsb-release

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade
RUN apt install ros-humble-desktop-full -y
RUN apt install ros-dev-tools -y
RUN rosdep init && rosdep update

###### Install ROS Dependencies
RUN apt install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-imu-plugin \
    ros-${ROS_DISTRO}-librealsense2*

####### Install ROS2 Realsense
RUN apt install -y git
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src
RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
WORKDIR /ros2_ws
RUN rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} --skip-keys=librealsense2 -y

RUN apt install -y libgflags-dev nlohmann-json3-dev  \
    ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-image-publisher ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-diagnostic-updater ros-${ROS_DISTRO}-diagnostic-msgs ros-${ROS_DISTRO}-statistics-msgs \
    ros-${ROS_DISTRO}-backward-ros libdw-dev


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
    python3-rosdep \
    gedit \
    git

###### Install TORCH
RUN pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

###### Source ROS2
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
