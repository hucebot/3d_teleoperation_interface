
isRunning=`docker ps -f name=ros2_3d_interface | grep -c "ros2_3d_interface"`;

if [ $isRunning -eq 0 ]; then
    xhost +local:docker
    docker rm ros2_3d_interface
    docker run  \
        --name ros2_3d_interface  \
        --gpus all \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --env QT_X11_NO_MITSHM=1 \
        --net host \
        --ipc host \
        --pid host \
        --privileged \
        -it \
        -v /dev:/dev \
        -v /run/udev:/run/udev \
        --device /dev/dri \
        --device /dev/snd \
        --device /dev/input \
        --device /dev/bus/usb \
        -v `pwd`/../:/ros2_ws/src/ros2_3d_interface \
        -w /ros2_ws \
        ros2_3d_interface:latest

else
    echo "Docker already running."
    docker exec -it ros2_3d_interface /bin/bash
fi