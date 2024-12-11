
isRunning=`docker ps -f name=ros2_teleop_interface | grep -c "ros2_teleop_interface"`;

if [ $isRunning -eq 0 ]; then
    xhost +local:docker
    docker rm ros2_teleop_interface
    docker run  \
        --name ros2_teleop_interface  \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --env QT_X11_NO_MITSHM=1 \
        --net host \
        --ipc host \
        --pid host \
        --privileged \
        -it \
        -v /dev:/dev \
        -v `pwd`/../:/ros2_ws/src/teleoperation_interface \
        -w /ros2_ws \
        ros2_teleop_interface:latest

else
    echo "Docker already running."
    docker exec -it ros2_teleop_interface /bin/bash
fi