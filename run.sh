#!/bin/bash

xhost +local:docker

docker run --rm -it \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env TURTLEBOT3_MODEL=burger \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)/ros_ws:/ros_ws" \
    --name gazebo_container \
    turtlebot3_gazebo_robotica26