#!/bin/bash

docker exec -it gazebo_container bash -c "
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
cd /ros_ws
exec bash
"