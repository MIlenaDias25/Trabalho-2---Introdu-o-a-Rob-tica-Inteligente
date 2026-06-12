#!/bin/bash

set -e

# Cores para o terminal
GREEN='\033[0;32m'
NC='\033[0m' # Sem cor

echo -e "${GREEN}### Iniciando Setup do Workspace ROS 2 Gazebo ###${NC}"

# 1. Clona os repositórios necessários dentro de src/
echo "Clonando pacotes..."
cd ros_ws/src/

echo "Clonando repositórios..."

if [ ! -d "turtlebot3" ]; then
    git clone -b humble \
    https://github.com/ROBOTIS-GIT/turtlebot3.git
else
    echo "turtlebot3 já existe."
fi

if [ ! -d "turtlebot3_msgs" ]; then
    git clone -b humble \
    https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
else
    echo "turtlebot3_msgs já existe."
fi

if [ ! -d "turtlebot3_simulations" ]; then
    git clone -b humble \
    https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
else
    echo "turtlebot3_simulations já existe."
fi

# Volta para 
cd ../..

cp launch/turtlebot3_dqn_stage5.launch.py \
ros_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/

cp world/turtlebot3_dqn_stage5.world \
ros_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/

echo -e "${GREEN}### Build da imagem Docker ###${NC}"

docker build \
    -t turtlebot3_gazebo_robotica26 \
    -f docker/Dockerfile .

echo -e "${GREEN}### Setup concluído ###${NC}"