#!/bin/bash

# Script para executar o ZED ROS 2 wrapper
# Uso: ./run_zed.sh [zed2|zed2i|zedm|zedx]

ZED_MODEL=${1:-zed2}

echo "🚀 Iniciando ZED ROS 2 wrapper para modelo: $ZED_MODEL"

# Verifica se o container está rodando
if ! docker ps | grep -q "zed-container"; then
    echo "❌ Container ZED não está rodando. Inicie primeiro com: docker-compose up zed"
    exit 1
fi

# Executa o launch do ZED
docker exec -it zed-container bash -c "
    source /opt/ros/humble/setup.bash &&
    source /root/zed_workspace/install/setup.bash &&
    ros2 launch zed_wrapper ${ZED_MODEL}.launch.py
"
