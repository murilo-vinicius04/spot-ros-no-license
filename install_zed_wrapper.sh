#!/bin/bash

echo "🚀 Iniciando configuração do ZED ROS 2 wrapper..."
source /opt/ros/humble/setup.bash

mkdir -p /root/zed_workspace/src
cd /root/zed_workspace/src

if [ ! -d "zed-ros2-wrapper" ]; then
    echo "📦 Clonando ZED ROS 2 wrapper (versão compatível com ZED SDK v4.2)..."
    git clone --recursive -b humble-v4.2.x https://github.com/stereolabs/zed-ros2-wrapper.git
fi

if [ ! -d "zed-ros2-interfaces" ]; then
    echo "📦 Clonando ZED ROS 2 interfaces..."
    git clone https://github.com/stereolabs/zed-ros2-interfaces.git
fi

cd /root/zed_workspace

echo "🔧 Instalando dependências..."
apt-get update
apt-get install -y ros-humble-xacro ros-humble-robot-localization
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "🏗️ Compilando ZED ROS 2 wrapper..."
colcon build --parallel-workers $(nproc) --symlink-install

echo "✅ ZED ROS 2 wrapper instalado com sucesso!"
echo "💡 Para usar: source /root/zed_workspace/install/setup.bash"
echo "🎯 Container ZED pronto! Use: ros2 launch zed_wrapper zed2.launch.py"

exec bash
