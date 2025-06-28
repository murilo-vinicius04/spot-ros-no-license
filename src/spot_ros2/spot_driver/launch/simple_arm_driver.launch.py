#!/usr/bin/env python3

# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

"""
Launch file para o driver simplificado do braço do Spot.

Este launch file inicia o driver que permite controlar apenas o braço
sem precisar da licença de baixo nível.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Gera a descrição do launch."""
    
    # Argumentos do launch
    declare_username_arg = DeclareLaunchArgument(
        'username',
        default_value=os.getenv('BOSDYN_CLIENT_USERNAME', 'user'),
        description='Username para autenticação no Spot'
    )
    
    declare_password_arg = DeclareLaunchArgument(
        'password', 
        default_value=os.getenv('BOSDYN_CLIENT_PASSWORD', 'password'),
        description='Password para autenticação no Spot'
    )
    
    declare_hostname_arg = DeclareLaunchArgument(
        'hostname',
        default_value=os.getenv('SPOT_IP', '10.0.0.3'),
        description='IP do Spot'
    )
    
    declare_port_arg = DeclareLaunchArgument(
        'port',
        default_value='0',
        description='Porta para conexão com o Spot (0 = padrão)'
    )
    
    declare_spot_name_arg = DeclareLaunchArgument(
        'spot_name',
        default_value='Spot',
        description='Nome do robô Spot'
    )
    
    declare_state_rate_arg = DeclareLaunchArgument(
        'state_rate',
        default_value='10.0',
        description='Taxa de publicação do estado das juntas (Hz)'
    )
    
    # Nó do driver simplificado do braço
    simple_arm_driver_node = Node(
        package='spot_driver',
        executable='simple_arm_driver',
        name='simple_arm_driver',
        output='screen',
        parameters=[{
            'username': LaunchConfiguration('username'),
            'password': LaunchConfiguration('password'),
            'hostname': LaunchConfiguration('hostname'),
            'port': LaunchConfiguration('port'),
            'spot_name': LaunchConfiguration('spot_name'),
            'state_rate': LaunchConfiguration('state_rate'),
        }]
    )
    
    return LaunchDescription([
        declare_username_arg,
        declare_password_arg,
        declare_hostname_arg,
        declare_port_arg,
        declare_spot_name_arg,
        declare_state_rate_arg,
        simple_arm_driver_node,
    ]) 