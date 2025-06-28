#!/usr/bin/env python3

# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from synchros2.launch.actions import DeclareBooleanLaunchArgument


def generate_launch_description():
    """
    Launch file para testar posições personalizadas do Spot
    - Driver com rviz habilitado
    - ros2_control sem rviz 
    - Controlador de posição personalizada
    """
    
    # Declarar argumentos de launch
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Arquivo de configuração do robô'
    )
    
    spot_name_arg = DeclareLaunchArgument(
        'spot_name',
        default_value='',
        description='Nome do robô Spot'
    )
    
    custom_position_arg = DeclareLaunchArgument(
        'custom_position',
        default_value='[0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6]',
        description='Posição personalizada das juntas (12 valores em radianos)'
    )
    
    transition_duration_arg = DeclareLaunchArgument(
        'transition_duration',
        default_value='4.0',
        description='Duração da transição para a posição alvo (segundos)'
    )
    
    auto_start_controllers_arg = DeclareBooleanLaunchArgument(
        'auto_start_controllers',
        default_value=True,
        description='Iniciar automaticamente os controladores'
    )
    
    # Launch do driver do Spot com rviz habilitado
    spot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('spot_driver'),
                'launch',
                'spot_driver.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
            'spot_name': LaunchConfiguration('spot_name'),
            'launch_rviz': 'True',  # RViz habilitado APENAS no driver
            'controllable': 'True',
            'launch_image_publishers': 'False',
        }.items()
    )
    
    # Nó do controlador de posição personalizada
    custom_position_controller = Node(
        package='spot_ros2_control',
        executable='custom_position_controller.py',
        name='custom_position_controller',
        output='screen',
        parameters=[{
            'spot_name': LaunchConfiguration('spot_name'),
            'custom_position': LaunchConfiguration('custom_position'),
            'transition_duration': LaunchConfiguration('transition_duration'),
            'command_rate': 50.0,
        }],
        namespace=LaunchConfiguration('spot_name')
    )
    
    # Nó para ativar o controlador automaticamente se solicitado
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c', 'controller_manager',
            'forward_position_controller',
            '--activate'
        ],
        namespace=LaunchConfiguration('spot_name'),
        condition=IfCondition(LaunchConfiguration('auto_start_controllers'))
    )
    
    return LaunchDescription([
        # Argumentos
        config_file_arg,
        spot_name_arg,
        custom_position_arg,
        transition_duration_arg,
        auto_start_controllers_arg,
        
        # Nodes
        spot_driver_launch,
        custom_position_controller,
        controller_spawner,
    ]) 