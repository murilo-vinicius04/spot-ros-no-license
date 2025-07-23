#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        # Argumentos específicos para modo manual
        DeclareLaunchArgument(
            'enable_vision',
            default_value='false',
            description='Habilitar visão no modo manual'
        ),
        DeclareLaunchArgument(
            'enable_grasp',
            default_value='false',
            description='Habilitar grasp no modo manual'
        ),
        
        # Log inicial
        LogInfo(msg='🎮 Iniciando Spot Operation System - Modo Manual...'),
        
        # Core (sempre necessário)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_core'),
                    'launch',
                    'core.launch.py'
                ])
            ])
        ),
        
        # Controle (principal no modo manual)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_control'),
                    'launch',
                    'control.launch.py'
                ])
            ])
        ),
        
        # Gestos (essencial para controle manual)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_gesture'),
                    'launch',
                    'gesture.launch.py'
                ])
            ])
        ),
        
        # Visão (opcional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_vision'),
                    'launch',
                    'vision.launch.py'
                ])
            ]),
            condition=LaunchConfiguration('enable_vision')
        ),
        
        # Grasp (opcional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_grasp'),
                    'launch',
                    'grasp.launch.py'
                ])
            ]),
            condition=LaunchConfiguration('enable_grasp')
        ),
        
        # RViz para visualização
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('spot_operation_launch'),
                'config',
                'spot_operation_manual.rviz'
            ])]
        ),
        
        # Log final
        LogInfo(msg='✅ Modo Manual iniciado! Use gestos para controlar o robô.'),
    ]) 