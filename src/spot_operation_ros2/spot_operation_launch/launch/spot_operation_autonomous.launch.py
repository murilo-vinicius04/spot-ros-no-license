#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        # Argumentos específicos para modo autônomo
        DeclareLaunchArgument(
            'enable_gestures',
            default_value='true',
            description='Habilitar gestos no modo autônomo'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.7',
            description='Limiar de confiança para detecção autônoma'
        ),
        
        # Log inicial
        LogInfo(msg='🤖 Iniciando Spot Operation System - Modo Autônomo...'),
        
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
        
        # Visão (essencial para autonomia)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_vision'),
                    'launch',
                    'vision.launch.py'
                ])
            ]),
            launch_arguments={
                'confidence_threshold': LaunchConfiguration('confidence_threshold')
            }.items()
        ),
        
        # Grasp (essencial para autonomia)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_grasp'),
                    'launch',
                    'grasp.launch.py'
                ])
            ])
        ),
        
        # Controle (adaptado para autonomia)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_control'),
                    'launch',
                    'control.launch.py'
                ])
            ])
        ),
        
        # Gestos (opcional, para override manual)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_gesture'),
                    'launch',
                    'gesture.launch.py'
                ])
            ]),
            condition=LaunchConfiguration('enable_gestures')
        ),
        
        # RViz para monitoramento
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('spot_operation_launch'),
                'config',
                'spot_operation_autonomous.rviz'
            ])]
        ),
        
        # Log final
        LogInfo(msg='✅ Modo Autônomo iniciado! O robô detectará e manipulará objetos automaticamente.'),
    ]) 