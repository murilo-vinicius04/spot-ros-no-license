#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        # Argumentos globais
        DeclareLaunchArgument(
            'launch_core',
            default_value='true',
            description='Lançar módulo core'
        ),
        DeclareLaunchArgument(
            'launch_vision',
            default_value='true',
            description='Lançar módulo de visão'
        ),
        DeclareLaunchArgument(
            'launch_grasp',
            default_value='true',
            description='Lançar módulo de grasp'
        ),
        DeclareLaunchArgument(
            'launch_control',
            default_value='true',
            description='Lançar módulo de controle'
        ),
        DeclareLaunchArgument(
            'launch_gesture',
            default_value='true',
            description='Lançar módulo de gestos'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Lançar RViz'
        ),
        
        # Log inicial
        LogInfo(msg='🚀 Iniciando Spot Operation System completo...'),
        
        # Core (sempre lançado primeiro)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_core'),
                    'launch',
                    'core.launch.py'
                ])
            ]),
            condition=LaunchConfiguration('launch_core')
        ),
        
        # Visão
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_vision'),
                    'launch',
                    'vision.launch.py'
                ])
            ]),
            condition=LaunchConfiguration('launch_vision')
        ),
        
        # Grasp
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_grasp'),
                    'launch',
                    'grasp.launch.py'
                ])
            ]),
            condition=LaunchConfiguration('launch_grasp')
        ),
        
        # Controle
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_control'),
                    'launch',
                    'control.launch.py'
                ])
            ]),
            condition=LaunchConfiguration('launch_control')
        ),
        
        # Gestos
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_gesture'),
                    'launch',
                    'gesture.launch.py'
                ])
            ]),
            condition=LaunchConfiguration('launch_gesture')
        ),
        
        # RViz (opcional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('spot_operation_launch'),
                'config',
                'spot_operation.rviz'
            ])],
            condition=LaunchConfiguration('launch_rviz')
        ),
        
        # Log final
        LogInfo(msg='✅ Spot Operation System iniciado com sucesso!'),
    ]) 