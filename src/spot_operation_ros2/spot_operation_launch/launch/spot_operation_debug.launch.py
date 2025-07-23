#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        # Argumentos para debug
        DeclareLaunchArgument(
            'enable_all_modules',
            default_value='true',
            description='Habilitar todos os módulos para debug'
        ),
        DeclareLaunchArgument(
            'debug_level',
            default_value='DEBUG',
            description='Nível de debug (DEBUG, INFO, WARN, ERROR)'
        ),
        DeclareLaunchArgument(
            'enable_rqt',
            default_value='true',
            description='Habilitar RQT para debug'
        ),
        
        # Log inicial
        LogInfo(msg='🔍 Iniciando Spot Operation System - Modo Debug...'),
        
        # Core
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_core'),
                    'launch',
                    'core.launch.py'
                ])
            ]),
            condition=LaunchConfiguration('enable_all_modules')
        ),
        
        # Visão com debug
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_vision'),
                    'launch',
                    'vision.launch.py'
                ])
            ]),
            launch_arguments={
                'debug_mode': 'true',
                'publish_debug_images': 'true'
            }.items(),
            condition=LaunchConfiguration('enable_all_modules')
        ),
        
        # Grasp com debug
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_grasp'),
                    'launch',
                    'grasp.launch.py'
                ])
            ]),
            launch_arguments={
                'debug_mode': 'true'
            }.items(),
            condition=LaunchConfiguration('enable_all_modules')
        ),
        
        # Controle com debug
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_control'),
                    'launch',
                    'control.launch.py'
                ])
            ]),
            launch_arguments={
                'debug_mode': 'true',
                'publish_debug_info': 'true'
            }.items(),
            condition=LaunchConfiguration('enable_all_modules')
        ),
        
        # Gestos com debug
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_operation_gesture'),
                    'launch',
                    'gesture.launch.py'
                ])
            ]),
            launch_arguments={
                'debug_mode': 'true'
            }.items(),
            condition=LaunchConfiguration('enable_all_modules')
        ),
        
        # RQT para debug
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
            arguments=['--perspective-file', PathJoinSubstitution([
                FindPackageShare('spot_operation_launch'),
                'config',
                'spot_operation_debug.perspective'
            ])],
            condition=LaunchConfiguration('enable_rqt')
        ),
        
        # RViz para debug
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('spot_operation_launch'),
                'config',
                'spot_operation_debug.rviz'
            ])]
        ),
        
        # PlotJuggler para análise de dados
        Node(
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler',
            arguments=['--layout', PathJoinSubstitution([
                FindPackageShare('spot_operation_launch'),
                'config',
                'spot_operation_debug.xml'
            ])],
            condition=LaunchConfiguration('enable_rqt')
        ),
        
        # Log final
        LogInfo(msg='✅ Modo Debug iniciado! Use RQT e RViz para análise detalhada.'),
    ]) 