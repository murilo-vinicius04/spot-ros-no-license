#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'default_strategy',
            default_value='yolo_grasp',
            description='Estratégia de grasp padrão'
        ),
        DeclareLaunchArgument(
            'fallback_strategy',
            default_value='manual_grasp',
            description='Estratégia de grasp de fallback'
        ),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='object_detections_3d',
            description='Tópico de detecções 3D'
        ),
        DeclareLaunchArgument(
            'results_topic',
            default_value='grasp_results',
            description='Tópico de resultados de grasp'
        ),
        LogInfo(msg='Iniciando grasp_manager...'),
        Node(
            package='spot_operation_grasp',
            executable='grasp_manager',
            name='grasp_manager',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'default_strategy': LaunchConfiguration('default_strategy'),
                'fallback_strategy': LaunchConfiguration('fallback_strategy'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'results_topic': LaunchConfiguration('results_topic'),
            }]
        )
    ]) 