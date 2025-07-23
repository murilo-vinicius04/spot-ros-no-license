#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('spot_operation_vision'), 'models', 'yolo11n.pt'
            ]),
            description='Caminho para o modelo YOLO'
        ),
        DeclareLaunchArgument(
            'allowed_objects_csv',
            default_value=PathJoinSubstitution([
                FindPackageShare('spot_operation_vision'), 'config', 'allowed_objects.csv'
            ]),
            description='CSV de objetos permitidos'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw',
            description='Tópico de imagem de entrada'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.3',
            description='Limiar de confiança para detecção'
        ),
        DeclareLaunchArgument(
            'publish_topic',
            default_value='object_detections',
            description='Tópico de publicação das detecções'
        ),
        LogInfo(msg='Iniciando object_detector...'),
        Node(
            package='spot_operation_vision',
            executable='object_detector',
            name='object_detector',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'allowed_objects_csv': LaunchConfiguration('allowed_objects_csv'),
                'image_topic': LaunchConfiguration('image_topic'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'publish_topic': LaunchConfiguration('publish_topic'),
            }]
        )
    ]) 