#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'service_name',
            default_value='/finger_count_node/get_finger_count',
            description='Nome do serviço de contagem de dedos'
        ),
        DeclareLaunchArgument(
            'check_interval',
            default_value='0.5',
            description='Intervalo de verificação (segundos)'
        ),
        DeclareLaunchArgument(
            'publish_topic',
            default_value='gesture_command',
            description='Tópico de publicação de comandos de gesto'
        ),
        LogInfo(msg='Iniciando finger_count_client...'),
        Node(
            package='spot_operation_gesture',
            executable='finger_count_client',
            name='finger_count_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'service_name': LaunchConfiguration('service_name'),
                'check_interval': LaunchConfiguration('check_interval'),
                'publish_topic': LaunchConfiguration('publish_topic'),
            }]
        )
    ]) 