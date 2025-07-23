#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'group_name',
            default_value='manipulator',
            description='Nome do grupo MoveIt'
        ),
        DeclareLaunchArgument(
            'end_effector_link',
            default_value='arm_link_fngr',
            description='Link do end-effector'
        ),
        DeclareLaunchArgument(
            'control_rate',
            default_value='5.0',
            description='Taxa de controle (Hz)'
        ),
        DeclareLaunchArgument(
            'pf_k_att',
            default_value='0.2',
            description='Ganho de atração do campo de potencial'
        ),
        DeclareLaunchArgument(
            'pf_attraction_distance',
            default_value='0.4',
            description='Distância de atração do campo de potencial (metros)'
        ),
        DeclareLaunchArgument(
            'escape_distance',
            default_value='0.2',
            description='Distância de escape (metros)'
        ),
        LogInfo(msg='Iniciando spot_controller...'),
        Node(
            package='spot_operation_control',
            executable='spot_controller',
            name='spot_controller',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'group_name': LaunchConfiguration('group_name'),
                'end_effector_link': LaunchConfiguration('end_effector_link'),
                'control_rate': LaunchConfiguration('control_rate'),
                'pf_k_att': LaunchConfiguration('pf_k_att'),
                'pf_attraction_distance': LaunchConfiguration('pf_attraction_distance'),
                'escape_distance': LaunchConfiguration('escape_distance'),
            }]
        )
    ]) 