#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch file para o pacote core da operação do Spot."""
    
    # Argumentos de launch
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('spot_operation_core'),
            'config',
            'core_config.yaml'
        ]),
        description='Caminho para o arquivo de configuração'
    )
    
    # Nó do gerenciador de clientes do robô
    robot_client_manager_node = Node(
        package='spot_operation_core',
        executable='robot_client_manager',
        name='robot_client_manager',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True
    )
    
    # Nó do gerenciador de TF
    tf_manager_node = Node(
        package='spot_operation_core',
        executable='tf_manager',
        name='tf_manager',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True
    )
    
    # Nó do gerenciador de estado da operação
    operation_state_manager_node = Node(
        package='spot_operation_core',
        executable='operation_state_manager',
        name='operation_state_manager',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        LogInfo(msg="Iniciando pacote core da operação do Spot..."),
        robot_client_manager_node,
        tf_manager_node,
        operation_state_manager_node,
        LogInfo(msg="Pacote core da operação do Spot iniciado!")
    ]) 