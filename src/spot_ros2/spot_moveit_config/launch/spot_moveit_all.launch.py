import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # As configurações iniciais continuam as mesmas
    sim = LaunchConfiguration("sim").perform(context).lower() in ("true", "1", "yes")
    cfg_file = LaunchConfiguration("config_file").perform(context)
    spot_name = ""

    # --- Fase 1: Iniciar o back-end (driver/mock) primeiro ---
    # Esta ação será iniciada imediatamente
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("spot_ros2_control"),
                "launch",
                "spot_ros2_control_with_arm.launch.py",
            ])
        ),
        launch_arguments={
            "hardware_interface": "mock" if sim else "robot",
            "auto_start": "true",
            "launch_rviz": "false",
            "spot_name": spot_name,
            "config_file": cfg_file,
        }.items(),
    )

    # --- Fase 2: Preparar as ações do MoveIt, que serão atrasadas ---
    # Primeiro, preparamos as configurações do MoveIt como antes
    moveit_cfg = (
        MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
        .robot_description(
            mappings={"arm": "true", "add_ros2_control_tag": "false"}
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    # Agora, criamos os nós do MoveIt
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_cfg.robot_description],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_cfg.to_dict()],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("spot_moveit_config"), "config", "moveit.rviz"])],
        parameters=[moveit_cfg.to_dict()],
    )
    
    # Timer para o refresh do RViz (continua importante)
    rviz_refresh_timer = TimerAction(
        period=7.0, # Aumentei um pouco pra dar tempo de tudo subir antes do refresh
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/rviz/motion_planning/update_start_state', 'std_srvs/srv/Empty', '{}'],
                output='screen',
            )
        ],
    )

    # --- O Pulo do Gato: Atrasar o início do MoveIt ---
    # Esta ação vai esperar 5 segundos APÓS o início do launch para executar
    # as ações que estão dentro dela, resolvendo a "race condition".
    delayed_moveit_launch = TimerAction(
        period=5.0,  # <-- Espere por 5 segundos. Ajuste se precisar de mais tempo.
        actions=[
            robot_state_publisher_node,
            move_group_node,
            rviz_node,
            rviz_refresh_timer, # Colocamos o refresh aqui também pra ele rodar depois que o rviz estiver no ar
        ]
    )

    # A lista final de ações a serem retornadas para o launch
    return [
        ros2_control_launch,
        delayed_moveit_launch,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "sim",
            default_value="true",
            description="true = ros2_control mock | false = Spot real via driver",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="YAML com credenciais/ganhos do Spot real (se sim=false)",
        ),
        OpaqueFunction(function=launch_setup),
    ])