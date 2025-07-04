# spot_moveit_all.launch.py  (coloque em spot_moveit_config/launch ou onde preferir)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    ld = LaunchDescription()

    # -------- argumentos ----------
    sim = LaunchConfiguration("sim").perform(context).lower() in ("true", "1", "yes")
    cfg_file = LaunchConfiguration("config_file").perform(context)

    spot_name = ""  # se quiser namespace, troque aqui --------------------------------

    # ----------- (A) back-end low-level -------------
    if sim:
        # ros2_control em modo mock
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("spot_ros2_control"),
                            "launch",
                            "spot_ros2_control_with_arm.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "hardware_interface": "mock",
                    "auto_start": "true",
                    "launch_rviz": "false",
                    "spot_name": spot_name,
                }.items(),
            )
        )
    else:
        # driver real da Boston Dynamics (control + feeds)
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("spot_driver"), "launch", "spot_driver.launch.py"]
                    )
                ),
                launch_arguments={
                    "controllable": "true",
                    "launch_rviz": "false",
                    "arm": "true",
                    "config_file": cfg_file,
                    "mock_enable": "false",          # garante que não vai pro mock
                    "mock_has_arm": "false",
                    # passe também spot_name se o driver aceitar
                }.items(),
                condition=UnlessCondition("{}".format(sim)),  # redundante, mas didático
            )
        )

    # ----------- (B) MoveIt & RViz ------------------
    moveit_cfg = (
        MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory("spot_moveit_config"),
                "config",
                "moveit_controllers.yaml",
            )
        )
        .robot_description(
            mappings={  # evita duplicar ros2_control tags (já estão no URDF do driver)
                "arm": "true",
                "add_ros2_control_tag": "false",
            }
        )
        .to_moveit_configs()
    )

    # robot_state_publisher  (mantemos – no real ele só replica /joint_states)
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_cfg.robot_description],
        )
    )

    # Move Group (com parâmetros explícitos p/ controlador)
    ld.add_action(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_cfg.robot_description,
                moveit_cfg.robot_description_semantic,
                moveit_cfg.robot_description_kinematics,
                moveit_cfg.joint_limits,
                moveit_cfg.planning_pipelines,
                {
                    # “cola” a config do simple_controller_manager direta em parâmetros
                    "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                    "moveit_simple_controller_manager.controller_names": ["arm_controller"],
                    "moveit_simple_controller_manager.arm_controller.type": "FollowJointTrajectory",
                    "moveit_simple_controller_manager.arm_controller.action_ns": "follow_joint_trajectory",
                    "moveit_simple_controller_manager.arm_controller.default": True,
                    "moveit_simple_controller_manager.arm_controller.joints": [
                        "arm_sh0",
                        "arm_sh1",
                        "arm_el0",
                        "arm_el1",
                        "arm_wr0",
                        "arm_wr1",
                    ],
                },
            ],
        )
    )

    # RViz (opcional — pode desligar com launch arg depois se quiser)
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=[
                "-d",
                os.path.join(
                    get_package_share_directory("spot_moveit_config"),
                    "config",
                    "moveit.rviz",
                )
            ],
            parameters=[
                moveit_cfg.robot_description,
                moveit_cfg.robot_description_semantic,
                moveit_cfg.robot_description_kinematics,
                moveit_cfg.joint_limits,
                moveit_cfg.planning_pipelines,
                moveit_cfg.trajectory_execution,
            ],
        )
    )

    return ld


def generate_launch_description():
    return LaunchDescription(
        [
            # argumento master-switch
            DeclareLaunchArgument(
                "sim",
                default_value="true",
                description="true = ros2_control mock  |  false = Spot real via driver",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="",
                description="YAML com credenciais/ganhos do Spot real (necessário se sim=false)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
