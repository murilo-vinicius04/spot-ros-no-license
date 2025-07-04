# Copyright (c) 2024-2025 Boston Dynamics AI Institute LLC. All rights reserved.
import os
from tempfile import NamedTemporaryFile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from synchros2.launch.actions import DeclareBooleanLaunchArgument

from spot_common.launch.spot_launch_helpers import (
    IMAGE_PUBLISHER_ARGS,
    declare_image_publisher_args,
    get_login_parameters,
    get_ros_param_dict,
    spot_has_arm,
)

THIS_PACKAGE = "spot_ros2_control"


def create_rviz_config_with_arm(spot_name: str) -> str:
    """Creates RViz config for Spot with arm visualization and control panel"""
    
    template_filename = os.path.join(get_package_share_directory(THIS_PACKAGE), "rviz", "template.rviz")
    
    if spot_name:
        with open(template_filename, "r") as template_file:
            config = yaml.safe_load(template_file)
            # replace fixed frame with robot body frame
            config["Visualization Manager"]["Global Options"]["Fixed Frame"] = f"{spot_name}/body"
            # Add robot models for each robot
            for display in config["Visualization Manager"]["Displays"]:
                if "RobotModel" in display["Class"]:
                    display["Description Topic"]["Value"] = f"/{spot_name}/robot_description"
                    
        # Add arm control panels
        control_panel = {
            "Class": "rviz_common/Tool Properties",
            "Expanded": True,
            "Name": "Spot Arm Control",
            "Property Tree Widget": {
                "Expanded": ["/Joint Command Publisher"],
                "Splitter Ratio": 0.5
            }
        }
        
        # Add joint state display for better visualization
        joint_state_display = {
            "Alpha": 1,
            "Class": "rviz_default_plugins/TF",
            "Enabled": True,
            "Frame Timeout": 15,
            "Marker Scale": 0.3,
            "Name": "Arm TF",
            "Show Arrows": True,
            "Show Axes": True,
            "Show Names": True,
            "Update Interval": 0,
            "Value": True
        }
        
        config["Panels"].append(control_panel)
        config["Visualization Manager"]["Displays"].append(joint_state_display)
        
        with NamedTemporaryFile(suffix=".rviz", mode="w", delete=False) as out_file:
            yaml.dump(config, out_file)
            return out_file.name
    else:
        return template_filename


def launch_setup(context: LaunchContext, ld: LaunchDescription) -> None:
    hardware_interface: str = LaunchConfiguration("hardware_interface").perform(context)
    controllers_config: str = LaunchConfiguration("controllers_config").perform(context)
    spot_name: str = LaunchConfiguration("spot_name").perform(context)
    config_file: str = LaunchConfiguration("config_file").perform(context)

    # Force arm to be True for this launch file
    arm = True
    login_params = ""
    gain_params = ""

    # If running on robot, query login parameters and gains
    if hardware_interface == "robot":
        username, password, hostname, port, certificate, _ = get_login_parameters(config_file)
        login_params = f" hostname:={hostname} username:={username} password:={password}"
        if port is not None:
            login_params += f" port:={port}"
        if certificate is not None:
            login_params += f" certificate:={certificate}"
        param_dict = get_ros_param_dict(config_file)
        if "k_q_p" in param_dict:
            k_q_p = " ".join(map(str, param_dict["k_q_p"]))
            gain_params += f' k_q_p:="{k_q_p}" '
        if "k_qd_p" in param_dict:
            k_qd_p = " ".join(map(str, param_dict["k_qd_p"]))
            gain_params += f' k_qd_p:="{k_qd_p}" '

    tf_prefix = f"{spot_name}/" if spot_name else ""

    # Generate the robot description with ARM ENABLED
    robot_urdf = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("spot_description"), "urdf", "spot.urdf.xacro"]),
            " add_ros2_control_tag:=True arm:=True",  # Force arm=True
            " tf_prefix:=",
            tf_prefix,
            " hardware_interface_type:=",
            LaunchConfiguration("hardware_interface"),
            " leasing:=",
            LaunchConfiguration("leasing_mode"),
            login_params,
            gain_params,
        ]
    )
    robot_description = {"robot_description": robot_urdf}

    # Use arm controller config
    if controllers_config == "":
        controllers_config = os.path.join(
            get_package_share_directory(THIS_PACKAGE), "config", "spot_default_controllers_with_arm.yaml"
        )

    # Add nodes
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[robot_description, controllers_config],
            namespace=spot_name,
        )
    )

    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"ignore_timestamp": True}],
            namespace=spot_name,
            condition=UnlessCondition(LaunchConfiguration("control_only")),
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="hardware_spawner",
            arguments=["-c", "controller_manager", "--activate", "SpotSystem"],
            namespace=spot_name,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "-c",
                        "controller_manager",
                        "joint_state_broadcaster",
                        "imu_sensor_broadcaster", 
                        "foot_state_broadcaster",
                        "spot_pose_broadcaster",
                        "forward_position_controller",
                        "arm_controller",
                        "gripper_controller",
                    ],
                    namespace=spot_name,
                )
            ],
            condition=IfCondition(LaunchConfiguration("auto_start")),
        )
    )

    # Launch RViz with arm configuration
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", create_rviz_config_with_arm(spot_name)],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
            namespace=spot_name,
        )
    )

    # Launch joint state publisher GUI for manual control
    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_gui")),
            namespace=spot_name,
            remappings=[("joint_states", f"/{tf_prefix}joint_command")],
        )
    )

    # Add rqt joint trajectory controller for more advanced control
    ld.add_action(
        Node(
            package="rqt_joint_trajectory_controller",
            executable="rqt_joint_trajectory_controller",
            name="rqt_joint_trajectory_controller",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_rqt")),
            namespace=spot_name,
        )
    )

    # Finally, launch extra nodes for state and image publishing if we are running on a robot.
    if hardware_interface == "robot":
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("spot_driver"), "launch", "spot_image_publishers.launch.py"]
                        )
                    ]
                ),
                launch_arguments={
                    key: LaunchConfiguration(key) for key in ["config_file", "spot_name"] + IMAGE_PUBLISHER_ARGS
                }.items(),
                condition=IfCondition(LaunchConfiguration("launch_image_publishers")),
            )
        )


def generate_launch_description():
    launch_args = []

    launch_args.append(
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Path to configuration file for the driver.",
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "controllers_config",
            default_value="",
            description="Configuration file for spot_ros2_control controllers.",
        ),
    )
    launch_args.append(
        DeclareLaunchArgument(
            "hardware_interface",
            default_value="mock",
            description="Hardware interface type: mock or robot",
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "leasing_mode",
            default_value="direct",
            description="Leasing mode for the robot",
        )
    )
    launch_args.append(
        DeclareBooleanLaunchArgument(
            "auto_start",
            default_value=True,
            description="Automatically start controllers",
        )
    )
    launch_args.append(
        DeclareBooleanLaunchArgument(
            "control_only",
            default_value=False,
            description="Start only control nodes without robot state publisher",
        )
    )
    launch_args.append(
        DeclareBooleanLaunchArgument(
            "launch_rviz",
            default_value=True,
            description="Launch RViz",
        )
    )
    launch_args.append(
        DeclareBooleanLaunchArgument(
            "launch_gui",
            default_value=True,
            description="Launch joint state publisher GUI for manual control",
        )
    )
    launch_args.append(
        DeclareBooleanLaunchArgument(
            "launch_rqt",
            default_value=False,
            description="Launch RQT joint trajectory controller",
        )
    )
    launch_args.append(
        DeclareBooleanLaunchArgument(
            "launch_image_publishers",
            default_value=False,
            description="Launch image publishers (only for real robot)",
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "spot_name",
            default_value="",
            description="Name of the Spot robot",
        )
    )

    ld = LaunchDescription(launch_args)
    ld.add_action(OpaqueFunction(function=launch_setup, args=[ld]))

    return ld 