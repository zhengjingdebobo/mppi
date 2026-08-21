"""Corridor omni launch with external ref publisher + local-input MPPI.

Starts:
    - ddr_minimal_sim corridor omni scenario
    - global_ref_path (publishes a global Path)
    - mppi_local (subscribes to that Path + /scan + /odom and publishes /cmd_vel)

This launch keeps the ROS inputs/outputs unchanged while running the installed
`exact_mppi` JAX controller through the ROS2 bridge package.

Usage:
    ros2 launch exact_mppi_jax sim_corridor_omni_external_ref_launch.py
    ros2 launch exact_mppi_jax sim_corridor_omni_external_ref_launch.py use_rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.logging import get_logger
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


logger = get_logger("exact_mppi_jax_sim_corridor_omni_external_ref_launch")


def generate_launch_description() -> LaunchDescription:

    sim_env_config_arg = DeclareLaunchArgument(
        "sim_env_config",
        default_value="scenario_corridor_omni.yaml",
        description="Simulation environment configuration file (in ddr_minimal_sim/config/)",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 for visualization",
    )

    rviz_config = os.path.join(
        get_package_share_directory("exact_mppi_jax"),
        "rviz",
        "mppi_sim.rviz",
    )

    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ddr_minimal_sim"), "/launch/complete_sim.launch.py"]),
        launch_arguments={
            "sim_env_config": LaunchConfiguration("sim_env_config"),
            "rviz": "false",
        }.items(),
    )

    global_ref_node = Node(
        package="exact_mppi_jax",
        executable="global_ref_path",
        name="global_ref_path",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": True},
            {"mppi_config_file": "corridor_omni_planner.yaml"},
        ],
    )

    mppi_local_node = Node(
        package="exact_mppi_jax",
        executable="mppi_local",
        name="mppi_local",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": True},
            {"mppi_config_file": "corridor_omni_planner.yaml"},
        ],
    )

    cmd_vel_watchdog_node = Node(
        package="exact_mppi_jax",
        executable="cmd_vel_watchdog",
        name="cmd_vel_watchdog",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": True},
            {"input_cmd_vel_topic": "/mppi_cmd_vel"},
            {"output_cmd_vel_topic": "/cmd_vel"},
            {"timeout_s": 0.3},
        ],
    )

    rviz_args = ["-d", rviz_config] if os.path.exists(rviz_config) else []
    if not rviz_args:
        logger.warning(f"RViz config not found: {rviz_config}")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=rviz_args,
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            sim_env_config_arg,
            use_rviz_arg,
            simulator_launch,
            global_ref_node,
            mppi_local_node,
            cmd_vel_watchdog_node,
            rviz_node,
        ]
    )
