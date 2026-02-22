from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("gazebo_keyboard_cmdvel")

    default_bridge_yaml = os.path.join(pkg_share, "config", "bridge.yaml")
    default_node_params = os.path.join(pkg_share, "config", "keyboard_to_cmdvel.yaml")

    # ---------- Arguments ----------
    declare_launch_gazebo = DeclareLaunchArgument(
        "launch_gazebo",
        default_value="False",
        description="If True, start Gazebo (gz sim) from this launch file."
    )

    declare_world = DeclareLaunchArgument(
        "world",
        default_value="",
        description="Absolute path to SDF world to pass to 'gz sim'. Used only if launch_gazebo:=True."
    )

    declare_bridge_yaml = DeclareLaunchArgument(
        "bridge_yaml",
        default_value=default_bridge_yaml,
        description="Bridge YAML config file (ros_gz_bridge parameter_bridge config)."
    )

    declare_node_params = DeclareLaunchArgument(
        "node_params",
        default_value=default_node_params,
        description="ROS parameters YAML for keyboard_to_cmdvel node."
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level ROS namespace."
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use /clock if available."
    )

    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for nodes (debug, info, warn, error, fatal)."
    )

    declare_respawn = DeclareLaunchArgument(
        "respawn",
        default_value="False",
        description="Respawn nodes if they crash."
    )

    # ---------- Nodes / Processes ----------
    # 1) Gazebo (optionnel)
    # NOTE: launch_ros Node ne lance pas "gz sim". Pour cela on utilise ExecuteProcess.
    # Pour rester simple et portable, je te le mets en option.
    from launch.actions import ExecuteProcess

    gz_sim = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("launch_gazebo")),
        cmd=["gz", "sim", LaunchConfiguration("world")],
        output="screen",
    )

    # 2) Bridge (lis le YAML)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        respawn=LaunchConfiguration("respawn"),
        arguments=[
            "--ros-args",
            "-p",
            ["config_file:=", LaunchConfiguration("bridge_yaml")],
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # 3) Node clavier -> cmd_vel
    keyboard_to_cmdvel = Node(
        package="gazebo_keyboard_cmdvel",
        executable="keyboard_to_cmdvel",
        name="keyboard_to_cmdvel",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        respawn=LaunchConfiguration("respawn"),
        parameters=[
            LaunchConfiguration("node_params"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
    )

    ld = LaunchDescription()

    # Register arguments
    ld.add_action(declare_launch_gazebo)
    ld.add_action(declare_world)
    ld.add_action(declare_bridge_yaml)
    ld.add_action(declare_node_params)
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_log_level)
    ld.add_action(declare_respawn)

    # Start actions
    ld.add_action(gz_sim)
    ld.add_action(bridge)
    ld.add_action(keyboard_to_cmdvel)

    return ld