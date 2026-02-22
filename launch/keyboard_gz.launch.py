from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("gazebo_keyboard_cmdvel")
    bridge_yaml = os.path.join(pkg_share, "config", "bridge.yaml")
    #params_file = os.path.join(pkg_share, "config", "keyboard_to_cmdvel.yaml")   # Si vous avez des paramètres à charger pour le noeud keyboard_to_cmdvel

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=["--ros-args", "-p", f"config_file:={bridge_yaml}"],
    )

    keyboard_to_cmdvel = Node(
        package="gazebo_keyboard_cmdvel",
        executable="keyboard_to_cmdvel",
        name="keyboard_to_cmdvel",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        bridge,
        keyboard_to_cmdvel,
    ])