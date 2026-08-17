import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    package_share = get_package_share_directory("sen0140_ros2")

    config_file = LaunchConfiguration(
        "config_file"
    ).perform(context)

    if not os.path.isabs(config_file):
        config_file = os.path.join(
            package_share,
            "config",
            config_file
        )

    node = Node(
        package="sen0140_ros2",
        executable="sen0140_node",
        name="sen0140_node",
        output="screen",
        parameters=[config_file],
    )

    return [node]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value="sen0140.yaml",
            description=(
                "Parameter YAML file. Relative paths are "
                "resolved against the package config directory."
            ),
        ),

        OpaqueFunction(
            function=launch_setup
        ),
    ])