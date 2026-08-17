import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_share_dir = get_package_share_directory(
        "sen0140_ros2"
    )

    default_config_file = os.path.join(
        package_share_dir,
        "config",
        "sen0140.yaml"
    )

    config_file = LaunchConfiguration(
        "config_file"
    )

    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_file,
        description="Path to the SEN0140 parameter YAML file"
    )

    sen0140_node = Node(
        package="sen0140_ros2",
        executable="sen0140_node",
        name="sen0140_node",
        output="screen",
        parameters=[
            config_file
        ]
    )

    return LaunchDescription([
        declare_config_file,
        sen0140_node,
    ])