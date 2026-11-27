from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    localization = Node(
        package="localization",
        executable="localization",
        name="localization",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([localization])
