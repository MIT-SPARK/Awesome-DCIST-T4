from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="topic_tools",
                executable="throttle",
                arguments=["messages", "/tf", "2"],
                output="screen",
            )
        ]
    )
