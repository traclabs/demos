"""Launch the ingenuity demo."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription  # type: ignore
from launch_ros.actions import Node, SetParameter  # type: ignore

def generate_launch_description():
    """Launch the ingenuity demo."""

    altitude_node = Node(
        package="ingenuity_demo", executable="move_vertical", output="screen"
    )

    plane_node = Node(
        package="ingenuity_demo", executable="move_xy", output="screen"
    )


    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            altitude_node,
            plane_node
        ]
    )
