from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_goliath = get_package_share_directory("goliath_controller")
    topics_yaml = os.path.join(pkg_goliath, "config", "twist_mux_topics.yaml")

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[topics_yaml],
        remappings=[("/cmd_vel_out", "/amr_controller/cmd_vel_unstamped")]
    )

    return LaunchDescription([
        twist_mux_node
    ])
