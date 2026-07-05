import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('goliath_lights'), 'config', 'lights.yaml')

    status_lights = Node(
        package='goliath_lights',
        executable='status_lights',
        name='status_lights',
        output='screen',
        parameters=[{'use_sim_time': True, 'config': cfg}],
    )

    return LaunchDescription([status_lights])
