import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('goliath_doors'), 'config', 'doors.yaml')

    auto_doors = Node(
        package='goliath_doors',
        executable='auto_doors',
        name='auto_doors',
        output='screen',
        parameters=[{'use_sim_time': True, 'config': cfg}],
    )

    return LaunchDescription([auto_doors])
