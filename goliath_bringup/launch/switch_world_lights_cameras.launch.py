"""
Sim + status lights + auto doors + teleop relay + CAMERAS, in one launch.

Same as switch_world_lights.launch.py but with the two robot cameras enabled
(gripper camera -> /arm_camera/image_raw, base camera -> /base_camera/image_raw).

Usage:
    ros2 launch goliath_bringup switch_world_lights_cameras.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('goliath_bringup')
    lights = get_package_share_directory('goliath_lights')
    doors = get_package_share_directory('goliath_doors')

    # sim with cameras turned on
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'switch_world.launch.py')),
        launch_arguments={'enable_cameras': 'true'}.items())

    status_lights = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lights, 'launch', 'status_lights.launch.py')))

    auto_doors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(doors, 'launch', 'auto_doors.launch.py')))

    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', '/amr_controller/cmd_vel_unstamped'],
        output='screen',
    )

    return LaunchDescription([sim, status_lights, auto_doors, cmd_vel_relay])
