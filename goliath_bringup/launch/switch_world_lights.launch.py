"""
Sim + status lights + auto doors + teleop relay, in one launch.

Brings up the world + robot + switch_wall (switch_world.launch.py) together with:
  * the switch status lights (goliath_lights),
  * the automatic proximity doors (goliath_doors),
  * a /cmd_vel relay so teleop drives the robot directly.

Usage:
    ros2 launch goliath_bringup switch_world_lights.launch.py
    # then, in another terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
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

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'switch_world.launch.py')))

    status_lights = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lights, 'launch', 'status_lights.launch.py')))

    auto_doors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(doors, 'launch', 'auto_doors.launch.py')))

    # teleop publishes /cmd_vel; the diff-drive controller listens on
    # /amr_controller/cmd_vel_unstamped. Forward one to the other.
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', '/amr_controller/cmd_vel_unstamped'],
        output='screen',
    )

    return LaunchDescription([sim, status_lights, auto_doors, cmd_vel_relay])
