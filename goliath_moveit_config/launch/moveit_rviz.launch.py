"""RViz with the MotionPlanning plugin for the Goliath arm.

    ros2 launch goliath_moveit_config moveit_rviz.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    urdf = os.path.join(
        get_package_share_directory("goliath_description"),
        "urdf", "goliath.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder("goliath", package_name="goliath_moveit_config")
        .robot_description(
            file_path=urdf,
            mappings={"is_sim": "true", "enable_cameras": "false"},
        )
        .robot_description_semantic(file_path="config/goliath.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    rviz_config = os.path.join(
        get_package_share_directory("goliath_moveit_config"), "config", "moveit.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([rviz])
