"""Start the MoveIt move_group node for the Goliath arm.

Run this ALONGSIDE the running sim (which already provides robot_state_publisher,
/joint_states and the arm_controller). move_group plans arm motions and executes
them through arm_controller/follow_joint_trajectory.

    ros2 launch goliath_moveit_config move_group.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Single-source the robot description from goliath_description (sim variant).
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
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # The SRDF roots the robot at a fixed "world" frame, but the live TF tree
    # starts at "odom" (diff-drive) -> base_footprint. This static identity
    # transform links MoveIt's planning frame to that tree so RViz can display
    # the robot and interactive markers ("No world frame" fix).
    world_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([move_group, world_to_odom])
