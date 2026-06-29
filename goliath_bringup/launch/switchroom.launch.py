import os
from os import pathsep

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ---------------------------------------------------------------------
    # Package directories
    # ---------------------------------------------------------------------
    goliath_description = get_package_share_directory("goliath_description")
    goliath_description_prefix = get_package_prefix("goliath_description")
    goliath_controller = get_package_share_directory("goliath_controller")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    # Path to the switch wall model (spawned as a separate entity)
    switch_wall_model = os.path.join(
        goliath_description, "models", "switch_wall", "model.sdf"
    )

    # Path to the robot description (mobile base + arm)
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            goliath_description, "urdf", "goliath.urdf.xacro"
        ),
        description="Absolute path to the robot xacro file",
    )

    # ---------------------------------------------------------------------
    # Gazebo Classic needs to find the model:// resources and meshes
    # ---------------------------------------------------------------------
    model_path = os.path.join(goliath_description, "models")
    model_path += pathsep + os.path.join(goliath_description_prefix, "share")

    gazebo_model_path = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # ---------------------------------------------------------------------
    # Robot description / state publisher
    # ---------------------------------------------------------------------
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model"), " is_sim:=true"]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # ---------------------------------------------------------------------
    # Gazebo Classic server + client (empty world)
    # ---------------------------------------------------------------------
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        ),
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    # ---------------------------------------------------------------------
    # Spawn entities
    # ---------------------------------------------------------------------
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "goliath",
            "-topic", "robot_description",
            "-x", "1.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
        output="screen",
    )

    spawn_wall = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "switch_wall",
            "-file", switch_wall_model,
            "-x", "0.0",
            "-y", "1.0",
            "-z", "0.0",
        ],
        output="screen",
    )

    # ---------------------------------------------------------------------
    # ros2_control controllers (loaded once the robot is in Gazebo)
    # ---------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    amr_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["amr_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Start the controllers only after the robot has finished spawning so the
    # controller_manager (provided by gazebo_ros2_control) is available.
    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                amr_controller_spawner,
                arm_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        gazebo_model_path,
        model_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot,
        spawn_wall,
        controllers_after_spawn,
    ])
