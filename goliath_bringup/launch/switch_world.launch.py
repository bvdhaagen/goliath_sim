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
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    # Pad naar het switch wall model
    switch_wall_model = os.path.join(
        goliath_description, "models", "switch_wall", "model.sdf"
    )

    # Pad naar de robot description (mobile base + arm)
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            goliath_description, "urdf", "goliath.urdf.xacro"
        ),
        description="Absolute path to the robot xacro file",
    )

    # ---------------------------------------------------------------------
    # Gazebo Classic model paden configureren
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
    # Gazebo Classic server + client (Laat de wereld MET muren in)
    # ---------------------------------------------------------------------
    datacenter_world_path = os.path.join(goliath_description, "worlds", "datacenter.world")

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": datacenter_world_path}.items(),
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    # ---------------------------------------------------------------------
    # Spawn losse entities (Schakelkast, Robot)
    # ---------------------------------------------------------------------
    # Spawn jouw schakelkasten muur in de technische ruimte
    spawn_wall = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "switch_wall",
            "-file", switch_wall_model,
            "-x", "16.5", "-y", "7", "-z", "0.0",
        ],
        output="screen",
    )

    # Spawn Goliath de robot aan het begin van de gang
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "goliath",
            "-topic", "robot_description",
            "-x", "1.0", "-y", "1.0", "-z", "0.0",
        ],
        output="screen",
    )

    # ---------------------------------------------------------------------
    # ros2_control controllers
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

    # Start controllers pas nadat de robot succesvol is geladen
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
        spawn_wall,             # Spawnt de schakelkast los in de kamer
        spawn_robot,            # Spawnt de robot los in de gang
        controllers_after_spawn,
    ])
