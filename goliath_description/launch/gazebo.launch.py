import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    goliath_description = get_package_share_directory("goliath_description")
    goliath_description_prefix = get_package_prefix("goliath_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                      goliath_description, "urdf", "goliath.urdf.xacro"
                                      ),
                                      description="Absolute path to robot urdf file"
    )
    
    
    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="datacenter")

    world_path = PathJoinSubstitution([
            goliath_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = os.path.join(goliath_description, "models")
    model_path += pathsep + os.path.join(goliath_description_prefix, "share")

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    robot_description = ParameterValue(
        Command([
        "xacro ", 
        LaunchConfiguration("model"),
        " enable_cameras:=", LaunchConfiguration("enable_cameras", default="true")
    ]),
    value_type=str
)
    #======================================================
    # fixed models to be included in world later 
    #======================================================
    
    switch_wall_model = os.path.join(
        goliath_description, "models", "switch_wall", "model.sdf"
    )
    
    battery_rack_model = os.path.join(
        goliath_description, "models", "battery_rack", "model.sdf"
    )
        
    
    #======================================================
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        ),
        
        launch_arguments={
            "world": world_path, "enable_cameras": "True"
        }.items(),
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    spawn_robot = Node(package="gazebo_ros", executable="spawn_entity.py",
                        arguments=["-entity", "goliath",
                                   "-topic", "robot_description",
                                   "-x", "2.0", "-y", "1.0", "-z", "0.0",
                                  ],
                        output="screen"
    )
    
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
    
    spawn_rack = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "battery_rack",
            "-file", battery_rack_model,
            "-x", "9.0", "-y", "4.5", "-z", "0.0",
        ],
        output="screen",
    )
    
    
    return LaunchDescription([
        env_var,
        model_arg,
        world_name_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot
        #spawn_rack,
        #spawn_wall
    ])
