import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from os.path import join


def generate_launch_description():

    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_goliath_description = get_package_share_directory('goliath_description')
    
    # Path to your model
    switch_wall_model = os.path.join(
        pkg_goliath_description,
        'models',
        'switch_wall',
        'model.sdf'
    )
    
    # Path to robot description (your arm + base)
    robot_description_file = os.path.join(
        pkg_goliath_description,
        'urdf',
        'goliath.urdf.xacro'
    )
    
    # Parse robot description
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': True,
    }

    # Set environment variables for Gazebo to find meshes
    # This tells Gazebo where to look for model:// URIs
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        [
            os.path.join(pkg_goliath_description, 'models'), ':',
            os.path.join(pkg_goliath_description, 'meshes'), ':',
            os.path.dirname(pkg_goliath_description), ':',
            os.environ.get('GAZEBO_MODEL_PATH', '')
        ]
    )
    
    # For Ignition Gazebo resource paths
    ign_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        [
            os.path.join(pkg_goliath_description, 'models'), ':',
            os.path.join(pkg_goliath_description, 'meshes'), ':',
            os.path.dirname(pkg_goliath_description), ':',
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
        ]
    )
    
    # Also add to IGN_RESOURCE_PATH (for general Ignition resources)
    ign_path = SetEnvironmentVariable(
        'IGN_RESOURCE_PATH',
        [
            os.path.join(pkg_goliath_description, 'meshes'), ':',
            os.environ.get('IGN_RESOURCE_PATH', '')
        ]
    )

    # Start Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": '-r -v 4 empty.sdf'
        }.items()
    )

    # Spawn the switch wall model
    spawn_wall = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            "-file", switch_wall_model,
            "-name", "switch_wall",
            "-x", "0.0",
            "-y", "1.0",
            "-z", "0.0",
            "-Y", "0.0"
        ],
        output='screen',
    )

    # Spawn your robot arm on a base
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            "-topic", "/robot_description",
            "-name", "goliath",
            "-allow_renaming", "true",
            "-z", "0.0",
            "-x", "1.0",
            "-y", "0.0",
            "-Y", "0.0"
        ],
        output='screen',
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Bridge for ROS-Gazebo communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_model_path,
        ign_resource_path,
        ign_path,
        gazebo,
        spawn_wall,
        spawn_robot,
        robot_state_publisher,
        bridge,
    ])
