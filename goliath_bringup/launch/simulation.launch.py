import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("goliath_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("goliath_controller"),
            "launch",
            "controller.launch.py"
        ),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("goliath_controller"),
            "launch",
            "twist_mux.launch.py"
        ),
    )


    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("goliath_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("goliath_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("goliath_mapping"),
                "rviz",
                "slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("goliath_nav2"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_slam)
    )


    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("goliath_moveit"),
            "launch",
            "moveit.launch.py"
        ),
    )

    
    
    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        joystick,
        localization,
        slam,
        navigation,
        rviz_slam,
        rviz
        # moveit
    ])