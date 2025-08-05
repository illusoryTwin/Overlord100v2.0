from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # RViz node
    rviz_config_file = PathJoinSubstitution([FindPackageShare("overlord100"), "rviz", "sim.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if launch argument is 'True'",
    )
    # Merging lidars
    merge_lidars = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros2_laser_scan_merger"),
                        "launch",
                        "merge_2_scan.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
        ],
    )
    # TODO: why simulator launcher is in diff_drive_control.launch.py
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("odom_pub_diff_control"),
                        "launch",
                        "diff_drive_control.launch.py",
                    ]
                )
            ]
        )
    )

    path_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("path_planner"),
                        "launch",
                        "path_planner_nav2.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
        ],
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("overlord100_bringup"),
                        "launch",
                        "simulator_control.launch.py",
                    ]
                )
            ]
        )
    )

    init_map_odom_spawner = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "map",
            "odom",
        ],
    )

    return LaunchDescription(
        [
            use_sim_time,
            init_map_odom_spawner,
            # Launch the main simulation
            sim_launch,
            # Launch the controller
            controller_launch,
            # Launch the path planner
            path_planner_launch,
            # Merge two laserscans together
            merge_lidars,
            # Finally launch RViz
            rviz_node,
        ]
    )
