import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import UnlessCondition
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    Command,
)
from launch_ros.actions import Node
import launch_ros


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="sam_bot_description"
    ).find("sam_bot_description")
    default_model_path = os.path.join(
        pkg_share, "src/description/sam_bot_description.urdf"
    )

    # nav = Node(
    #     package="nav2_bringup",
    #     executable="naviagtion_launch",
    #     name="nav2_bringup",
    #     output="screen",
    #     parameters=[
    #         os.path.join(pkg_share, "config/nav2_params.yaml"),
    #     ],
    # )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
            # {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    map_dummy = Node(
        package="dummy_map_server_mauz",
        executable="dummy_map_server",
        name="dummy_map_server",
    )

    robot_localization_node_odom_only = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf_only_odom.yaml"),
            # {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[("/odometry/filtered", "/odometry/filtered_only_odom")],
    )

    joy_params = os.path.join(os.path.join(pkg_share, "config/joystick.yaml"))

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_params],
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[joy_params],
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/slam_toolbox.yaml"),
        ],
    )

    # bringup_dir = get_package_share_directory("nav2_bringup")
    # launch_dir = os.path.join(bringup_dir, "launch")

    # nav = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(launch_dir, "navigation_launch.py")),
    #     launch_arguments={
    #         "params_file": os.path.join(pkg_share, "config/slam_toolbox.yaml"),
    #     }.items(),
    # )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="gui",
                default_value="True",
                description="Flag to enable joint_state_publisher_gui",
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=os.path.join(
                    pkg_share, "src/description/sam_bot_description.urdf"
                ),
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=os.path.join(pkg_share, "rviz/nav2_default_view.rviz"),
                description="Absolute path to rviz config file",
            ),
            joint_state_publisher_node,
            robot_state_publisher_node,
            # robot_localization_node,
            robot_localization_node_odom_only,
            rviz_node,
            joy_node,
            teleop_node,
            slam,
            map_dummy,
            # nav,
        ]
    )
