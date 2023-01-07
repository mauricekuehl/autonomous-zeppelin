import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    Command,
)
from launch_ros.actions import Node
import launch_ros
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


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
        # output="screen",
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

    gui = DeclareLaunchArgument(
        name="gui",
        default_value="True",
        description="Flag to enable joint_state_publisher_gui",
    )

    model = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            pkg_share, "src/description/sam_bot_description.urdf"
        ),
        description="Absolute path to robot urdf file",
    )

    rviz_config = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=os.path.join(pkg_share, "rviz/nav2_default_view.rviz"),
        description="Absolute path to rviz config file",
    )

    """nav2"""
    #
    #
    #
    bringup_dir = get_package_share_directory("nav2_bringup")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "autostart": autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composed bringup if True",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="the name of conatiner that nodes will load in if use composition",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings
                + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_controller",
                plugin="nav2_controller::ControllerServer",
                name="controller_server",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            ComposableNode(
                package="nav2_smoother",
                plugin="nav2_smoother::SmootherServer",
                name="smoother_server",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_planner",
                plugin="nav2_planner::PlannerServer",
                name="planner_server",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_behaviors",
                plugin="behavior_server::BehaviorServer",
                name="behavior_server",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_bt_navigator",
                plugin="nav2_bt_navigator::BtNavigator",
                name="bt_navigator",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_waypoint_follower",
                plugin="nav2_waypoint_follower::WaypointFollower",
                name="waypoint_follower",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_velocity_smoother",
                plugin="nav2_velocity_smoother::VelocitySmoother",
                name="velocity_smoother",
                parameters=[configured_params],
                remappings=remappings
                + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    #
    #
    #

    # Create the launch description and populate
    ld = LaunchDescription()

    # # Set environment variables
    # ld.add_action(stdout_linebuf_envvar)

    # # Declare the launch options
    # ld.add_action(declare_namespace_cmd)
    # ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_params_file_cmd)
    # ld.add_action(declare_autostart_cmd)
    # ld.add_action(declare_use_composition_cmd)
    # ld.add_action(declare_container_name_cmd)
    # ld.add_action(declare_use_respawn_cmd)
    # ld.add_action(declare_log_level_cmd)
    # # Add the actions to launch all of the navigation nodes
    # ld.add_action(load_nodes)
    # ld.add_action(load_composable_nodes)

    """none nav2"""
    ld.add_action(gui)
    ld.add_action(model)
    ld.add_action(rviz_config)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(robot_localization_node)
    # ld.add_action(robot_localization_node_odom_only)
    ld.add_action(rviz_node)
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    ld.add_action(slam)
    # ld.add_action(map_dummy)

    return ld
