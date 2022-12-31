import launch
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="sam_bot_description"
    ).find("sam_bot_description")
    slam = launch_ros.actions.Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/slam_toolbox.yaml"),
        ],
    )

    return launch.LaunchDescription([slam])
