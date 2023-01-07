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
        package="nav2_bringup",
        executable="naviagtion_launch",
        name="nav2_bringup",
        # output="screen",
        # parameters=[
        #     os.path.join(pkg_share, "config/nav2_params.yaml"),
        # ],
    )

    return launch.LaunchDescription([slam])
