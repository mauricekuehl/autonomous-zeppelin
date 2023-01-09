import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("bno055"), "config", "bno055_params.yaml"
    )

    bno = Node(package="bno055", executable="bno055", parameters=[config])

    hardware = Node(package="hardware", output="screen", executable="drive_hardware")

    lidar = Node(package="tfmini_ros", executable="tfmini_ros_node")

    ld.add_action(hardware)
    ld.add_action(bno)
    ld.add_action(lidar)
    return ld
