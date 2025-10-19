from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share_dir = get_package_share_directory('ainex_kinematics')
    script_path = os.path.join(package_share_dir, 'kinematics.py')

    pig_daemon = ExecuteProcess(
        cmd=['/usr/bin/python3.8', script_path],
        output='screen',  
        shell=False
    )

    return LaunchDescription([
        kinematics_daemon
    ])
