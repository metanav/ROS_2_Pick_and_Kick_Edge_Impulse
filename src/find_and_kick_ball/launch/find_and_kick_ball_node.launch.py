from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    find_and_kick_ball_pkg = get_package_share_directory('find_and_kick_ball')
    calib_config_file = os.path.join(find_and_kick_ball_pkg, 'config/calib.yaml')
    action_group_path = os.path.join(find_and_kick_ball_pkg, 'config/action_groups')

    ainex_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ainex_kinematics'),
                'launch', 'kinematics.launch.py'
            )
        )
    )

    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('find_and_kick_ball'),
                'launch', 'object_detection_node.launch.py'
            )
        )
    )

    # Define the kick_ball node
    kick_ball_node = Node(
        package='find_and_kick_ball',
        executable='find_and_kick_ball_node',
        name='find_and_kick_ball',
        output='screen',
        parameters=[
            {
                'calib_config_file' : calib_config_file,
            }
        ]
    )
    
    # Return the launch description
    return LaunchDescription([
        ainex_controller_launch,
        object_detection_launch,
        kick_ball_node
    ])
