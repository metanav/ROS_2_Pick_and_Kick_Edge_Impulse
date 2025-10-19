import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    ainex_kinematics_pkg = get_package_share_directory('ainex_kinematics')

    servo_params_file = os.path.join(ainex_kinematics_pkg, 'config/servo_controller.yaml')
    init_pose_params_file    = os.path.join(ainex_kinematics_pkg, 'config/init_pose.yaml')
    walking_params_file = os.path.join(ainex_kinematics_pkg, 'config/walking_params.yaml')
    action_group_path = os.path.join(ainex_kinematics_pkg, 'config/action_groups')

    controller_Node = Node(
        package='ainex_kinematics',
        executable='controller_node',
        name='controller_node',
        parameters=[
            {
                'init_pose_params_file': init_pose_params_file,
                'servo_params_file': servo_params_file,
                'walking_params_file': walking_params_file,
                'action_group_path' : action_group_path
            }
        ],
        output='screen'
    )


    return LaunchDescription([
        controller_Node
    ])
