from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os


def generate_launch_description():
    ainex_peripherals_pkg = get_package_share_directory('ainex_peripherals')
    resource_path = Path(get_package_share_directory('find_and_kick_ball'), 'resource')

    enable_camera = DeclareLaunchArgument(
        'enable_camera', default_value='true', description='Enable camera node'
    )
    camera_name = DeclareLaunchArgument(
        'camera_name', default_value='camera', description='Camera name'
    )
    image_topic = DeclareLaunchArgument(
        'image_topic', default_value='image_rect_color', description='Image topic name'
    )
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ainex_peripherals_pkg, 'launch', 'camera.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_camera')),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'image_topic': LaunchConfiguration('image_topic')
        }.items()
    )

    object_detection_node = Node(
            package = 'find_and_kick_ball',
            namespace = '',
            executable = 'object_detection_node',
            output = 'screen',
            parameters = [ {'resource_path' : str(resource_path)} ]
    )

    img_bbox_node = Node(
            package = 'find_and_kick_ball',
            namespace = '',
            executable = 'image_bbox_node',
            output = 'screen',
    )

    return LaunchDescription([
        enable_camera,
        camera_name,
        image_topic,
        camera_launch,
        object_detection_node,
        img_bbox_node
    ])
