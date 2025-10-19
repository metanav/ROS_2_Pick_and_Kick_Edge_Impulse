from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='image_raw',
    )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=LaunchConfiguration('camera_name'),
        namespace=LaunchConfiguration('camera_name'),
        output='screen',
        parameters=[
            {
                'camera_name': LaunchConfiguration('camera_name'),
                'video_device': '/dev/usbcam',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv',
                'av_device_format': 'YUV422P', 
                'camera_frame_id': LaunchConfiguration('camera_name'),
                'io_method': 'mmap',
                'camera_info_url': 'package://ainex_peripherals/config/head_camera.yaml'
            }
        ]
    )

    rgb_node = Node(
        package='ainex_peripherals',
        name='rgb_node',
        executable='rgb',  
        output='screen'
    )

    image_proc_node = Node(
        package='image_proc',
        executable='rectify_node',  
        name='rectify_color',
        namespace=LaunchConfiguration('camera_name'),
        #parameters=[{'queue_size': 10}],
        remappings=[
            ('image', 'image_rgb'),
            ('camera_info', 'camera_info'),
            ('image_rect', 'image_rect_color')
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_name_arg,
        image_topic_arg,
        usb_cam_node,
        rgb_node,
        image_proc_node
    ])
