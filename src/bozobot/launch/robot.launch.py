from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bozobot',
            namespace='bozobot',
            executable='telop_subscriber',
            name='telop_subscriber', 
        ),
        # RosUsbCam
        Node(
            package='usb_cam',
            namespace='bozobot',
            executable='usb_cam_node_exe',
            name='usb_cam_node_exe',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'},
                {'camera_frame_id': 'camera'},
                {'io_method': 'mmap'},
                {'image_pub_rate': 30},
                {'camera_name': 'bozobot'},
            ]
        ),
    ])