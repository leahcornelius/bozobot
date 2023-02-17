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
        
    ])