from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dummy_usb_cam',
            executable='dummy_usb_cam',
            name='dummy_usb_cam'
        ),
        Node(
            package='image_conversion',
            executable='image_conversion',
            name='image_conversion',
            parameters=[{
                'input_topic': '/image_raw',
                'output_topic': '/image_converted'
            }]
        )
    ])
