from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fake_visual_odometry_publisher',
            executable='publisher',
            name='fake_visual_odometry_publisher',
            output='screen',
            parameters=[]
        ),
    ])
