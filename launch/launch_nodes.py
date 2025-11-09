from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beginner_tutorials',
            executable='talker',
            name='talker',
            output='screen'
        ),
        Node(
            package='beginner_tutorials',
            executable='listener',
            name='listener',
            output='screen'
        )
    ])
