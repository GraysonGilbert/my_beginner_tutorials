from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare a launch argument for publish_rate with a default value
    declare_publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='0.5',
        description='Publish rate for the talker node'
    )

    # Use LaunchConfiguration to retrieve the value
    publish_rate = LaunchConfiguration('publish_rate')

    # Define nodes
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='talker',
        output='screen',
        parameters=[{'publish_rate': publish_rate}],
        arguments=['--ros-args', '--log-level', 'debug']
    )

    listener_node = Node(
        package='beginner_tutorials',
        executable='listener',
        name='listener',
        output='screen',
        arguments=['--ros-args', '--log-level', 'debug']
    )

    # Return LaunchDescription
    return LaunchDescription([
        declare_publish_rate_arg,
        talker_node,
        listener_node
    ])
