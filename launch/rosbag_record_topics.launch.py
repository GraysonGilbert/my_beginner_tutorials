from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    # Launch argument to enable/disable bag recording
    enable_bag_recording = LaunchConfiguration("enable_bag_recording")

    declare_enable_bag_recording = DeclareLaunchArgument(
        "enable_bag_recording",
        default_value="true",
        description="Enable rosbag recording"
    )

    # Bag recording command (records ALL topics)
    rosbag_record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-a"],
        output="screen",
        condition=IfCondition(enable_bag_recording)
    )
    # Launch talker node
    talker = Node(
        package="beginner_tutorials",
        executable="talker",
        name="talker"
    )
    # Launch listener node
    listener = Node(
        package="beginner_tutorials",
        executable="listener",
        name="listener"
    )

    return LaunchDescription([
        declare_enable_bag_recording,
        talker,
        listener,
        rosbag_record
    ])
