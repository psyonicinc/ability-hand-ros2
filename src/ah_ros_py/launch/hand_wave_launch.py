from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "write_thread",
                default_value="False",
                description="Enable write thread",
            ),
            Node(
                package="ah_ros_py",
                executable="ah_node",
                name="ah_node",
                output="screen",
                parameters=[
                    {"write_thread": LaunchConfiguration("write_thread")}
                ],
            ),
            Node(
                package="ah_ros_py",
                executable="hand_wave",
                name="hand_wave",
                output="screen",
            ),
        ]
    )
