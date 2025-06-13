from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # AH NODE SETUP
            DeclareLaunchArgument(
                "write_thread",
                default_value="False",
                description="Enable write thread",
                choices=["True", "False"],
            ),
            DeclareLaunchArgument(
                "port",
                default_value="",
                description="Serial Port",
            ),
            DeclareLaunchArgument(
                "baud_rate",
                default_value="0",
                description="Baud Rate",
            ),
            DeclareLaunchArgument(
                "hand_size",
                default_value="Large",
                description="Ability Hand Size (Small/Large)",
                choices=["Small", "Large"],
            ),
            DeclareLaunchArgument(
                "js_publisher",
                default_value="False",
                description="Publish Joint States for Joint State Publisher",
                choices=["True", "False"],
            ),
            Node(
                package="ah_ros_py",
                executable="ah_node",
                name="ah_node",
                output="screen",
                parameters=[
                    {"write_thread": LaunchConfiguration("write_thread")},
                    {"port": LaunchConfiguration("port")},
                    {"baud_rate": LaunchConfiguration("baud_rate")},
                    {"js_publisher": LaunchConfiguration("js_publisher")},
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
