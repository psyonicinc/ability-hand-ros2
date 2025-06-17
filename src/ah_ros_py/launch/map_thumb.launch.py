import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    sub_launch_path = os.path.join(
        FindPackageShare("urdf_launch").find("urdf_launch"),
        "launch",
        "display.launch.py",
    )
    return LaunchDescription(
        [
            # Start URDF & RVIZ Node
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sub_launch_path)
            ),
            DeclareLaunchArgument(
                "write_thread",
                default_value="True",
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
                default_value="True",
                description="Publish Joint States for Joint State Publisher",
                choices=["True", "False"],
            ),
            DeclareLaunchArgument(
                "simulated_hand",
                default_value="True",
                description="Simulated Ability Hand",
                choices=["True", "False"],
            ),
            DeclareLaunchArgument(
                "hand_side",
                default_value="right",
                description="Right or Left Hand",
                choices=["right", "left"],
            ),
            DeclareLaunchArgument(
                "create_text_map",
                default_value="False",
                description="Create a text file of finger mappings",
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
                    {"simulated_hand": LaunchConfiguration("simulated_hand")},
                ],
            ),
            Node(
                package="ah_ros_py",
                executable="map_thumb_node",
                name="map_thumb_node",
                output="screen",
                parameters=[
                    {"hand_side": LaunchConfiguration("hand_side")},
                    {
                        "create_text_map": LaunchConfiguration(
                            "create_text_map"
                        )
                    },
                ],
            ),
        ]
    )
