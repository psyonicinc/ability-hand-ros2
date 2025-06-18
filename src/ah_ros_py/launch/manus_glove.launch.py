import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
                default_value="False",
                description="Simulated Ability Hand",
                choices=["True", "False"],
            ),
            DeclareLaunchArgument(
                "right_base_frame",
                default_value="world",
                description="Base frame right hand is attached to",
            ),
            DeclareLaunchArgument(
                "left_base_frame",
                default_value="world",
                description="Base frame left hand is attached to",
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
                    {
                        "right_base_frame": LaunchConfiguration(
                            "right_base_frame"
                        )
                    },
                ],
            ),
            Node(
                package="ah_ros_py",
                executable="manus_glove_node",
                name="manus_glove",
                output="screen",
            ),
            Node(
                package="manus_ros2",
                executable="manus_data_publisher",
                name="manus_data_publisher",
                output="screen",
            ),
        ]
    )
