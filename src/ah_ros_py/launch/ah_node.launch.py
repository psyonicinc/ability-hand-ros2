from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
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
                default_value="large",
                description="Ability Hand Size (Small/Large)",
                choices=["small", "large"],
            ),
            DeclareLaunchArgument(
                "hand_side",
                default_value="right",
                description="Left or Right Hand",
                choices=["right", "left"],
            ),
            DeclareLaunchArgument(
                "js_publisher",
                default_value="false",
                description="Publish Joint States for Joint State Publisher",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "simulated_hand",
                default_value="false",
                description="Simulated Ability Hand",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "plot_fsr",
                default_value="false",
                description="Plot touch sensors",
                choices=["true", "false"],
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
                    {"hand_side": LaunchConfiguration("hand_side")},
                    {"plot_fsr": LaunchConfiguration("plot_fsr")},
                ],
            ),
        ]
    )
