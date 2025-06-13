from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    ld = LaunchDescription()

    urdf_launch_package = FindPackageShare("urdf_launch")

    ld.add_action(
        DeclareLaunchArgument(
            name="jsp_gui",
            default_value="false",
            choices=["true", "false"],
            description="Flag to enable joint_state_publisher_gui",
        )
    )

    default_rviz_config_path = PathJoinSubstitution(
        [urdf_launch_package, "config", "urdf.rviz"]
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="rviz_config",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="urdf_package",
            default_value="ah_urdf",
            description="Package containing URDF",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="urdf_package_path",
            default_value=os.path.join(
                "urdf", "ability_hand_right_large.urdf"
            ),
            description="Path to URDF file",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="hand_base_link",
            default_value="world",
            description="Frame to attach hand to",
        )
    )

    # need to manually pass configuration in because of https://github.com/ros2/launch/issues/313
    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [urdf_launch_package, "launch", "description.launch.py"]
            ),
            launch_arguments={
                "urdf_package": LaunchConfiguration("urdf_package"),
                "urdf_package_path": LaunchConfiguration("urdf_package_path"),
            }.items(),
        )
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    ld.add_action(
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[
                {
                    "publish_default_positions": False,
                    "rate": 100,
                    "source_list": [
                        "joint_states_ah",
                    ],
                }
            ],
            condition=UnlessCondition(LaunchConfiguration("jsp_gui")),
        )
    )

    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("jsp_gui")),
        )
    )

    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_hand_tf_pub",
            arguments=[
                "0.0",
                "0.0",
                "0.0",  # x y z translation
                "0.0",
                "0.0",
                "0.0",  # roll pitch yaw (in radians)
                LaunchConfiguration("hand_base_link"),
                "base_link",  # parent frame, child frame
            ],
        )
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        )
    )

    return ld
