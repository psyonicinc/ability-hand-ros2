from setuptools import setup
import os

package_name = "ah_ros_py"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            [os.path.join("launch", "ah_node.launch.py")],
        ),
        (
            os.path.join("share", package_name, "launch"),
            [os.path.join("launch", "hand_wave.launch.py")],
        ),
    ],
    install_requires=["setuptools", "ability-hand"],
    entry_points={
        "console_scripts": [
            "ah_node = ah_ros_py.ah_node:main",
            "hand_wave = ah_ros_py.hand_wave:main",
        ],
    },
    zip_safe=True,
    maintainer="Justin Francis",
    maintainer_email="jfrancis@psyonic.io",
    description="Python ROS2 Package for Controlling PSYONIC Ability Hand",
    license="MIT",
)
