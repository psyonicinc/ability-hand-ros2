from setuptools import setup
import os

package_name = "ah_ros_py"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            os.path.join("share", package_name, "data"),
            [
                os.path.join(package_name, "data", "all_fingers_right.pkl"),
                os.path.join(package_name, "data", "all_fingers_left.pkl"),
                os.path.join(package_name, "data", "thumb_left.pkl"),
                os.path.join(package_name, "data", "thumb_right.pkl"),
            ],
        ),
        (
            os.path.join("share", package_name),
            [
                os.path.join(package_name, "plots.py"),
            ],
        ),
        (
            os.path.join("share", package_name, "images"),
            [
                os.path.join(
                    package_name, "images", "touch_sensor_legend_sml.png"
                ),
            ],
        ),
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            [
                os.path.join("launch", "ah_node.launch.py"),
                os.path.join("launch", "hand_wave.launch.py"),
                os.path.join("launch", "manus_glove.launch.py"),
                os.path.join("launch", "map_fingerpoints.launch.py"),
            ],
        ),
    ],
    install_requires=[
        "setuptools",
        "ability-hand",
        "numpy==1.24",
        "scipy==1.8",
        "ikpy>=3.4.2",
    ],
    entry_points={
        "console_scripts": [
            "ah_node = ah_ros_py.ah_node:main",
            "hand_wave_node = ah_ros_py.hand_wave_node:main",
            "manus_glove_node = ah_ros_py.manus_glove_node:main",
            "map_fingerpoints_node = ah_ros_py.map_fingerpoints_node:main",
            "build_thumb_maps = ah_ros_py.build_thumb_maps:main",
        ],
    },
    zip_safe=True,
    maintainer="Justin Francis",
    maintainer_email="jfrancis@psyonic.io",
    description="Python ROS2 Package for Controlling PSYONIC Ability Hand",
    license="MIT",
)
