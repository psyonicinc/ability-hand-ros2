from setuptools import setup

package_name = "ah_ros_py"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/ah_ros_py/launch", ["launch/ah_node_launch.py"]),
        ("share/ah_ros_py/launch", ["launch/hand_wave_launch.py"]),
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
