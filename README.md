# Ability Hand ROS2

ROS2 Humble implementation for integrating Ability Hand [Python Wrapper](https://github.com/psyonicinc/ability-hand-api/tree/master/python) 
with ROS2


### Regular Setup

#### Install ROS2 Humble

https://docs.ros.org/en/humble/Installation.html

It is recommended to use the included localhost only cyclonedds.xml config file.
To install cyclone dds and use the included config file run

`./cyclone_dds_install.sh`

Follow the instructions [here](https://github.com/psyonicinc/ability-hand-api/tree/master/python) 
to install the Python Ability Hand Wrapper

From ability-hand-ros2 directory build using `colcon build` and source using `source ./install/setup.bash`

### Docker Setup

Choose which nodes you wish to launch using Docker by modifying `entrypoint.sh`

#### Windows

- Install [WSL](https://learn.microsoft.com/en-us/windows/wsl/install)
- Install [Docker Desktop](https://www.docker.com/products/docker-desktop/)
- Install [XLaunch](https://sourceforge.net/projects/vcxsrv/)
- Launch Docker Desktop
- Launch XLaunch program select Multiple windows, Next, Start no client, Next, Un-Select Native opengl, Next, Finish
- Open a power shell
- Enter `$env:DISPLAY="host.docker.internal:0.0"`
- Navigate to the docker directory of this repo and enter the following command

`docker compose up`

#### Linux

Docker installation instructions for various linux distributions can be found
[here](https://docs.docker.com/engine/install/). Ensure you do the 
[post installation instructions](https://docs.docker.com/engine/install/linux-postinstall/)

Navigate to the docker directory of this repo and enter the following command

`xhost + && docker compose up`

### Topics

You can receive motor feedback on the following topics:

`/ability_hand/feedback/velocity`  
`/ability_hand/feedback/position`  
`/ability_hand/feedback/current`  

Using the following indexes  
`[index, middle, ring, pinky, thumb flexor,  thumb rotator]`

You can receive touch sensor feedback as well with 6 touch sensors per finger
via the topic:

`/ability_hand/feedback/touch`  

To control the hand publish on any of the following topics:

`/ability_hand/target/velocity`  
`/ability_hand/target/position`  
`/ability_hand/target/current`  
`/ability_hand/target/duty`

using an *ah_messages/msg/Digits.msg* message, for example:

`ros2 topic pub -r 200 /ability_hand/target/position ah_messages/msg/Digits "{reply_mode: 0, data: [0.0, 90.0, 90.0, 0.0, 0.0, 0.0]}"`

### Examples

To start the Ability Hand Node with a automatic write thread use:

`colcon build && source ./install/setup.bash`  
`ros2 launch ah_ros_py ah_node.launch.py write_thread:=True`

You can change the hand's position using
`ros2 topic pub --once /ability_hand/target/position ah_messages/msg/Digits "{reply_mode: 0, data: [0.0, 90.0, 90.0, 0.0, 0.0, 0.0]}"`

Since the write thread is running you only have to publish the message once. In 
most typical cases you will set write_thread:=False and have another ROS node 
which will be publishing to the hand at least every 0.3 seconds / 5 hz. Ideally 
you publish at least 200hz for smoother control.

For example run the hand wave node without a write thread:

`ros2 launch ah_ros_py hand_wave.launch.py`

If you enable the `js_publisher` argument in the `ah_node.launch.py` launch file
it will publish to `/joint_states_ah` which the joint state publisher in the
`urdf_launch` package subscribes too and allows you to visualise the joint states
and URDF using:

`ros2 launch ah_ros_py hand_wave.launch.py js_publisher:=True`  
`ros2 launch urdf_launch display.launch.py`
