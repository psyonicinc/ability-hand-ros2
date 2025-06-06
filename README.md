# Ability Hand ROS2

ROS2 Humble implementation for integrating ability-hand python wrapper with ROS2

First follow the instructions [here](https://github.com/psyonicinc/ability-hand-api/tree/master/python) to install the Python Ability Hand Wrapper

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

`ros2 topic pub -r 50 /ability_hand/target/position ah_messages/msg/Digits "{reply_mode: 0, data: [0.0, 90.0, 90.0, 0.0, 0.0, 0.0]}"`

### Examples

To start the Ability Hand Node with a automatic write thread use:

`ros2 launch ah_ros_py ah_node_launch.py write_thread:=True`

In most typical cases you will have another ROS node which will be publishing to
the hand at least every 0.3 seconds / 5 hz.  Ideally you publish at least 200hz
for smoother control.

or run a hand wave example without a write thread:

`ros2 launch ah_ros_py hand_wave_launch.py`

