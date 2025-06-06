# Ability Hand ROS2

ROS2 Humble implementation for integrating ability-hand python wrapper with ROS2

First follow the instructions [here] to install the Python Ability Hand Wrapper

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

using a *ah_messages/msg/Digits.msg* message, for example:

`ros2 topic pub -r 50 /ability_hand/target/position ah_messages/msg/Digits "{reply_mode: 0, data: [0.0, 90.0, 90.0, 0.0, 0.0, 0.0]}"`
