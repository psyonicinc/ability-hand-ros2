from math import radians

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt16
from sensor_msgs.msg import JointState
from ah_messages.msg import Digits

from ah_wrapper import AHSerialClient
from ah_wrapper.observer import Observer


class AbilityHandNode(Node, Observer):
    def __init__(self):
        super().__init__("ability_hand_node")
        self.declare_parameter("write_thread", True)
        self.declare_parameter("port", "")
        self.declare_parameter("baud_rate", 0)
        self.declare_parameter("hand_size", "large")
        self.declare_parameter("js_publisher", False)

        # Read Parameters
        self.js_publisher = (
            self.get_parameter("js_publisher").get_parameter_value().bool_value
        )
        self.write_thread = (
            self.get_parameter("write_thread").get_parameter_value().bool_value
        )
        self.hand_size = (
            self.get_parameter("hand_size")
            .get_parameter_value()
            .string_value.lower()
        )
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = (
            self.get_parameter("baud_rate").get_parameter_value().integer_value
        )

        if port and baud:
            self.client = AHSerialClient(
                write_thread=self.write_thread, port=port, baud_rate=baud
            )
        elif port:
            self.client = AHSerialClient(
                write_thread=self.write_thread, port=port
            )
        elif baud:
            self.client = AHSerialClient(
                write_thread=self.write_thread, baud_rate=baud
            )
        else:
            self.client = AHSerialClient(write_thread=self.write_thread)

        self.client.hand.add_observer(self)

        # Joint names and states
        self.joint_names = [
            "index_q1",
            "index_q2",
            "middle_q1",
            "middle_q2",
            "ring_q1",
            "ring_q2",
            "pinky_q1",
            "pinky_q2",
            "thumb_q1",
            "thumb_q2",
        ]
        self.joint_states = [0.0] * 10

        # Feedback publishers
        self.pub_velocity_fb = self.create_publisher(
            Float32MultiArray, "/ability_hand/feedback/velocity", 10
        )
        self.pub_position_fb = self.create_publisher(
            Float32MultiArray, "/ability_hand/feedback/position", 10
        )
        self.pub_current_fb = self.create_publisher(
            Float32MultiArray, "/ability_hand/feedback/current", 10
        )
        self.pub_touch_fb = self.create_publisher(
            Float32MultiArray, "/ability_hand/feedback/touch", 10
        )
        self.pub_hot_cold_fb = self.create_publisher(
            UInt16, "/ability_hand/feedback/hot_cold", 10
        )

        if self.js_publisher:
            self.js_pub = self.create_publisher(
                JointState, "/joint_states_ah", 10
            )

        # Target subscribers
        self.sub_velocity_target = self.create_subscription(
            Digits, "/ability_hand/target/velocity", self.velocity_callback, 10
        )
        self.sub_position_target = self.create_subscription(
            Digits, "/ability_hand/target/position", self.position_callback, 10
        )
        self.sub_current_target = self.create_subscription(
            Digits, "/ability_hand/target/current", self.current_callback, 10
        )
        self.sub_duty_target = self.create_subscription(
            Digits, "/ability_hand/target/duty", self.duty_callback, 10
        )

    def safe_publish(self, pub, msg):
        # Call publish outside of client class... I think... Not really... But seems better this way... This causes less errors in the log...
        if rclpy.ok():
            try:
                pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish: {e}")
        else:
            self.get_logger().warn("ROS context is invalid")

    def publish_joint_states(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        for i in range(4):
            self.joint_states[i * 2] = radians(positions[i])
            self.joint_states[i * 2 + 1] = (
                self.joint_states[i * 2] * 1.05851325 + 0.72349796
            )
        self.joint_states[-1] = radians(positions[-2])
        self.joint_states[-2] = radians(positions[-1])
        msg.position = self.joint_states
        self.safe_publish(self.js_pub, msg)

    def update_pos(self, position):
        msg = Float32MultiArray()
        msg.data = position
        self.safe_publish(self.pub_position_fb, msg)
        if self.js_publisher:
            self.publish_joint_states(positions=position)

    def update_vel(self, velocity):
        msg = Float32MultiArray()
        msg.data = velocity
        self.safe_publish(self.pub_velocity_fb, msg)

    def update_cur(self, current):
        msg = Float32MultiArray()
        msg.data = current
        self.safe_publish(self.pub_current_fb, msg)

    def update_fsr(self, fsr):
        msg = Float32MultiArray()
        msg.data = fsr
        self.safe_publish(self.pub_touch_fb, msg)

    def update_hot_cold(self, hot_cold):
        if hot_cold:
            msg = UInt16()
            msg.data = hot_cold
            self.safe_publish(self.pub_hot_cold_fb, msg)

    # Target Subscriber callbacks
    def velocity_callback(self, msg):
        self.client.set_velocity(
            list(msg.data), reply_mode=int(msg.reply_mode)
        )
        if not self.write_thread:
            self.client.send_command()

    def position_callback(self, msg):
        self.client.set_position(
            list(msg.data), reply_mode=int(msg.reply_mode)
        )
        if not self.write_thread:
            self.client.send_command()

    def current_callback(self, msg):
        self.client.set_torque(list(msg.data), reply_mode=int(msg.reply_mode))
        if not self.write_thread:
            self.client.send_command()

    def duty_callback(self, msg):
        self.client.set_duty(list(msg.data), reply_mode=int(msg.reply_mode))
        if not self.write_thread:
            self.client.send_command()


def main(args=None):
    rclpy.init(args=args)
    node = AbilityHandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.close()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
