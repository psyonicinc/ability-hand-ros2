import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ah_messages.msg import Digits

from ah_wrapper import AHSerialClient
from ah_wrapper.observer import Observer


class AbilityHandNode(Node, Observer):
    def __init__(self):
        super().__init__("ability_hand_node")
        self.declare_parameter("write_thread", True)
        self.write_thread = (
            self.get_parameter("write_thread").get_parameter_value().bool_value
        )
        # Ability Hand Wrapper Client
        self.client = AHSerialClient(write_thread=self.write_thread)
        self.client.hand.add_observer(self)

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

    def safe_publish(self, pub, data):
        # Call publish outside of client class
        if rclpy.ok():
            msg = Float32MultiArray()
            msg.data = data
            try:
                pub.publish(msg)
            except Exception as e:
                print(e)
                self.get_logger().warn(f"Failed to publish: {e}")
        else:
            self.get_logger().warn("ROS context is invalid")

    def update_pos(self, position):
        self.safe_publish(self.pub_position_fb, position)

    def update_vel(self, velocity):
        self.safe_publish(self.pub_velocity_fb, velocity)

    def update_cur(self, current):
        self.safe_publish(self.pub_current_fb, current)

    def update_fsr(self, fsr):
        self.safe_publish(self.pub_touch_fb, fsr)

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
