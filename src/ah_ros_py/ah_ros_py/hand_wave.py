import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
from math import pi, sin

from ah_messages.msg import Digits


class HandWaveNode(Node):
    def __init__(self):
        super().__init__("ability_hand_node")
        self.declare_parameter("hand_side", "Right")
        hand_side = (
            self.get_parameter("hand_side")
            .get_parameter_value()
            .string_value.lower()
        )

        # Target position publisher
        if hand_side.lower() == "right":
            self.pub_position = self.create_publisher(
                Digits, "/ability_hand/right/target/position", 10
            )
        elif hand_side.lower() == "left":
            self.pub_position = self.create_publisher(
                Digits, "/ability_hand/left/target/position", 10
            )

        self.position_data = [30, 30, 30, 30, 30, -30]

        timer_period = 0.002  # seconds
        self.timer = self.create_timer(timer_period, self.publish_position)
        self.msg = Digits()
        self.msg.reply_mode = 0

    def publish_position(self):
        current_time = time.time()
        for i in range(0, len(self.position_data)):
            ft = current_time * 3 + i * (2 * pi) / 12
            self.position_data[i] = (0.5 * sin(ft) + 0.5) * 45 + 15
        self.position_data[5] = -self.position_data[5]
        self.msg.data = self.position_data
        self.pub_position.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandWaveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
