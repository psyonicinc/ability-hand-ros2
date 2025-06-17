from math import radians
import pickle

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

from ah_messages.msg import Digits
from ah_ros_py.kd_tree_lookup import FINGER_INCREMENT
import time


class MapFingerPointsNode(Node):
    def __init__(self):
        super().__init__("ability_hand_map_finger_node")
        self.declare_parameter("hand_side", "right")
        self.declare_parameter("create_text_map", False)

        # Read Parameters
        self.hand_side = (
            self.get_parameter("hand_side")
            .get_parameter_value()
            .string_value.lower()
        )
        self.create_text_map = (
            self.get_parameter("create_text_map")
            .get_parameter_value()
            .bool_value
        )

        # Target position publisher
        self.pub_position = self.create_publisher(
            Digits, "/ability_hand/target/position", 10
        )
        self.position = 0
        self.frames = (
            "index_anchor",
            "middle_anchor",
            "ring_anchor",
            "pinky_anchor",
        )
        self.locations = [[], [], [], []]
        self.init_hand = True  # Init hand to zeros in timer

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        if self.init_hand:
            msg = Digits()
            msg.reply_mode = 0
            msg.data = [0.0] * 6
            self.pub_position.publish(msg)
            self.init_hand = False
            rclpy.sleep(10)

        if self.position > 100:
            with open(f"all_fingers_{self.hand_side}.pkl", "wb") as f:
                pickle.dump(self.locations, f)
            self.get_logger().info("Done creating finger map")
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()

        for i in range(len(self.frames)):
            transform = self.tf_buffer.lookup_transform(
                "world", self.frames[i], Time()  # Parent frame  # Child frame
            )

            if self.create_text_map:
                with open("finger_mapping_right.txt", "a") as f:
                    f.write(
                        f"{radians(self.position)} {transform.transform.translation.x} {transform.transform.translation.y} {transform.transform.translation.z}\n"
                    )
            self.locations[i].append(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ]
            )

        self.position += FINGER_INCREMENT
        msg = Digits()
        msg.reply_mode = 0
        msg.data = [
            float(self.position),
            float(self.position),
            float(self.position),
            float(self.position),
            0.0,
            0.0,
        ]
        self.pub_position.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapFingerPointsNode()
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
