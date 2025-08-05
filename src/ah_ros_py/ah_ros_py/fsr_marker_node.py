import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from visualization_msgs.msg import Marker, MarkerArray

fsr_map = {
    0: "fsr4",
    1: "fsr2",
    2: "fsr5",
    3: "fsr1",
    4: "fsr0",
    5: "fsr3",
    6: "fsr10",
    7: "fsr8",
    8: "fsr11",
    9: "fsr7",
    10: "fsr6",
    11: "fsr9",
    12: "fsr16",
    13: "fsr14",
    14: "fsr17",
    15: "fsr13",
    16: "fsr12",
    17: "fsr15",
    18: "fsr22",
    19: "fsr20",
    20: "fsr23",
    21: "fsr19",
    22: "fsr18",
    23: "fsr21",
    24: "fsr28",
    25: "fsr26",
    26: "fsr29",
    27: "fsr25",
    28: "fsr24",
    29: "fsr27",
}

class MarkerNode(Node):
    def __init__(self):
        super().__init__("fsr_marker_node")
        self.touch_data = [0] * 30

        self.marker_pub = self.create_publisher(MarkerArray, 'touch_markers', 10)

        self.fsr_sub = self.create_subscription(
            Float32MultiArray, "/ability_hand/right/feedback/touch", self.fsr_callback, 10
        )

    def fsr_callback(self, msg):
        self.touch_data = msg.data
        self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()
        for i in range(len(self.touch_data)):
            arrow = Marker()
            arrow.header.frame_id = fsr_map[i]
            arrow.header.stamp = rclpy.time.Time().to_msg()

            # Make arrow move out Z direction
            arrow.pose.orientation.x = 0.0
            arrow.pose.orientation.y = -0.7071
            arrow.pose.orientation.z = 0.0
            arrow.pose.orientation.w = 0.7071

            arrow.ns = "touch"
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            
            arrow.scale.x = 0.0025 + 0.01 * self.touch_data[i]  # shaft length
            arrow.scale.y = 0.0025  # shaft diameter
            arrow.scale.z = 0.0025  # head diameter

            arrow.color.r = min(1.0, self.touch_data[i] / 5.0)
            arrow.color.g = 1.0 - arrow.color.r
            arrow.color.b = 0.0
            arrow.color.a = 1.0

            marker_array.markers.append(arrow)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
