#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, TransformStamped, Pose
import tf2_ros
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener, LookupException
from rclpy.duration import Duration

from manus_ros2_msgs.msg import ManusGlove, ManusRawNode
from ah_messages.msg import Digits

from scipy.spatial.transform import Rotation as R
import numpy as np

from ah_ros_py.kd_tree_lookup import (
    KDLookup,
)


class PoseToTFBroadcaster(Node):
    def __init__(self):
        super().__init__("pose_to_tf_broadcaster")

        # Create a subscriber to /manus_glove_0 and /manus_glove_1
        self.subscription = self.create_subscription(
            ManusGlove, "/manus_glove_0", self.pose_callback, 10
        )

        self.subscription2 = self.create_subscription(
            ManusGlove, "/manus_glove_1", self.pose_callback, 10
        )

        # Target position publisher
        self.pub_position = self.create_publisher(
            Digits, "/ability_hand/target/position", 10
        )

        # TF Broadcaster
        self.br = tf2_ros.TransformBroadcaster(self)

        # Frame ID for all transforms
        self.parent_frame = "world"

        # Manus glove offsets
        self.manus_offsets = (0, -0.055, 0.072)

        # KD Tree for retargeting
        self.kd_tree_lookup_r = KDLookup(
            side="right", file_prefix=get_package_share_directory("ah_ros_py")
        )

    def pose_callback(self, msg: ManusGlove):
        side = str(msg.side)

        digits = Digits()
        digits.reply_mode = 0

        finger_positions = []
        t_j1 = 0.0
        t_j2 = 0.0
        for i, pose in enumerate(msg.raw_sensor):

            """Manus offset from field generator to wrist base"""
            manus_offset = TransformStamped()
            manus_offset.header.stamp = self.get_clock().now().to_msg()
            manus_offset.header.frame_id = self.parent_frame
            manus_offset.child_frame_id = "manus_glove_base"
            manus_offset.transform.translation.z = self.manus_offsets[2]
            manus_offset.transform.translation.y = self.manus_offsets[1]
            self.br.sendTransform(manus_offset)

            """Mirror (Manus mirrors for some reason)"""
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "manus_glove_base"
            t.child_frame_id = f"{side}_point_{i}"

            # Position
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = -pose.position.y
            t.transform.translation.z = pose.position.z

            rot = R.from_quat(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            )

            mirror_z = np.diag([1, -1, 1])  # 3x3 reflection matrix
            mirrored_rot_matrix = mirror_z @ rot.as_matrix() @ mirror_z
            mirrored_rot = R.from_matrix(mirrored_rot_matrix)
            mirrored_quat = mirrored_rot.as_quat()

            # Orientation
            t.transform.rotation.x = mirrored_quat[0]
            t.transform.rotation.y = mirrored_quat[1]
            t.transform.rotation.z = mirrored_quat[2]
            t.transform.rotation.w = mirrored_quat[3]

            # self.br.sendTransform(t)

            """Transform to move from nail to anchor point"""

            # Convert mirrored finger transform to a homogenous
            T = np.eye(4)
            T[:3, :3] = mirrored_rot_matrix
            T[:3, 3] = [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ]

            # Translate 20mm about the y-axis
            T2 = np.eye(4)
            T2[1][3] = 0.0325

            # If pinky translate 35 mm up the z axis
            if i == 4:
                TP = np.eye(4)
                TP[2][3] = 0.0325
                T3 = T @ TP @ T2
            else:
                T3 = T @ T2

            # nail_offset = TransformStamped()
            # nail_offset.header.stamp = self.get_clock().now().to_msg()
            # nail_offset.header.frame_id = f"manus_glove_base"
            # nail_offset.child_frame_id = f"{side}_anchor_{i}"
            # nail_offset.transform.translation.x = T3[0, 3]
            # nail_offset.transform.translation.y = T3[1, 3]
            # nail_offset.transform.translation.z = T3[2, 3]
            # self.br.sendTransform(nail_offset)

            # Get nail offset in world frame
            TW = np.eye(4)
            TM = np.eye(4)
            TM[:3, 3] = [
                self.manus_offsets[0],
                self.manus_offsets[1],
                self.manus_offsets[2],
            ]

            TN = TW @ TM @ T3
            nail_offset_world = TransformStamped()
            nail_offset_world.header.stamp = self.get_clock().now().to_msg()
            nail_offset_world.header.frame_id = "world"
            nail_offset_world.child_frame_id = f"{side}_anchor_world{i}"
            nail_offset_world.transform.translation.x = TN[0, 3]
            nail_offset_world.transform.translation.y = TN[1, 3]
            nail_offset_world.transform.translation.z = TN[2, 3]
            self.br.sendTransform(nail_offset_world)

            if i == 0:
                if side.lower() == "right":
                    t_j1, t_j2 = self.kd_tree_lookup_r.lookup_thumb(
                        position=[
                            nail_offset_world.transform.translation.x,
                            nail_offset_world.transform.translation.y,
                            max(
                                nail_offset_world.transform.translation.z,
                                0.0915,
                            ),  # Prevents weird configurations
                        ]
                    )
                    t_j1 = min(100.0, t_j1)
                    t_j2 = min(100.0, t_j2)

            if i in range(1, 5):
                if side.lower() == "right":
                    pos = self.kd_tree_lookup_r.lookup_finger(
                        finger=i - 1,
                        position=[
                            nail_offset_world.transform.translation.x,
                            nail_offset_world.transform.translation.y,
                            nail_offset_world.transform.translation.z,
                        ],
                    )
                    finger_positions.append(float(pos))

        finger_positions.append(t_j2)
        finger_positions.append(-t_j1)
        digits.data = finger_positions
        self.pub_position.publish(digits)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
