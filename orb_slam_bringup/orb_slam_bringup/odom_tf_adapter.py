"""Replay-only odometry -> TF adapter (odom -> base_link).

Enabled only when launch arg publish_odom_tf:=true. Live profiles default false.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def is_finite_pose(pose: Pose) -> bool:
    """Return True if position and quaternion components are all finite."""
    p = pose.position
    q = pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    return all(math.isfinite(v) for v in values)


def build_odom_transform(
    msg: Odometry,
    *,
    expected_frame: str = "odom",
    expected_child: str = "base_link",
) -> Optional[TransformStamped]:
    """Build odom->base_link TransformStamped from Odometry, or None if invalid.

    Validates header.frame_id / child_frame_id and finite pose. Pose is copied
    unchanged; stamp is taken from the original message.
    """
    if msg.header.frame_id != expected_frame:
        return None
    if msg.child_frame_id != expected_child:
        return None
    if not is_finite_pose(msg.pose.pose):
        return None

    tf = TransformStamped()
    tf.header = msg.header
    tf.child_frame_id = msg.child_frame_id
    tf.transform.translation.x = msg.pose.pose.position.x
    tf.transform.translation.y = msg.pose.pose.position.y
    tf.transform.translation.z = msg.pose.pose.position.z
    tf.transform.rotation = msg.pose.pose.orientation
    return tf


class OdomTfAdapter(Node):
    """Subscribe to nav_msgs/Odometry and broadcast pose as TF."""

    def __init__(self) -> None:
        super().__init__("odom_tf_adapter")
        self.declare_parameter("odom_topic", "/odom_wheel")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        self.expected_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.expected_child = self.get_parameter("base_frame").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

        self.invalid_message_count = 0
        self._broadcaster = TransformBroadcaster(self)
        self._sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 50)
        self.get_logger().info(
            f"OdomTfAdapter: {odom_topic} -> TF {self.expected_frame}->{self.expected_child}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        tf = build_odom_transform(
            msg,
            expected_frame=self.expected_frame,
            expected_child=self.expected_child,
        )
        if tf is None:
            self.invalid_message_count += 1
            return
        self._broadcaster.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
