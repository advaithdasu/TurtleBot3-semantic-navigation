#!/usr/bin/env python3
"""
semantic_memory_marker_node.py — Visualize semantic memory objects in RViz.

Subscribes to the semantic memory output (Detection3DArray in base_link),
transforms each object into map frame via TF, and publishes a MarkerArray
with a sphere + text label per active object.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration

from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PointStamped
from std_msgs.msg import ColorRGBA

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

CLASS_COLORS = {
    "person":    ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.9),
    "bench":     ColorRGBA(r=0.8, g=0.6, b=0.2, a=0.9),
    "stop sign": ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.9),
}
DEFAULT_COLOR = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.9)


class SemanticMemoryMarkerNode(Node):

    def __init__(self) -> None:
        super().__init__("semantic_memory_marker_node")

        self.declare_parameter("input_topic", "/semantic_memory_node/objects")
        self.declare_parameter("output_topic", "/semantic_memory_markers")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tf_timeout", 0.3)
        self.declare_parameter("sphere_radius", 0.18)
        self.declare_parameter("text_offset_z", 0.45)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self._target_frame = self.get_parameter("target_frame").value
        self._tf_timeout = self.get_parameter("tf_timeout").value
        self._sphere_r = self.get_parameter("sphere_radius").value
        self._text_z = self.get_parameter("text_offset_z").value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(Detection3DArray, in_topic, self._cb, qos)
        self._pub = self.create_publisher(MarkerArray, out_topic, qos)

        self._prev_count = 0
        self.get_logger().info(
            "SemanticMemoryMarkerNode ready  %s -> %s  frame=%s"
            % (in_topic, out_topic, self._target_frame))

    def _cb(self, msg: Detection3DArray) -> None:
        ma = MarkerArray()
        marker_id = 0

        for det in msg.detections:
            if not det.results:
                continue

            label = det.results[0].hypothesis.class_id
            obj_id = det.id or label
            source_frame = det.header.frame_id or msg.header.frame_id or "base_link"

            pt_in = PointStamped()
            pt_in.header.stamp = msg.header.stamp
            pt_in.header.frame_id = source_frame
            pt_in.point.x = det.bbox.center.position.x
            pt_in.point.y = det.bbox.center.position.y
            pt_in.point.z = 0.0

            try:
                pt_out = self._tf_buffer.transform(
                    pt_in, self._target_frame,
                    timeout=Duration(seconds=self._tf_timeout))
            except Exception as e:
                self.get_logger().debug(
                    "TF %s->%s failed for %s: %s"
                    % (source_frame, self._target_frame, obj_id, e))
                continue

            color = CLASS_COLORS.get(label, DEFAULT_COLOR)
            stamp = self.get_clock().now().to_msg()

            sphere = Marker()
            sphere.header.stamp = stamp
            sphere.header.frame_id = self._target_frame
            sphere.ns = "semantic_memory"
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = pt_out.point
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = self._sphere_r * 2
            sphere.scale.y = self._sphere_r * 2
            sphere.scale.z = self._sphere_r * 2
            sphere.color = color
            sphere.lifetime = Duration(seconds=2.0).to_msg()
            ma.markers.append(sphere)
            marker_id += 1

            text = Marker()
            text.header.stamp = stamp
            text.header.frame_id = self._target_frame
            text.ns = "semantic_memory_text"
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pt_out.point.x
            text.pose.position.y = pt_out.point.y
            text.pose.position.z = pt_out.point.z + self._text_z
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = "%s (%s)" % (obj_id, label)
            text.lifetime = Duration(seconds=2.0).to_msg()
            ma.markers.append(text)
            marker_id += 1

        if marker_id < self._prev_count * 2:
            for i in range(marker_id, self._prev_count * 2):
                d = Marker()
                d.header.stamp = self.get_clock().now().to_msg()
                d.header.frame_id = self._target_frame
                d.ns = "semantic_memory" if i % 2 == 0 else "semantic_memory_text"
                d.id = i
                d.action = Marker.DELETE
                ma.markers.append(d)

        self._prev_count = marker_id // 2 if marker_id > 0 else 0
        self._pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMemoryMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
