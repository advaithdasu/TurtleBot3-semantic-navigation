#!/usr/bin/env python3
"""
semantic_query_node.py — Stage-4 semantic query ROS 2 node.

Resolves text commands against the current semantic memory state and
publishes a SemanticQueryResult with the selected target object.

Two resolution paths:
  * Plain commands ("go to person 2") — deterministic label/index match,
    nearest-first (unchanged Stage-4 behaviour).
  * Attribute commands ("go to the sofa with warm color") — the referring
    expression is grounded by a vision-language model (LocateAnything)
    against the best-view evidence frames stored by tb3_grounding's
    evidence_store_node; the candidate whose stored bbox the model's
    answer lands on wins. Degrades to the plain path if the grounding
    server or evidence is unavailable.

Subscribed topics
-----------------
  /semantic_map_memory_node/landmark_objects   vision_msgs/Detection3DArray
  ~/command                                    std_msgs/String

Published topics
----------------
  ~/selected_target                 tb3_query/SemanticQueryResult
  ~/query_status                    std_msgs/String   (human-readable debug)

Parameters  (see config/semantic_query.yaml)
----------
  memory_topic          str     Memory state topic
  command_topic         str     Input command topic
  output_topic          str     Selected target topic
  status_topic          str     Debug status topic
  semantic_targets_file str     Path to semantic_targets.yaml
  output_frame          str     Frame for output positions
  grounding_enabled     bool    Enable VLM attribute resolution
  grounding_server_url  str     grounding/server.py endpoint
  grounding_timeout_sec float   Per-request HTTP timeout
  evidence_dir          str     Evidence store (shared with evidence_store_node)
  min_grounding_score   float   Floor for accepting a grounded match
"""

from __future__ import annotations

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import Point

from ament_index_python.packages import get_package_share_directory

from tb3_query.query_core import (
    MemoryObject,
    ParsedCommand,
    QueryResult,
    load_target_mapping,
    parse_command,
    select_target,
)

try:
    from tb3_query.msg import SemanticQueryResult
except ImportError:
    SemanticQueryResult = None

# Grounding is optional: without tb3_grounding the node still runs and
# attribute queries degrade to nearest-first.
try:
    from tb3_grounding.evidence_core import EvidenceStore
    from tb3_grounding.grounding_client import GroundingClient, GroundingError
    from tb3_grounding.resolver_core import (
        EvidenceCandidate,
        pick_best,
        rank_candidates,
    )
    _GROUNDING_IMPORTED = True
except ImportError:
    _GROUNDING_IMPORTED = False


class SemanticQueryNode(Node):

    def __init__(self) -> None:
        super().__init__("semantic_query_node")

        if SemanticQueryResult is None:
            self.get_logger().fatal(
                "tb3_query/msg/SemanticQueryResult not available. "
                "Did you source install/setup.bash after building?"
            )
            raise RuntimeError("SemanticQueryResult message not found")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("memory_topic", "/semantic_memory_node/objects")
        self.declare_parameter("command_topic", "~/command")
        self.declare_parameter("output_topic", "~/selected_target")
        self.declare_parameter("status_topic", "~/query_status")
        self.declare_parameter("semantic_targets_file", "")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("grounding_enabled", True)
        self.declare_parameter("grounding_server_url", "http://127.0.0.1:8801")
        self.declare_parameter("grounding_timeout_sec", 20.0)
        self.declare_parameter("evidence_dir", "~/.tb3_semantic_nav/evidence")
        self.declare_parameter("min_grounding_score", 0.05)

        mem_topic    = self.get_parameter("memory_topic").value
        cmd_topic    = self.get_parameter("command_topic").value
        out_topic    = self.get_parameter("output_topic").value
        status_topic = self.get_parameter("status_topic").value
        targets_file = self.get_parameter("semantic_targets_file").value
        self._out_frame = self.get_parameter("output_frame").value

        self._grounding_enabled = (
            self.get_parameter("grounding_enabled").value and _GROUNDING_IMPORTED
        )
        self._min_ground_score = self.get_parameter("min_grounding_score").value
        if self._grounding_enabled:
            self._ground_client = GroundingClient(
                self.get_parameter("grounding_server_url").value,
                timeout_sec=self.get_parameter("grounding_timeout_sec").value,
            )
            self._evidence = EvidenceStore(
                self.get_parameter("evidence_dir").value)
            self.get_logger().info(
                "Grounding enabled  server=%s  evidence=%s"
                % (self._ground_client.base_url, str(self._evidence.root))
            )
        else:
            self._ground_client = None
            self._evidence = None
            if self.get_parameter("grounding_enabled").value:
                self.get_logger().warn(
                    "grounding_enabled=true but tb3_grounding is not importable; "
                    "attribute queries will degrade to nearest-first"
                )

        # ── Load semantic mapping ─────────────────────────────────────────
        if not targets_file:
            pkg_share = get_package_share_directory("tb3_frontier_exploration")
            targets_file = os.path.join(pkg_share, "config", "semantic_targets.yaml")

        self._sem2det, self._det2sem = load_target_mapping(targets_file)
        self._known_targets = set(self._sem2det.keys())
        self.get_logger().info(
            "Loaded %d semantic targets: %s"
            % (len(self._known_targets), sorted(self._known_targets))
        )

        # ── State: latest memory snapshot ─────────────────────────────────
        self._memory_objects: list[MemoryObject] = []

        # ── QoS ───────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            Detection3DArray, mem_topic, self._memory_cb, reliable_qos
        )
        self.create_subscription(
            String, cmd_topic, self._command_cb, reliable_qos
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._result_pub = self.create_publisher(
            SemanticQueryResult, out_topic, reliable_qos
        )
        self._status_pub = self.create_publisher(
            String, status_topic, reliable_qos
        )

        self.get_logger().info("SemanticQueryNode ready  frame=%s" % self._out_frame)

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _memory_cb(self, msg: Detection3DArray) -> None:
        objs: list[MemoryObject] = []
        for det in msg.detections:
            if not det.results:
                continue
            objs.append(MemoryObject(
                object_id=det.id,
                detector_label=det.results[0].hypothesis.class_id,
                x=det.bbox.center.position.x,
                y=det.bbox.center.position.y,
                confidence=det.results[0].hypothesis.score,
            ))
        self._memory_objects = objs

    def _command_cb(self, msg: String) -> None:
        raw = msg.data.strip()
        self.get_logger().info("Command received: '%s'" % raw)

        parsed = parse_command(raw, self._known_targets)

        if parsed is None:
            self._publish_failure(
                raw, "", "",
                f"unsupported semantic target in: '{raw}' "
                f"(known: {sorted(self._known_targets)})"
            )
            return

        semantic_name = parsed.semantic_name
        detector_label = self._sem2det.get(semantic_name, "")
        if not detector_label:
            self._publish_failure(
                raw, semantic_name, "",
                f"no detector_label mapping for '{semantic_name}'"
            )
            return

        if parsed.desired_index is not None:
            self.get_logger().info(
                "Parsed: semantic='%s'  desired_index=%d"
                % (semantic_name, parsed.desired_index)
            )

        # Attribute expressions go through VLM grounding; an explicit index
        # wins ("person 2 with red shirt" is an indexed lookup of person_2).
        result = None
        if parsed.attribute_expression is not None and parsed.desired_index is None:
            self.get_logger().info(
                "Attribute query: expression='%s'" % parsed.attribute_expression
            )
            result = self._resolve_attribute_query(
                parsed, semantic_name, detector_label, raw
            )

        if result is None:
            result = select_target(
                self._memory_objects,
                semantic_name,
                detector_label,
                raw,
                desired_index=parsed.desired_index,
            )

        out = SemanticQueryResult()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._out_frame
        out.success = result.success
        out.query_text = result.query_text
        out.semantic_name = result.semantic_name
        out.detector_label = result.detector_label
        out.object_id = result.object_id
        out.position = Point(x=result.x, y=result.y, z=0.0)
        out.frame_id = self._out_frame
        out.confidence = float(result.confidence)
        out.status_message = result.status_message

        self._result_pub.publish(out)

        status = String()
        status.data = result.status_message
        self._status_pub.publish(status)

        if result.success:
            self.get_logger().info(
                "Query OK: %s → %s at (%.2f, %.2f)"
                % (semantic_name, result.object_id, result.x, result.y)
            )
        else:
            self.get_logger().warn("Query FAILED: %s" % result.status_message)

    def _resolve_attribute_query(
        self,
        parsed: ParsedCommand,
        semantic_name: str,
        detector_label: str,
        raw: str,
    ) -> QueryResult | None:
        """Resolve an attribute expression via the grounding server.

        Returns a QueryResult (success, or a definitive "nothing matched"
        failure), or None when the grounding infrastructure is unavailable
        so the caller falls back to nearest-first.
        """
        expression = parsed.attribute_expression

        if not self._grounding_enabled:
            self.get_logger().warn(
                "grounding disabled — falling back to nearest-first for '%s'"
                % expression)
            return None

        candidates = [
            o for o in self._memory_objects
            if o.detector_label == detector_label
        ]
        if not candidates:
            return QueryResult(
                success=False,
                query_text=raw,
                semantic_name=semantic_name,
                detector_label=detector_label,
                status_message=f"no active {semantic_name} in memory",
            )
        if len(candidates) == 1:
            only = candidates[0]
            return QueryResult(
                success=True,
                query_text=raw,
                semantic_name=semantic_name,
                detector_label=detector_label,
                object_id=only.object_id,
                x=only.x,
                y=only.y,
                confidence=only.confidence,
                status_message=(
                    f"single {semantic_name} candidate {only.object_id}; "
                    f"grounding skipped for '{expression}'"
                ),
            )

        # Gather best-view evidence written by evidence_store_node.
        self._evidence.reload()
        ev_candidates = []
        missing: list[str] = []
        for o in candidates:
            rec = self._evidence.get(o.object_id)
            img = self._evidence.load_image(o.object_id) if rec else None
            if rec is None or img is None:
                missing.append(o.object_id)
                continue
            ev_candidates.append(
                EvidenceCandidate(o.object_id, rec.bbox_xyxy, img))
        if missing:
            self.get_logger().warn(
                "no stored evidence for %s" % missing)
        if not ev_candidates:
            self.get_logger().warn(
                "no evidence for any %s candidate — falling back to "
                "nearest-first" % semantic_name)
            return None

        try:
            ranked = rank_candidates(
                ev_candidates, expression, self._ground_client.ground)
        except GroundingError as exc:
            self.get_logger().warn(
                "grounding failed (%s) — falling back to nearest-first" % exc)
            return None

        detail = ", ".join(
            "%s=%.2f" % (r.object_id, r.score) for r in ranked)
        best = pick_best(ranked, self._min_ground_score)

        if best is None:
            # The model looked at every candidate and none matched the
            # expression — navigating to the nearest one would be wrong.
            return QueryResult(
                success=False,
                query_text=raw,
                semantic_name=semantic_name,
                detector_label=detector_label,
                status_message=(
                    f"no {semantic_name} matched '{expression}' "
                    f"(scores: {detail})"
                ),
            )

        obj = next(o for o in candidates if o.object_id == best.object_id)
        return QueryResult(
            success=True,
            query_text=raw,
            semantic_name=semantic_name,
            detector_label=detector_label,
            object_id=obj.object_id,
            x=obj.x,
            y=obj.y,
            confidence=obj.confidence,
            status_message=(
                f"grounded '{expression}' → {obj.object_id} "
                f"(score={best.score:.2f}, iou={best.best_iou:.2f}; {detail})"
            ),
        )

    def _publish_failure(
        self, raw: str, semantic_name: str, detector_label: str, msg: str
    ) -> None:
        out = SemanticQueryResult()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._out_frame
        out.success = False
        out.query_text = raw
        out.semantic_name = semantic_name
        out.detector_label = detector_label
        out.status_message = msg
        self._result_pub.publish(out)

        status = String()
        status.data = msg
        self._status_pub.publish(status)
        self.get_logger().warn("Query FAILED: %s" % msg)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticQueryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
