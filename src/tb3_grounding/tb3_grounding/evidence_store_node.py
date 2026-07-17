#!/usr/bin/env python3
"""Best-view evidence collector.

Keeps, for every persistent semantic landmark, the best camera frame in
which it was observed, so semantic_query_node can run VLM grounding over
them for attribute queries ("go to the sofa with warm color").

Per Detection3DArray from the localizer: join the raw frame and the
Detection2DArray of the same header stamp, pair each 3D detection with
its source 2D bbox by label + bearing, transform the position into the
map frame, and match it to the nearest same-class landmark. Landmarks
only exist after several promoting observations, so unmatched
observations are buffered and retried when the next landmark snapshot
arrives.

Subscribes: /camera/image_raw, /detector_node/detections,
/localizer_node/localized_objects,
/semantic_map_memory_node/landmark_objects.
Writes <evidence_dir>/index.json plus one JPEG per landmark.
See config/grounding.yaml for parameters.
"""

from __future__ import annotations

import math
import time as _time
from collections import OrderedDict, deque
from dataclasses import dataclass

import cv2

import rclpy
import rclpy.time
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2DArray, Detection3DArray

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform)

try:
    from cv_bridge import CvBridge
except ImportError as e:
    raise ImportError(
        "cv_bridge not found. Install ros-humble-cv-bridge.\n"
        f"Original error: {e}"
    )

from tb3_localizer.localizer_core import LocalizerCore
from tb3_grounding.evidence_core import EvidenceStore


@dataclass
class _PendingObservation:
    """An observation whose landmark hasn't been promoted yet."""

    detector_label: str
    bbox_xyxy: list
    confidence: float
    map_x: float
    map_y: float
    image_width: int
    image_height: int
    jpeg_bytes: bytes
    stamp_sec: float
    wall_time: float


class EvidenceStoreNode(Node):

    def __init__(self) -> None:
        super().__init__("evidence_store_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/detector_node/detections")
        self.declare_parameter("localized_topic", "/localizer_node/localized_objects")
        self.declare_parameter("landmarks_topic", "/semantic_map_memory_node/landmark_objects")
        self.declare_parameter("evidence_dir", "~/.tb3_semantic_nav/evidence")
        self.declare_parameter("camera_hfov_deg", 62.2)
        self.declare_parameter("bearing_tolerance_rad", 0.06)
        self.declare_parameter("match_distance", 0.8)
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tf_timeout", 0.3)
        self.declare_parameter("frame_buffer_size", 40)
        self.declare_parameter("pending_ttl_sec", 60.0)
        self.declare_parameter("pending_max", 200)
        self.declare_parameter("min_confidence", 0.12)
        self.declare_parameter("process_min_interval_sec", 0.15)
        self.declare_parameter("jpeg_quality", 85)

        image_topic     = self.get_parameter("image_topic").value
        det_topic       = self.get_parameter("detections_topic").value
        loc_topic       = self.get_parameter("localized_topic").value
        lm_topic        = self.get_parameter("landmarks_topic").value
        evidence_dir    = self.get_parameter("evidence_dir").value
        hfov_deg        = self.get_parameter("camera_hfov_deg").value
        self._bear_tol  = self.get_parameter("bearing_tolerance_rad").value
        self._match_d   = self.get_parameter("match_distance").value
        self._frame     = self.get_parameter("target_frame").value
        self._tf_tout   = self.get_parameter("tf_timeout").value
        buf_size        = self.get_parameter("frame_buffer_size").value
        self._pend_ttl  = self.get_parameter("pending_ttl_sec").value
        pending_max     = self.get_parameter("pending_max").value
        self._min_conf  = self.get_parameter("min_confidence").value
        self._min_dt    = self.get_parameter("process_min_interval_sec").value
        self._jpeg_q    = int(self.get_parameter("jpeg_quality").value)

        self._loc_core = LocalizerCore(camera_hfov_rad=math.radians(hfov_deg))
        self._store = EvidenceStore(evidence_dir)
        self._bridge = CvBridge()

        self._buf_size = int(buf_size)
        self._frames: OrderedDict[tuple, object] = OrderedDict()      # stamp -> bgr
        self._det2d: OrderedDict[tuple, list] = OrderedDict()         # stamp -> dets
        self._landmarks: dict[str, tuple[str, float, float]] = {}     # id -> (label, x, y)
        self._pending: deque[_PendingObservation] = deque(maxlen=int(pending_max))
        self._last_process = 0.0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Image, image_topic, self._image_cb, sensor_qos)
        self.create_subscription(
            Detection2DArray, det_topic, self._detections_cb, reliable_qos)
        self.create_subscription(
            Detection3DArray, loc_topic, self._localized_cb, reliable_qos)
        self.create_subscription(
            Detection3DArray, lm_topic, self._landmarks_cb, reliable_qos)

        self.get_logger().info(
            "EvidenceStoreNode ready  dir=%s  hfov=%.1f°  match=%.2fm  "
            "existing evidence: %s"
            % (str(self._store.root), hfov_deg, self._match_d,
               self._store.object_ids() or "none")
        )

    @staticmethod
    def _key(stamp) -> tuple:
        return (stamp.sec, stamp.nanosec)

    def _image_cb(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning("cv_bridge conversion failed: %s" % e)
            return
        self._frames[self._key(msg.header.stamp)] = bgr
        while len(self._frames) > self._buf_size:
            self._frames.popitem(last=False)

    def _detections_cb(self, msg: Detection2DArray) -> None:
        dets = []
        for det in msg.detections:
            if not det.results:
                continue
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y
            dets.append({
                "label": det.results[0].hypothesis.class_id,
                "conf": det.results[0].hypothesis.score,
                "bbox_xyxy": [cx - w / 2.0, cy - h / 2.0,
                              cx + w / 2.0, cy + h / 2.0],
                "center_u": cx,
            })
        self._det2d[self._key(msg.header.stamp)] = dets
        while len(self._det2d) > self._buf_size:
            self._det2d.popitem(last=False)

    def _localized_cb(self, msg: Detection3DArray) -> None:
        now = _time.monotonic()
        if now - self._last_process < self._min_dt:
            return
        self._last_process = now

        key = self._key(msg.header.stamp)
        frame = self._frames.get(key)
        dets2d = self._det2d.get(key)
        if frame is None or not dets2d:
            return

        img_h, img_w = frame.shape[:2]
        source_frame = msg.header.frame_id or "base_link"
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        jpeg_bytes: bytes | None = None   # encoded lazily, once per frame
        pool = list(dets2d)

        for det in msg.detections:
            if not det.results:
                continue
            label = det.results[0].hypothesis.class_id
            conf = det.results[0].hypothesis.score
            if conf < self._min_conf:
                continue
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y

            match2d = self._pair_detection(pool, label, x, y, img_w)
            if match2d is None:
                continue
            pool.remove(match2d)

            map_pt = self._to_map(source_frame, x, y)
            if map_pt is None:
                continue

            if jpeg_bytes is None:
                ok, buf = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q])
                if not ok:
                    return
                jpeg_bytes = buf.tobytes()

            obs = _PendingObservation(
                detector_label=label,
                bbox_xyxy=match2d["bbox_xyxy"],
                confidence=conf,
                map_x=map_pt[0],
                map_y=map_pt[1],
                image_width=img_w,
                image_height=img_h,
                jpeg_bytes=jpeg_bytes,
                stamp_sec=stamp_sec,
                wall_time=_time.time(),
            )

            if not self._try_store(obs):
                self._pending.append(obs)

    def _pair_detection(
        self, pool: list[dict], label: str, x: float, y: float, img_w: int
    ) -> dict | None:
        """Find the 2D detection this 3D detection came from. The localizer
        derived its bearing from the same bbox center pixel, so a true
        pair differs only by float noise."""
        target_bearing = math.atan2(y, x)
        best = None
        best_diff = self._bear_tol
        for det in pool:
            if det["label"] != label:
                continue
            bearing = self._loc_core.pixel_to_bearing(det["center_u"], img_w)
            diff = abs(bearing - target_bearing)
            if diff < best_diff:
                best = det
                best_diff = diff
        return best

    def _to_map(self, source_frame: str, x: float, y: float) -> tuple | None:
        pt = PointStamped()
        pt.header.stamp = rclpy.time.Time(seconds=0).to_msg()
        pt.header.frame_id = source_frame
        pt.point.x = x
        pt.point.y = y
        try:
            out = self._tf_buffer.transform(
                pt, self._frame, timeout=Duration(seconds=self._tf_tout))
        except Exception:
            return None
        return out.point.x, out.point.y

    def _landmarks_cb(self, msg: Detection3DArray) -> None:
        landmarks: dict[str, tuple[str, float, float]] = {}
        for det in msg.detections:
            if not det.results or not det.id:
                continue
            landmarks[det.id] = (
                det.results[0].hypothesis.class_id,
                det.bbox.center.position.x,
                det.bbox.center.position.y,
            )
        self._landmarks = landmarks
        self._retry_pending()

    def _match_landmark(self, label: str, mx: float, my: float) -> str | None:
        best_id = None
        best_d = self._match_d
        for lid, (lm_label, lx, ly) in self._landmarks.items():
            if lm_label != label:
                continue
            d = math.hypot(lx - mx, ly - my)
            if d < best_d:
                best_id = lid
                best_d = d
        return best_id

    def _try_store(self, obs: _PendingObservation) -> bool:
        """Returns True when the observation matched a landmark (whether
        or not it replaced the stored view); False keeps it pending."""
        lid = self._match_landmark(obs.detector_label, obs.map_x, obs.map_y)
        if lid is None:
            return False
        stored = self._store.consider(
            object_id=lid,
            detector_label=obs.detector_label,
            bbox_xyxy=obs.bbox_xyxy,
            confidence=obs.confidence,
            image_width=obs.image_width,
            image_height=obs.image_height,
            image_bytes=obs.jpeg_bytes,
            stamp=obs.stamp_sec,
        )
        if stored:
            rec = self._store.get(lid)
            self.get_logger().info(
                "[evidence] new best view for %s  score=%.3f  conf=%.2f  "
                "bbox=%s" % (lid, rec.view_score, obs.confidence,
                             [int(v) for v in obs.bbox_xyxy])
            )
        return True

    def _retry_pending(self) -> None:
        # A landmark may be promoted after its best frames were seen, so
        # buffered observations get another chance on each snapshot.
        if not self._pending:
            return
        now = _time.time()
        keep: deque[_PendingObservation] = deque(maxlen=self._pending.maxlen)
        for obs in self._pending:
            if now - obs.wall_time > self._pend_ttl:
                continue
            if not self._try_store(obs):
                keep.append(obs)
        self._pending = keep


def main(args=None):
    rclpy.init(args=args)
    node = EvidenceStoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
