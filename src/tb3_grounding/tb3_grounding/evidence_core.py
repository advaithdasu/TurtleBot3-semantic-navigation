"""Best-view evidence store.

Semantic memory only keeps (label, x, y); the camera frames are gone by
query time. This store retains one best-view image per remembered object
so a VLM can reason about appearance later.

Layout under root: index.json plus one image file per object. Writes go
through temp file + os.replace so a concurrent reader never sees a torn
index. No ROS/cv2 dependency; images are passed as encoded bytes.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional


@dataclass
class EvidenceRecord:
    object_id: str
    detector_label: str
    bbox_xyxy: list          # in the stored frame, pixels
    confidence: float
    image_width: int
    image_height: int
    stamp: float
    view_score: float
    image_file: str          # relative to the store root


def view_score(
    bbox_xyxy: list,
    image_width: int,
    image_height: int,
    confidence: float,
    edge_margin_px: float = 3.0,
    edge_penalty: float = 0.5,
) -> float:
    """Quality of an observation as grounding evidence: confidence scaled
    by sqrt(bbox area fraction), halved when the box touches the image
    border (a truncated object is poor evidence for appearance)."""
    if image_width <= 0 or image_height <= 0:
        return 0.0
    x1, y1, x2, y2 = bbox_xyxy
    w = max(0.0, x2 - x1)
    h = max(0.0, y2 - y1)
    area_frac = min(1.0, (w * h) / float(image_width * image_height))

    touches_edge = (
        x1 <= edge_margin_px
        or y1 <= edge_margin_px
        or x2 >= image_width - edge_margin_px
        or y2 >= image_height - edge_margin_px
    )
    factor = edge_penalty if touches_edge else 1.0
    return float(confidence) * math.sqrt(area_frac) * factor


class EvidenceStore:

    def __init__(self, root_dir: str | Path, min_improvement: float = 0.02) -> None:
        self.root = Path(root_dir).expanduser()
        self.root.mkdir(parents=True, exist_ok=True)
        self.min_improvement = min_improvement
        self._index_path = self.root / "index.json"
        self._records: dict[str, EvidenceRecord] = {}
        self.reload()

    def reload(self) -> None:
        """Re-read the on-disk index (for cross-process readers)."""
        self._records = {}
        if not self._index_path.is_file():
            return
        try:
            with open(self._index_path, "r") as f:
                raw = json.load(f)
        except (json.JSONDecodeError, OSError):
            return
        for oid, fields in raw.items():
            try:
                self._records[oid] = EvidenceRecord(**fields)
            except TypeError:
                continue  # record from an incompatible version

    def get(self, object_id: str) -> Optional[EvidenceRecord]:
        return self._records.get(object_id)

    def load_image(self, object_id: str) -> Optional[bytes]:
        rec = self._records.get(object_id)
        if rec is None:
            return None
        try:
            return (self.root / rec.image_file).read_bytes()
        except OSError:
            return None

    def object_ids(self) -> list[str]:
        return sorted(self._records.keys())

    def consider(
        self,
        object_id: str,
        detector_label: str,
        bbox_xyxy: list,
        confidence: float,
        image_width: int,
        image_height: int,
        image_bytes: bytes,
        stamp: float,
    ) -> bool:
        """Store this observation iff it beats the current best view."""
        score = view_score(bbox_xyxy, image_width, image_height, confidence)
        existing = self._records.get(object_id)
        if existing is not None and score < existing.view_score + self.min_improvement:
            return False

        image_file = f"{_safe_name(object_id)}.jpg"
        rec = EvidenceRecord(
            object_id=object_id,
            detector_label=detector_label,
            bbox_xyxy=[float(v) for v in bbox_xyxy],
            confidence=float(confidence),
            image_width=int(image_width),
            image_height=int(image_height),
            stamp=float(stamp),
            view_score=score,
            image_file=image_file,
        )

        self._atomic_write_bytes(self.root / image_file, image_bytes)
        self._records[object_id] = rec
        self._flush_index()
        return True

    def _flush_index(self) -> None:
        payload = {oid: asdict(rec) for oid, rec in self._records.items()}
        self._atomic_write_bytes(self._index_path, json.dumps(payload, indent=2).encode())

    def _atomic_write_bytes(self, path: Path, data: bytes) -> None:
        fd, tmp = tempfile.mkstemp(dir=str(self.root), prefix=".tmp_")
        try:
            with os.fdopen(fd, "wb") as f:
                f.write(data)
            os.replace(tmp, str(path))
        except OSError:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise


def _safe_name(object_id: str) -> str:
    # Object ids can contain spaces ("stop sign_0").
    return "".join(c if (c.isalnum() or c in "-_") else "_" for c in object_id)
