"""Parse LocateAnything box tokens into pixel-space boxes.

The model emits boxes as `<box><x1><y1><x2><y2></box>` with coordinates
normalized to [0, 1000]; see https://huggingface.co/nvidia/LocateAnything-3B.
Importable without torch.
"""

from __future__ import annotations

import re

_BOX_RE = re.compile(r"<box><(\d+)><(\d+)><(\d+)><(\d+)></box>")


def parse_boxes(text: str, image_width: int, image_height: int) -> list[dict]:
    """Return [{"bbox_xyxy": [x1, y1, x2, y2], "score": 1.0, "label": ""}, ...].

    The model emits no per-box confidence, so score is a constant 1.0;
    downstream ranking should rely on IoU against a reference box instead.
    Coordinates are clamped to the image; degenerate boxes are dropped.
    """
    boxes = []
    for m in _BOX_RE.finditer(text or ""):
        x1, y1, x2, y2 = (int(g) / 1000.0 for g in m.groups())
        px1 = _clamp(x1 * image_width, 0, image_width)
        py1 = _clamp(y1 * image_height, 0, image_height)
        px2 = _clamp(x2 * image_width, 0, image_width)
        py2 = _clamp(y2 * image_height, 0, image_height)
        if px2 - px1 < 1.0 or py2 - py1 < 1.0:
            continue
        boxes.append({"bbox_xyxy": [px1, py1, px2, py2], "score": 1.0, "label": ""})
    return boxes


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))
