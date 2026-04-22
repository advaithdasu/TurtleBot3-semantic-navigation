#!/usr/bin/env python3
"""
detector_core.py  —  Stage-1 perception: YOLOv8 inference wrapper.

Responsibilities:
  - Load a YOLOv8 model from a configurable local path.
  - Run inference on a BGR numpy image (from cv_bridge).
  - Return a list of Detection dicts:
      {
          "label":      str,          # class name, e.g. "chair"
          "conf":       float,        # 0.0 – 1.0
          "bbox_xyxy":  [x1,y1,x2,y2] (pixels, float),
          "track_id":   int | None,   # placeholder; filled in when tracking is enabled
      }

NOT responsible for:
  - Coordinate projection to 3D (→ localizer, Stage 2)
  - Maintaining object history  (→ memory,    Stage 3)
  - Answering semantic queries  (→ query,     Stage 4)
  - Sending Nav2 goals          (→ nav adapter,Stage 5)

Future integration note:
  The returned dict list is the stable interface that the Stage-2 localizer will
  consume together with a depth / camera-info message. Do not change key names
  without updating the localizer as well.
"""

from __future__ import annotations
import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Optional import — graceful failure if ultralytics not yet installed
# ---------------------------------------------------------------------------
try:
    from ultralytics import YOLO as _UltralyticsYOLO
    _ULTRALYTICS_AVAILABLE = True
except ImportError:
    _UltralyticsYOLO = None
    _ULTRALYTICS_AVAILABLE = False
    logger.warning(
        "ultralytics not found. Install with:  pip install ultralytics\n"
        "detector_core will raise RuntimeError on load() until then."
    )


# ---------------------------------------------------------------------------
# Public detection type (plain dict for simplicity)
# ---------------------------------------------------------------------------
# Keys every downstream consumer may rely on:
DETECTION_KEYS = ("label", "conf", "bbox_xyxy", "track_id")


class DetectorCore:
    """
    Thin wrapper around a YOLOv8 model.

    Usage::

        core = DetectorCore(model_path="models/yolov8n.pt", conf_threshold=0.4)
        core.load()                          # loads weights once at startup
        detections = core.infer(bgr_image)   # list of dicts

    Parameters
    ----------
    model_path : str | Path
        Absolute or relative path to the YOLOv8 .pt weights file.
    conf_threshold : float
        Minimum confidence to include a detection (0.0 – 1.0).
    class_filter : list[str] | None
        If given, only return detections whose label is in this list.
        None means return all detected classes.
    device : str
        Torch device string, e.g. "cpu", "cuda:0", "mps".
    enable_tracking : bool
        If True, use model.track() instead of model.predict() (ByteTrack).
        track_id in the result dict will be populated.
    """

    def __init__(
        self,
        model_path: str | Path,
        conf_threshold: float = 0.35,
        class_filter: list[str] | None = None,
        device: str = "cpu",
        enable_tracking: bool = False,
    ) -> None:
        self.model_path = Path(model_path)
        self.conf_threshold = float(conf_threshold)
        self.class_filter = set(class_filter) if class_filter else None
        self.device = device
        self.enable_tracking = enable_tracking

        self._model: Any = None   # set by load()

    # ------------------------------------------------------------------
    def load(self) -> None:
        """
        Load model weights.  Call once during node on_configure / __init__.
        Raises RuntimeError if ultralytics is missing or the file is absent.
        """
        if not _ULTRALYTICS_AVAILABLE:
            raise RuntimeError(
                "ultralytics package is not installed. "
                "Run:  pip install ultralytics"
            )
        if not self.model_path.is_file():
            raise FileNotFoundError(
                f"Model file not found: {self.model_path}\n"
                "► See the STOP HERE checkpoint in the README to download weights."
            )

        logger.info("Loading YOLOv8 model from %s on device=%s", self.model_path, self.device)
        self._model = _UltralyticsYOLO(str(self.model_path))
        self._model.to(self.device)
        logger.info("Model loaded. Classes: %s", list(self._model.names.values()))

    # ------------------------------------------------------------------
    def infer(self, bgr_image) -> list[dict]:
        """
        Run inference on a single BGR uint8 numpy array.

        Returns a (possibly empty) list of detection dicts.
        Each dict has keys: label, conf, bbox_xyxy, track_id.
        """
        if self._model is None:
            raise RuntimeError("DetectorCore.load() has not been called yet.")

        if self.enable_tracking:
            results = self._model.track(
                bgr_image,
                conf=self.conf_threshold,
                device=self.device,
                persist=True,
                verbose=False,
            )
        else:
            results = self._model.predict(
                bgr_image,
                conf=self.conf_threshold,
                device=self.device,
                verbose=False,
            )

        detections: list[dict] = []
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            names = result.names  # int -> str

            for i in range(len(boxes)):
                label = names[int(boxes.cls[i].item())]
                if self.class_filter and label not in self.class_filter:
                    continue

                conf = float(boxes.conf[i].item())
                xyxy = boxes.xyxy[i].tolist()  # [x1, y1, x2, y2]

                track_id = None
                if self.enable_tracking and boxes.id is not None:
                    track_id = int(boxes.id[i].item())

                detections.append(
                    {
                        "label":     label,
                        "conf":      conf,
                        "bbox_xyxy": xyxy,
                        "track_id":  track_id,
                    }
                )

        return detections

    # ------------------------------------------------------------------
    @property
    def is_loaded(self) -> bool:
        return self._model is not None
