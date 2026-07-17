"""Grounding-based candidate selection.

For each same-class candidate, the grounding model is asked where the
expression ("sofa with warm color") is in that candidate's best-view
frame. A candidate scores by IoU between the model's answer box and its
own stored bbox, times the box score: if the model points at some other
object in the frame, the score collapses.

The grounding call is injected as ground_fn(image_bytes, expression),
so this module has no ROS or network dependency.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional


@dataclass
class EvidenceCandidate:
    object_id: str
    ref_bbox_xyxy: list      # the stored YOLO bbox in the evidence frame
    image_bytes: bytes


@dataclass
class CandidateGrounding:
    object_id: str
    score: float             # max(IoU x box score) over returned boxes
    best_iou: float
    n_boxes: int


def iou(a_xyxy: list, b_xyxy: list) -> float:
    ax1, ay1, ax2, ay2 = a_xyxy
    bx1, by1, bx2, by2 = b_xyxy
    iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    if union <= 0.0:
        return 0.0
    return inter / union


def score_candidate(boxes: list[dict], ref_bbox_xyxy: list) -> tuple[float, float]:
    """Return (score, best_iou) for one candidate's grounding response."""
    best_score = 0.0
    best_iou = 0.0
    for box in boxes:
        j = iou(box["bbox_xyxy"], ref_bbox_xyxy)
        s = j * float(box.get("score", 1.0))
        if s > best_score:
            best_score = s
        if j > best_iou:
            best_iou = j
    return best_score, best_iou


def rank_candidates(
    candidates: list[EvidenceCandidate],
    expression: str,
    ground_fn: Callable[[bytes, str], list[dict]],
) -> list[CandidateGrounding]:
    """Ground the expression against each candidate's frame; results are
    sorted best-first. Exceptions from ground_fn propagate so the caller
    decides how to degrade."""
    results = []
    for cand in candidates:
        boxes = ground_fn(cand.image_bytes, expression)
        score, best_iou = score_candidate(boxes, cand.ref_bbox_xyxy)
        results.append(CandidateGrounding(
            object_id=cand.object_id,
            score=score,
            best_iou=best_iou,
            n_boxes=len(boxes),
        ))
    results.sort(key=lambda r: -r.score)
    return results


def pick_best(
    results: list[CandidateGrounding],
    min_score: float,
) -> Optional[CandidateGrounding]:
    if not results or results[0].score < min_score:
        return None
    return results[0]
