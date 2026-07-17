import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from tb3_grounding.resolver_core import (  # noqa: E402
    CandidateGrounding,
    EvidenceCandidate,
    iou,
    pick_best,
    rank_candidates,
    score_candidate,
)


def test_iou_identical():
    assert iou([0, 0, 10, 10], [0, 0, 10, 10]) == pytest.approx(1.0)


def test_iou_disjoint():
    assert iou([0, 0, 10, 10], [20, 20, 30, 30]) == 0.0


def test_iou_half_overlap():
    # inter=50, union=150
    assert iou([0, 0, 10, 10], [5, 0, 15, 10]) == pytest.approx(1 / 3)


def test_iou_degenerate():
    assert iou([5, 5, 5, 5], [0, 0, 10, 10]) == 0.0


def test_score_candidate_takes_best_box():
    ref = [0, 0, 10, 10]
    boxes = [
        {"bbox_xyxy": [50, 50, 60, 60], "score": 1.0},   # miss
        {"bbox_xyxy": [0, 0, 10, 10], "score": 0.8},     # hit
    ]
    score, best_iou = score_candidate(boxes, ref)
    assert best_iou == pytest.approx(1.0)
    assert score == pytest.approx(0.8)


def test_score_candidate_no_boxes():
    assert score_candidate([], [0, 0, 10, 10]) == (0.0, 0.0)


def _fake_ground_fn(responses: dict):
    def fn(image_bytes, expression):
        return responses[image_bytes]
    return fn


def test_rank_selects_candidate_where_model_box_hits_stored_bbox():
    # couch_0's evidence: the model points elsewhere. couch_1: the model
    # box lands on the stored bbox, so couch_1 wins.
    candidates = [
        EvidenceCandidate("couch_0", [100, 100, 200, 200], b"img0"),
        EvidenceCandidate("couch_1", [300, 100, 420, 220], b"img1"),
    ]
    ground = _fake_ground_fn({
        b"img0": [{"bbox_xyxy": [400, 300, 500, 380], "score": 0.9}],
        b"img1": [{"bbox_xyxy": [305, 105, 415, 215], "score": 0.9}],
    })

    results = rank_candidates(candidates, "sofa with warm color", ground)
    assert [r.object_id for r in results] == ["couch_1", "couch_0"]
    assert results[0].score > 0.5
    assert results[1].score == 0.0

    best = pick_best(results, min_score=0.05)
    assert best is not None and best.object_id == "couch_1"


def test_pick_best_respects_floor():
    results = [CandidateGrounding("couch_0", score=0.01, best_iou=0.01, n_boxes=1)]
    assert pick_best(results, min_score=0.05) is None
    assert pick_best([], min_score=0.05) is None


def test_ground_fn_errors_propagate():
    def boom(image_bytes, expression):
        raise RuntimeError("server down")

    with pytest.raises(RuntimeError):
        rank_candidates(
            [EvidenceCandidate("couch_0", [0, 0, 1, 1], b"x")], "q", boom)
