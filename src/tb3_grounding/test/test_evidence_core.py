import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from tb3_grounding.evidence_core import EvidenceStore, view_score  # noqa: E402


def test_bigger_box_scores_higher():
    small = view_score([300, 200, 340, 260], 640, 480, confidence=0.8)
    big = view_score([200, 100, 440, 380], 640, 480, confidence=0.8)
    assert big > small > 0.0


def test_confidence_scales_score():
    lo = view_score([200, 100, 440, 380], 640, 480, confidence=0.4)
    hi = view_score([200, 100, 440, 380], 640, 480, confidence=0.8)
    assert abs(hi - 2.0 * lo) < 1e-9


def test_edge_touching_box_is_penalized():
    centered = view_score([200, 100, 440, 380], 640, 480, confidence=0.8)
    truncated = view_score([0, 100, 240, 380], 640, 480, confidence=0.8)
    assert truncated < centered


def test_degenerate_inputs():
    assert view_score([10, 10, 10, 10], 640, 480, 0.9) == 0.0
    assert view_score([10, 10, 50, 50], 0, 0, 0.9) == 0.0


def _consider(store, oid, bbox, conf, data=b"jpegdata", stamp=1.0):
    return store.consider(
        object_id=oid,
        detector_label="couch",
        bbox_xyxy=bbox,
        confidence=conf,
        image_width=640,
        image_height=480,
        image_bytes=data,
        stamp=stamp,
    )


def test_store_keeps_best_view(tmp_path):
    store = EvidenceStore(tmp_path)

    assert _consider(store, "couch_0", [300, 200, 340, 260], 0.5, b"small")
    assert not _consider(store, "couch_0", [310, 210, 335, 250], 0.5, b"worse")
    assert _consider(store, "couch_0", [150, 100, 500, 400], 0.7, b"better")

    rec = store.get("couch_0")
    assert rec is not None
    assert rec.bbox_xyxy == [150.0, 100.0, 500.0, 400.0]
    assert store.load_image("couch_0") == b"better"


def test_store_roundtrip_across_instances(tmp_path):
    writer = EvidenceStore(tmp_path)
    _consider(writer, "person_2", [100, 50, 300, 400], 0.9, b"frame")

    reader = EvidenceStore(tmp_path)
    rec = reader.get("person_2")
    assert rec is not None
    assert rec.detector_label == "couch"
    assert rec.image_width == 640
    assert reader.load_image("person_2") == b"frame"
    assert reader.object_ids() == ["person_2"]


def test_ids_with_spaces_are_filesystem_safe(tmp_path):
    store = EvidenceStore(tmp_path)
    _consider(store, "stop sign_0", [100, 50, 300, 400], 0.9)
    rec = store.get("stop sign_0")
    assert " " not in rec.image_file
    assert store.load_image("stop sign_0") == b"jpegdata"


def test_reload_picks_up_external_writes(tmp_path):
    reader = EvidenceStore(tmp_path)
    assert reader.get("couch_0") is None

    writer = EvidenceStore(tmp_path)
    _consider(writer, "couch_0", [100, 50, 300, 400], 0.9)

    reader.reload()
    assert reader.get("couch_0") is not None
