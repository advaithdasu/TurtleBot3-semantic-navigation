import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from boxparse import parse_boxes  # noqa: E402


def test_single_box_scales_to_pixels():
    text = "<box><100><200><500><800></box>"
    boxes = parse_boxes(text, image_width=1000, image_height=500)
    assert len(boxes) == 1
    assert boxes[0]["bbox_xyxy"] == [100.0, 100.0, 500.0, 400.0]
    assert boxes[0]["score"] == 1.0


def test_multiple_boxes_preserve_order():
    text = (
        "here <box><0><0><100><100></box> and "
        "<box><500><500><1000><1000></box> done"
    )
    boxes = parse_boxes(text, 200, 200)
    assert len(boxes) == 2
    assert boxes[0]["bbox_xyxy"] == [0.0, 0.0, 20.0, 20.0]
    assert boxes[1]["bbox_xyxy"] == [100.0, 100.0, 200.0, 200.0]


def test_out_of_range_coords_are_clamped():
    text = "<box><900><900><1200><1200></box>"
    boxes = parse_boxes(text, 100, 100)
    assert boxes[0]["bbox_xyxy"] == [90.0, 90.0, 100.0, 100.0]


def test_degenerate_box_dropped():
    text = "<box><500><500><500><900></box>"  # zero width
    assert parse_boxes(text, 640, 480) == []


def test_garbage_and_empty_text():
    assert parse_boxes("no boxes here", 640, 480) == []
    assert parse_boxes("", 640, 480) == []
    assert parse_boxes("<box><1><2><3></box>", 640, 480) == []  # 3 coords
