"""Pure-Python parser tests: python3 -m pytest src/tb3_query/test/test_query_core.py"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from tb3_query.query_core import parse_command  # noqa: E402

KNOWN = {"person", "table", "stop_sign", "sofa"}


def test_plain_command_has_no_expression():
    p = parse_command("go to the person", KNOWN)
    assert p is not None
    assert p.semantic_name == "person"
    assert p.desired_index is None
    assert p.attribute_expression is None


def test_indexed_command_has_no_expression():
    for text in ("go to person 3", "go to person_3",
                 "go to person3", "go to person number 3"):
        p = parse_command(text, KNOWN)
        assert p is not None, text
        assert p.desired_index == 3, text
        assert p.attribute_expression is None, text


def test_unknown_target_rejected():
    assert parse_command("go to the fridge", KNOWN) is None


def test_warm_color_sofa():
    p = parse_command("go to the sofa with warm color", KNOWN)
    assert p is not None
    assert p.semantic_name == "sofa"
    assert p.desired_index is None
    assert p.attribute_expression == "sofa with warm color"


def test_adjective_before_noun():
    p = parse_command("go to the red sofa", KNOWN)
    assert p is not None
    assert p.semantic_name == "sofa"
    assert p.attribute_expression == "red sofa"


def test_couch_alias_maps_to_sofa():
    p = parse_command("navigate to the couch with cool color", KNOWN)
    assert p is not None
    assert p.semantic_name == "sofa"
    assert p.attribute_expression == "couch with cool color"


def test_bare_with_is_not_an_attribute():
    p = parse_command("help me with the table", KNOWN)
    assert p is not None
    assert p.semantic_name == "table"
    assert p.attribute_expression is None


def test_index_wins_over_expression_flagging():
    # Both are reported; the query node prefers the explicit index.
    p = parse_command("go to person 2 with red shirt", KNOWN)
    assert p is not None
    assert p.semantic_name == "person"
    assert p.desired_index == 2
    assert p.attribute_expression == "person with red shirt"
