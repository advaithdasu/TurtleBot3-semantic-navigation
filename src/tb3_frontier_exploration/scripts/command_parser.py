#!/usr/bin/env python3
"""
Lightweight rule-based command parser for fake semantic navigation.

Normalizes text, tokenizes, strips filler words, finds the first known object label
from semantic_goals (passed as allowed_objects), returns optional trailing modifiers.

No LLM, no external NLP models — regex, stopword filtering, and keyword matching only.
"""

from __future__ import annotations

import re

# Default when no runtime object list is provided.
# Must match semantic_name values in config/semantic_targets.yaml.
DEFAULT_OBJECTS = frozenset({"table", "stop_sign", "person"})

# Tokens removed everywhere (leading context and modifier tail). Keep list tight to avoid eating content words.
_FILLER = frozenset(
    {
        "go",
        "to",
        "the",
        "a",
        "an",
        "please",
        "navigate",
        "navigation",
        "approach",
        "approaching",
        "find",
        "finding",
        "me",
        "take",
        "bring",
        "get",
        "gimme",
        "give",
        "us",
        "can",
        "you",
        "i",
        "want",
        "need",
        "would",
        "like",
        "could",
        "help",
        "with",
        "at",
        "for",
        "into",
        "towards",
        "toward",
        "and",
        "or",
        "then",
        "there",
        "here",
        "now",
        "soon",
        "just",
        "quickly",
        "please",
    }
)


def normalize_command_text(text: str) -> str:
    """Lowercase, strip edges, replace punctuation with spaces, collapse whitespace."""
    s = (text or "").lower().strip()
    s = re.sub(r"[^\w\s]", " ", s)
    s = re.sub(r"\s+", " ", s).strip()
    return s


def parse_command(
    text: str,
    allowed_objects: set[str] | frozenset[str] | None = None,
) -> tuple[bool, str | None, list[str], str | None]:
    """
    Parse a natural-language command and extract an object label plus optional modifiers.

    Args:
        text: Raw user input.
        allowed_objects: Valid object labels (lowercase). If None, uses DEFAULT_OBJECTS.

    Returns:
        (success, object_label, modifiers, error_message).
        - On success: (True, "table", ["blue", "mug"], None).
        - On failure: (False, None, [], "…").
        Navigation still uses only object_label; modifiers are for logging / future use.
    """
    if allowed_objects is None:
        allowed_objects = DEFAULT_OBJECTS
    else:
        allowed_objects = {str(o).lower() for o in allowed_objects}

    norm = normalize_command_text(text)
    if not norm:
        return False, None, [], "Empty command."

    tokens = norm.split()
    if not tokens:
        return False, None, [], "Empty command."

    object_idx: int | None = None
    object_label: str | None = None
    for i, tok in enumerate(tokens):
        if tok not in allowed_objects:
            continue
        prefix = tokens[:i]
        if not all(p in _FILLER for p in prefix):
            continue
        object_idx = i
        object_label = tok
        break

    if object_label is None or object_idx is None:
        supported = ", ".join(sorted(allowed_objects))
        return (
            False,
            None,
            [],
            f"No known object in command. Supported: {supported}.",
        )

    raw_tail = tokens[object_idx + 1 :]
    modifiers = [t for t in raw_tail if t not in _FILLER]

    return True, object_label, modifiers, None


# ---- Example usage and tests (run with: python3 command_parser.py) ----

def _run_tests() -> None:
    objs = frozenset({"table", "stop_sign", "person"})
    cases: list[tuple[str, bool, str | None, list[str]]] = [
        ("go to table", True, "table", []),
        ("  go to stop_sign  ", True, "stop_sign", []),
        ("Go To Person", True, "person", []),
        ("navigate to table", True, "table", []),
        ("go to the table", True, "table", []),
        ("navigate to the stop_sign", True, "stop_sign", []),
        ("go find a table", True, "table", []),
        ("approach the person", True, "person", []),
        ("take me to the stop_sign", True, "stop_sign", []),
        ("go to the table with a blue mug", True, "table", ["blue", "mug"]),
        ("find the person near the wall", True, "person", ["near", "wall"]),
        ("", False, None, []),
        ("pick up table", False, None, []),
        ("go to", False, None, []),
        ("go to sofa", False, None, []),
        ("go to TABLE", True, "table", []),
    ]
    for raw, expect_ok, expect_obj, expect_mod in cases:
        ok, obj, mods, err = parse_command(raw, allowed_objects=objs)
        assert ok == expect_ok and obj == expect_obj and mods == expect_mod, (
            f"parse_command({raw!r}) -> ok={ok}, obj={obj}, mods={mods}, err={err}"
        )
    print("All tests passed.")


if __name__ == "__main__":
    _run_tests()
