#!/usr/bin/env python3
"""
query_core.py — Pure-Python logic for Stage-4 semantic query.

Three responsibilities:
    1. parse_command()    — deterministic text → canonical semantic_name
    2. load_target_mapping() — YAML → semantic_name ↔ detector_label dicts
    3. select_target()    — pick the nearest matching object from memory

No ROS dependency.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml


# ── Data types ────────────────────────────────────────────────────────────

@dataclass
class MemoryObject:
    """Lightweight mirror of one Detection3D from the memory topic."""
    object_id: str
    detector_label: str
    x: float
    y: float
    confidence: float


@dataclass
class QueryResult:
    """Outcome of one semantic query."""
    success: bool
    query_text: str
    semantic_name: str = ""
    detector_label: str = ""
    object_id: str = ""
    x: float = 0.0
    y: float = 0.0
    confidence: float = 0.0
    status_message: str = ""


# ── Mapping loader ────────────────────────────────────────────────────────

def load_target_mapping(yaml_path: str | Path) -> tuple[dict[str, str], dict[str, str]]:
    """Load semantic_targets.yaml and return two lookup dicts.

    Returns:
        (sem2det, det2sem)
        sem2det:  semantic_name  → detector_label
        det2sem:  detector_label → semantic_name
    """
    path = Path(yaml_path)
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    targets = data.get("semantic_targets", [])
    sem2det: dict[str, str] = {}
    det2sem: dict[str, str] = {}
    for entry in targets:
        if not entry.get("enabled", True):
            continue
        sn = entry["semantic_name"]
        dl = entry["detector_label"]
        sem2det[sn] = dl
        det2sem[dl] = sn

    return sem2det, det2sem


# ── Command parser ────────────────────────────────────────────────────────

_FILLER = frozenset({
    "go", "to", "the", "a", "an", "please", "navigate", "approach",
    "find", "me", "take", "bring", "get", "can", "you", "i",
    "want", "need", "would", "like", "could", "help", "with",
})

_PHRASE_ALIASES: dict[str, str] = {
    "bench": "table",
    "stop sign": "stop_sign",
}


def parse_command(text: str, known_targets: set[str]) -> Optional[str]:
    """Parse a text command and return the canonical semantic_name, or None.

    Supports multi-word phrases like "stop sign" via alias lookup.
    """
    norm = (text or "").lower().strip()
    norm = re.sub(r"[^\w\s]", " ", norm)
    norm = re.sub(r"\s+", " ", norm).strip()

    if not norm:
        return None

    for phrase, canonical in _PHRASE_ALIASES.items():
        if phrase in norm and canonical in known_targets:
            return canonical

    tokens = norm.split()
    for tok in tokens:
        if tok in _FILLER:
            continue
        if tok in known_targets:
            return tok

    return None


# ── Target selection ──────────────────────────────────────────────────────

def select_target(
    objects: list[MemoryObject],
    semantic_name: str,
    detector_label: str,
    query_text: str,
) -> QueryResult:
    """Select the best memory object matching the given detector_label.

    Selection policy: highest confidence (observation_count for persistent
    landmarks), with distance as tiebreaker.
    """
    candidates = [o for o in objects if o.detector_label == detector_label]

    if not candidates:
        return QueryResult(
            success=False,
            query_text=query_text,
            semantic_name=semantic_name,
            detector_label=detector_label,
            status_message=f"no active {semantic_name} in memory",
        )

    best = max(candidates, key=lambda o: (o.confidence, -math.hypot(o.x, o.y)))

    return QueryResult(
        success=True,
        query_text=query_text,
        semantic_name=semantic_name,
        detector_label=detector_label,
        object_id=best.object_id,
        x=best.x,
        y=best.y,
        confidence=best.confidence,
        status_message=f"matched {best.object_id} (n={best.confidence:.0f}, "
                        f"d={math.hypot(best.x, best.y):.2f}m, "
                        f"{len(candidates)} candidate(s))",
    )
