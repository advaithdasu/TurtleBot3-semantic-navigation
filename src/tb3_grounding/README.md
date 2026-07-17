# tb3_grounding — Stage-5 attribute grounding

Gives the semantic-navigation stack **appearance-level spatial
reasoning**: commands like *"go to the sofa with warm color"* are
resolved by a vision-language grounding model
([NVIDIA LocateAnything](https://research.nvidia.com/labs/lpr/locate-anything/))
instead of the class/index rules of Stage 4.

## The problem

Stages 1–4 reduce every object to `(detector_label, x, y)`. The camera
frame is discarded the moment YOLO has produced boxes, so at query time
there is *nothing* that knows what `couch_0` looks like versus
`couch_1` — "the sofa with warm color" has no data to bind to.

## The design

```
        exploration time (continuous, cheap)
        ────────────────────────────────────
        /camera/image_raw ──────────────┐
        /detector_node/detections ──────┤   evidence_store_node
        /localizer_node/localized_objects┤   pair 2D bbox ↔ 3D point (label+bearing)
        /semantic_map_memory_node/       │   TF base_link → map
                    landmark_objects ────┘   match to nearest same-class landmark
                                             keep BEST view per landmark on disk
                                                    │
                                                    ▼
                                   ~/.tb3_semantic_nav/evidence/
                                   index.json + couch_0.jpg + couch_1.jpg ...

        query time (rare, expensive)
        ────────────────────────────
        "go to the sofa with warm color"
                 │ parse_command → attribute_expression
                 ▼
        semantic_query_node ── HTTP ──► grounding/server.py (GPU box)
                 │      for each same-class candidate's best-view frame:
                 │      "Locate all instances matching: sofa with warm color"
                 ▼
        score = IoU(model box, candidate's stored bbox) × box score
                 │  winner → SemanticQueryResult → nav adapter → Nav2
```

Key points:

- **The VLM never runs per-frame.** YOLOv8n keeps doing continuous
  detection; LocateAnything is consulted only when a command carries
  attributes, over ≤ a handful of stored frames.
- **IoU is the disambiguator.** The model is asked *where* the
  expression is in each candidate's own frame. If its answer lands on
  that candidate's stored bounding box, the candidate matches; if it
  lands on some *other* object in the frame (say, the other, cooler
  sofa in the background), the IoU — and the score — collapse.
- **Best-view selection** (`evidence_core.view_score`): confidence ×
  √(bbox area fraction), halved when the box touches the image border
  (a truncated object is poor grounding evidence). A new frame replaces
  the stored one only when it beats it.
- **Deferred association.** Landmarks are only promoted after several
  observations, often *after* the best frames were seen; unmatched
  observations wait in a pending buffer and are retro-matched when the
  next landmark snapshot arrives.
- **Graceful degradation.** Grounding server down, no evidence yet, or
  `tb3_grounding` not installed → the query node falls back to the
  deterministic nearest-first path. But when grounding *works* and no
  candidate clears `min_grounding_score`, the query fails explicitly —
  navigating to an arbitrary sofa would be a wrong answer, not a
  fallback.

## Nodes

### `evidence_store_node`

| | |
|---|---|
| Subscribes | `/camera/image_raw`, `/detector_node/detections`, `/localizer_node/localized_objects`, `/semantic_map_memory_node/landmark_objects` |
| Writes | `<evidence_dir>/index.json` + one best-view JPEG per landmark |
| Config | [`config/grounding.yaml`](config/grounding.yaml) |

`camera_hfov_deg` must match the localizer's, and `match_distance`
should equal the map memory's `merge_distance` (both 0.8 m by default).

## Modules (pure Python, unit-tested without ROS)

- `evidence_core.py` — best-view scoring + atomic on-disk store.
- `resolver_core.py` — IoU, per-candidate scoring, ranking.
- `grounding_client.py` — stdlib HTTP client for `grounding/server.py`.

```bash
python3 -m pytest src/tb3_grounding/test/ -q
```

## Running the full flow

1. Start the grounding server somewhere reachable (see
   [`grounding/README.md`](../../grounding/README.md); use
   `--backend mock` for a GPU-free color-only demo).
2. `ros2 launch tb3_coordinator full_semantic_nav.launch.py` — the
   evidence store node is included automatically.
3. Let exploration observe the objects, then:

```bash
ros2 topic pub --once /user_command std_msgs/String "data: 'go to the sofa with warm color'"
```

`/semantic_query_node/query_status` reports the grounding outcome, e.g.
`grounded 'sofa with warm color' → couch_1 (score=0.62, iou=0.71; couch_1=0.62, couch_0=0.03)`.
