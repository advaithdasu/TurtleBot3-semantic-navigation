# grounding/ — LocateAnything HTTP server

A minimal, framework-free HTTP wrapper around
[NVIDIA LocateAnything-3B](https://research.nvidia.com/labs/lpr/locate-anything/)
(referring-expression grounding: image + phrase → bounding boxes) so the
ROS 2 stack — which runs CPU-only, typically inside a container — can call
the model over the network at query time.

This directory is **not a ROS package**: it has no rclpy dependency and is
meant to run wherever the GPU is (a workstation, a lab server, a cloud
box). The ROS side talks to it through
`src/tb3_grounding/tb3_grounding/grounding_client.py`.

## Why a separate server

- LocateAnything-3B requires an **NVIDIA Ampere-or-newer GPU, Linux, and
  ~12 GB VRAM** ([model card](https://huggingface.co/nvidia/LocateAnything-3B));
  the simulation stack is CPU-only.
- Throughput is ~12.7 boxes/s *on an H100* — far too slow to replace
  YOLOv8n per-frame. The pipeline therefore calls it only **at query
  time**, over a handful of stored best-view frames.
- Pinning the model's exact dependency set (`transformers==4.57.1`,
  `numpy==1.25.0`, ...) in its own environment keeps it out of the ROS
  container entirely.

## API

```
GET  /health
     → {"status": "ok", "backend": "...", "device": "...", "model": "..."}

POST /ground
     {"image": "<base64 jpeg/png>", "query": "sofa with warm color"}
     → {"boxes": [{"bbox_xyxy": [x1, y1, x2, y2], "score": 0.9, "label": ""}, ...],
        "width": 640, "height": 480, "backend": "locate_anything", "latency_ms": 312.4}
```

Box coordinates are **pixels** in the posted image. The server is
synchronous and unauthenticated — run it on a trusted network only.

## Backends

### `locate_anything` (the real model)

```bash
# 1. Environment (Linux + CUDA GPU). Install torch for your CUDA first:
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
pip install "transformers==4.57.1" "numpy==1.25.0" "Pillow==11.1.0" \
            "opencv-python-headless==4.11.0.86" peft "decord==0.6.0" "lmdb==1.7.5"

# 2. Run (weights auto-download from HuggingFace on first start, ~7 GB):
python3 server.py --backend locate_anything --port 8801
```

Inference follows the model card: prompt
`"Locate all instances matching: <phrase>"`, hybrid generation mode
(parallel box decoding with next-token fallback), output parsed from
`<box><x1><y1><x2><y2></box>` tokens normalized to `[0, 1000]`
(`boxparse.py`).

**License:** the LocateAnything-3B weights permit **non-commercial
research use only**.

### `mock` (GPU-free stand-in)

```bash
pip install opencv-python-headless numpy
python3 server.py --backend mock --port 8801
```

An HSV color heuristic that resolves color-attribute queries ("warm
color", "cool color", "red", "blue", ...) by masking matching pixels and
returning the largest connected regions. It exercises the *entire*
evidence → grounding → IoU-ranking → Nav2 pipeline on a laptop, and is
what the unit tests target. It understands **colors only** — any other
attribute returns no boxes.

## Smoke test

```bash
python3 -m pytest tests/ -q

# with a server running:
curl -s localhost:8801/health
python3 - <<'EOF'
import base64, json, urllib.request
img = base64.b64encode(open("some_frame.jpg", "rb").read()).decode()
req = urllib.request.Request(
    "http://localhost:8801/ground",
    data=json.dumps({"image": img, "query": "sofa with warm color"}).encode(),
    headers={"Content-Type": "application/json"})
print(json.load(urllib.request.urlopen(req)))
EOF
```
