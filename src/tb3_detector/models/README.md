# tb3_detector/models

Place YOLOv8 weight files here.

## Required for first demo

| Filename       | Download source |
|----------------|-----------------|
| `yolov8n.pt`   | See README.md → STOP HERE checkpoint |

## Naming convention

- `yolov8n.pt`           — official nano weights (COCO-80)
- `yolov8s.pt`           — official small weights (COCO-80)
- `yolov8n_tb3_lab.pt`   — custom fine-tuned on your lab scene (future)

## .gitignore

Weight files (.pt) are excluded from version control (large binary files).
Use git-lfs or a separate model registry if you need to share weights with the team.
