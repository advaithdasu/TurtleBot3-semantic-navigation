# `tb3_detector`

`tb3_detector` is Stage 1 of the TurtleBot3 perception pipeline.

It answers the first perception question:

> What does the robot see?

The node subscribes to live RGB camera images, runs YOLOv8 inference, and publishes:

- 2D object detections on `/detector_node/detections`
- an annotated debug image on `/detector_node/debug_image`

This package has already been implemented and validated in Gazebo simulation.

## What `tb3_detector` Does

`tb3_detector` detects semantic objects from live RGB camera images.

At runtime it:

1. receives a camera image from `/camera/image_raw`
2. converts the ROS image message into an OpenCV image
3. runs YOLOv8 inference
4. optionally filters detections by configured class labels
5. publishes a `vision_msgs/Detection2DArray`
6. optionally publishes a debug image with bounding boxes

This stage is intentionally 2D-only. It detects objects in image space and prepares clean detection outputs for downstream localisation.

## What It Does Not Do Yet

The current detector does **not** do the following:

- no 3D localization
- no semantic memory
- no query logic
- no navigation goal generation
- no map-frame localization
- no full tracking / data association by default

Tracking support exists as an optional experimental mode via `enable_tracking`, but the current validated MVP uses plain frame-by-frame detection.

## Why YOLOv8 Is Used

YOLOv8 was chosen because it was the fastest way to get a practical, working detector MVP integrated into ROS 2 and Gazebo.

Compared with YOLO-World, YOLOv8 was more suitable for this stage because:

- it is simpler to install and run reliably in the current ROS 2 environment
- it provides a stable closed-set detector with known COCO class names
- it is easy to validate in simulation with standard Gazebo objects
- it gives a strong engineering baseline before attempting more open-vocabulary detection

For this project stage, the goal is not open-ended semantic language grounding yet. The goal is to get a dependable detector interface that downstream modules can consume.

## Input Topics

### `/camera/image_raw`

- Type: `sensor_msgs/msg/Image`
- Purpose: main RGB input stream for object detection

This is the only image stream used for inference in the current implementation.

### `/camera/camera_info`

- Type: `sensor_msgs/msg/CameraInfo`
- Purpose: subscribed and cached, but not deeply used yet in Stage 1

The detector currently listens to camera intrinsics mainly to keep the interface ready for later stages. The current 2D detector does not need camera intrinsics for inference.

## Output Topics

### `/detector_node/detections`

- Type: `vision_msgs/msg/Detection2DArray`
- Purpose: main machine-readable detection output for downstream nodes

Each detection contains:

- `results[0].hypothesis.class_id` -> YOLO class label string
- `results[0].hypothesis.score` -> confidence score
- `bbox.center.position.x / y` -> box center in pixels
- `bbox.size_x / size_y` -> box size in pixels
- `id` -> tracking id if tracking is enabled, otherwise empty

Downstream use:

- Stage 2 localizer uses the bbox center to estimate object bearing
- later stages will use the class label and localized position for semantic memory and navigation

### `/detector_node/debug_image`

- Type: `sensor_msgs/msg/Image`
- Purpose: human-facing debug visualization

This topic shows the original camera image with bounding boxes and labels drawn on top. It is useful for quick validation that the detector sees the expected objects.

Downstream use:

- mainly for debugging and validation
- not required by the localizer

## Detection Principle

The detector pipeline is straightforward and practical:

1. A live RGB image arrives from `/camera/image_raw`
2. `cv_bridge` converts the ROS image into an OpenCV BGR image
3. `DetectorCore` runs YOLOv8 inference on that frame
4. If `class_filter` is configured, only matching detector labels are kept
5. The remaining detections are converted into `vision_msgs/Detection2DArray`
6. A debug image with boxes is generated and published if `publish_debug_image=true`

This keeps the detector stage clean:

- perception in image space happens here
- geometry and localisation are left to Stage 2

## Current Validated Semantic Test Objects

The current validated semantic test set is:

| semantic_name | detector_label | gazebo_model |
|---|---|---|
| `table` | `bench` | `table_marble` |
| `person` | `person` | `person_standing` |
| `stop_sign` | `stop sign` | `stop_sign` |

These three names belong to different layers:

- `semantic_name`
  - human / NLP / project-level semantic name
  - example: `table`, `stop_sign`

- `detector_label`
  - exact YOLO output string
  - example: `bench`, `stop sign`

- `gazebo_model`
  - model directory name used in the simulation world
  - example: `table_marble`, `stop_sign`

These names are not always identical. That distinction is intentional and important.

## Important Naming Rule

`detector.yaml` `class_filter` must use **detector labels**, not semantic names.

For example:

- use `"bench"`, not `"table"`
- use `"stop sign"`, not `"stop_sign"`
- use `"person"`, not a custom semantic alias

If this rule is broken, the detector may run correctly but silently filter out the objects you expect to keep.

## Package Structure

```text
tb3_detector/
├── package.xml
├── setup.py
├── config/
│   └── detector.yaml
├── launch/
│   └── detector.launch.py
├── models/
│   └── yolov8n.pt
└── tb3_detector/
    ├── detector_core.py
    └── detector_node.py
```

### File roles

- `tb3_detector/detector_core.py`
  - YOLOv8 wrapper
  - loads the model
  - runs inference
  - applies optional class filtering
  - returns plain Python detection dicts

- `tb3_detector/detector_node.py`
  - ROS 2 node wrapper
  - handles subscriptions, ROS message conversion, and publishing

- `config/detector.yaml`
  - main detector configuration file

- `launch/detector.launch.py`
  - ROS 2 launch entry point

- `models/yolov8n.pt`
  - current default weights file

## Configuration

The main detector parameters live in `config/detector.yaml`.

### `model_path`

- path to the YOLOv8 `.pt` file
- relative paths are resolved from the package `models/` directory

### `conf_threshold`

- minimum confidence required to keep a detection

### `device`

- inference device, e.g. `cpu` or `cuda:0`

### `class_filter`

- list of detector labels to keep
- use detector labels, not semantic names
- use `[""]` to accept all classes
- do **not** use bare `[]` in this project, because ROS 2 Humble parameter type inference can mis-handle it

### `publish_debug_image`

- enables or disables publication of the annotated debug image

### `image_topic`

- input RGB image topic for inference

### `camera_info_topic`

- camera info topic
- currently subscribed and cached, but not heavily used in Stage 1

### Launch vs YAML behavior

`detector.yaml` is the main default configuration source.

The launch file is intentionally kept simple:

- YAML provides the default values for runtime behavior
- launch only overrides a small set of parameters when explicitly requested

Current launch behavior:

- `model_path`, `device`, and `use_sim_time` can be overridden from launch
- `conf_threshold`, `class_filter`, `publish_debug_image`, `image_topic`, `camera_info_topic`, and `enable_tracking` should come from YAML unless you deliberately change the config

This keeps the runtime behavior predictable: edit the YAML, restart the node, and the new defaults take effect.

## Model Weights

The detector expects a local YOLOv8 weights file such as `yolov8n.pt`.

Default location:

```text
src/tb3_detector/models/yolov8n.pt
```

If the file is missing, startup will fail with a clear `FileNotFoundError`.

## Dependencies

```bash
sudo apt install ros-humble-vision-msgs ros-humble-cv-bridge
pip install ultralytics
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install opencv-python-headless
```

## Build

```bash
cd ~/TurtleBot3-semantic-navigation
source /opt/ros/humble/setup.bash
colcon build --packages-select tb3_detector
source install/setup.bash
```

## How To Run

### 1. Launch the Gazebo detector test world

```bash
cd ~/TurtleBot3-semantic-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_frontier_exploration detector_test_sim.launch.py
```

### 2. Launch `tb3_detector`

```bash
cd ~/TurtleBot3-semantic-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tb3_detector detector.launch.py use_sim_time:=true
```

Optional overrides:

```bash
ros2 launch tb3_detector detector.launch.py \
  use_sim_time:=true \
  model_path:=/abs/path/to/yolov8n.pt \
  device:=cpu
```

### 3. Echo detections

```bash
source /opt/ros/humble/setup.bash
source ~/TurtleBot3-semantic-navigation/install/setup.bash
ros2 topic echo /detector_node/detections
```

### 4. Inspect images with `rqt_image_view`

Open the raw camera image:

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

Open the detector debug image:

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view /detector_node/debug_image
```

## Validated Behavior

The current detector has already been validated in practice in Gazebo:

- `detector_node` launches successfully
- YOLO detections appear in the debug image
- `/detector_node/detections` publishes correctly
- `person`, `bench`, and `stop sign` have all been detected successfully in the simulation test world

This means Stage 1 is not just designed on paper. It is already working as the perception front-end for the current pipeline.

## Limitations

Current detector limitations are practical and expected:

- only 2D detection, no depth
- detection quality depends on Gazebo model appearance and visibility
- some Gazebo models do not visually resemble real COCO objects well
- class naming may differ from human semantic naming
- detector output alone is not enough for navigation without a localizer

One important engineering caveat is that Gazebo appearance matters a lot: a model that is semantically correct for the project may still be visually poor for YOLO if its textures, geometry, or silhouette do not resemble real training data.

## Future Work

Useful next steps include:

- connect `tb3_detector` cleanly to Stage 2 `tb3_localizer`
- improve the semantic test object set and test worlds
- enable optional tracking when it becomes useful
- upgrade or replace the detector later if needed
- integrate more tightly with the semantic target mapping configuration

The current role of `tb3_detector` is to provide a stable, validated Stage 1 detection interface that later semantic-navigation components can build on.
