# Watchtower Node

A ROS2 node for robot localization using a watchtower camera with ArUco markers. Features automatic extrinsic calibration and real-time robot tracking.

## Overview

The watchtower node implements a state machine with three states:

1. **INIT**: Initialize node, load camera parameters, create GUI
2. **CALIBRATION**: Perform extrinsic calibration using SIFT feature matching
3. **RUNNING**: Continuously localize robots using ArUco markers

## Features

- **GUI Control**: Simple Tkinter GUI for easy calibration control
- **Automatic Extrinsic Calibration**: Uses SIFT features and PnP to calibrate camera pose
- **Real-time Robot Tracking**: Detects ArUco markers and publishes robot poses
- **Visualization**: Publishes annotated images with detected markers and axes
- **Webots Compatible**: Handles coordinate frame transformations for simulation

## Prerequisites

- Camera intrinsic calibration file (YAML format)
- Table reference image (top-down view of the playing field)
- ArUco markers mounted on robots (DICT_4X4_50)

## Usage

### Basic Launch

```bash
ros2 launch champi_vision watchtower.launch.py
```

### Launch with script
```bash
./start_watchtower.sh
```

### With Custom Parameters

```bash
ros2 launch champi_vision watchtower.launch.py \
  camera_info_file:=/path/to/camera_calib.yaml \
  table_reference_image:=/path/to/table_reference.png \
  marker_length:=0.07 \
  publish_rate:=30.0 \
  image_topic:=/watchtower/image_raw
```

### Calibration Workflow

1. Launch the node
2. A GUI window will appear showing "State: Initialization"
3. Click the **"Start Calibration"** button
4. The node will:
   - Capture one image from the camera
   - Match SIFT features between the reference image and camera image
   - Solve PnP to estimate camera pose
   - Display calibration results
5. Automatically transition to **RUNNING** state
6. Robot poses are now published on `/watchtower/robot_poses`

## Topics

### Subscribed
- `/watchtower/image_raw` (sensor_msgs/Image): Camera images

### Published
- `/watchtower/robot_poses` (geometry_msgs/PoseArray): Detected robot poses in world frame
- `/watchtower/state` (std_msgs/String): Current state of the node
- `/watchtower/visualization` (sensor_msgs/Image): Annotated image with detected markers

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_info_file` | string | (required) | Path to camera intrinsic calibration YAML |
| `table_reference_image` | string | (required) | Path to table reference image PNG |
| `marker_length` | double | 0.07 | Size of ArUco markers (meters) |
| `publish_rate` | double | 30.0 | Publishing frequency (Hz) |
| `image_topic` | string | /watchtower/image_raw | Camera image topic |
| `is_simu_with_webots` | bool | false | Enable Webots coordinate transform |
| `marker_id_min` | int | 0 | Minimum marker ID to detect |
| `marker_id_max` | int | 10 | Maximum marker ID to detect |

## Camera Calibration File Format

The camera info file should be a YAML file with the following structure:

```yaml
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]

distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]
```

## Table Reference Image

The table reference image should be:
- Top-down view of the playing field
- Grayscale or color (will be converted to grayscale)
- Contains distinctive features for SIFT matching
- Properly scaled (pixel-to-meter ratio is computed from `table_width` and `table_height`)

## Coordinate Frames

- **World frame**: Origin at table center, Z-up (or as defined by table reference)
- **Camera frame**: 
  - Real camera: Standard OpenCV convention
  - Webots: Webots camera convention with automatic transformation
- **Robot poses**: Published in world frame with quaternion orientation


## Implementation Details

### State Machine
- Uses Python `Enum` for clean state management
- Thread-safe GUI updates with callbacks
- Graceful shutdown on GUI close

### Calibration Algorithm
1. Detect SIFT keypoints in both images
2. Match descriptors with BFMatcher
3. Apply Lowe's ratio test for filtering
4. Robust outlier rejection with RANSAC
5. Convert pixel coordinates to 3D world coordinates
6. Solve PnP for camera pose estimation
7. Transform to desired coordinate frame

### Localization Algorithm
1. Detect ArUco markers in camera image
2. Estimate marker pose using solvePnP
3. Transform from camera frame to world frame
4. Filter by marker ID range
5. Publish as PoseArray


## See Also

- [WatchtowerRobotLocalizer](../champi_vision/watchtower/robot_localization_from_watchtower.py)
- [WatchtowerExtrinsicCalibrator](../champi_vision/watchtower/watchtower_extrinsic_calibration.py)
