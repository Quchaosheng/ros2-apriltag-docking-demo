# Rosbag Quality Analysis

The live bridge now publishes a docking pose only after camera intrinsics,
detection time, and AprilTag TF time pass the configured checks. Use this
capture workflow to make those decisions reviewable across distance, view
angle, occlusion, and lighting conditions.

## Record A Trial

Start recording before triggering a docking request. Preserve the raw bag and
write the test condition separately; the bag itself is evidence, not a claimed
hardware success rate.

```bash
mkdir -p artifacts/apriltag
ros2 bag record -o artifacts/apriltag/trial-001 \
  /camera/image_raw \
  /camera/camera_info \
  /apriltag/detections \
  /tf \
  /detected_dock_pose \
  /demo2/tag_state \
  /diagnostics
```

## Analyze Offline

Run the installed reader against the completed bag:

```bash
ros2 run demo2_apriltag_docking analyze_docking_bag \
  --bag artifacts/apriltag/trial-001 \
  --output artifacts/apriltag/trial-001.quality.json
```

The report records detection-to-recording and TF-to-recording timestamp
deltas, the nearest `tag36h11:<id>` TF association, camera-intrinsic validity,
and jumps in the published docking pose. Those timestamp deltas are
observability signals only: they are not camera exposure, model inference,
controller, or physical docking latency.

The default thresholds mirror `config/nav2_docking.yaml`; use the explicit
threshold flags when evaluating a deliberately different safety envelope.
