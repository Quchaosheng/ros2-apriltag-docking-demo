# AprilTag Visual Docking Demo Design

## Purpose

Build a small ROS 2 demo that shows this complete path in Gazebo:

1. Nav2 navigates a TurtleBot3 to a configured staging pose.
2. `apriltag_ros` detects the charging dock's AprilTag from the robot camera.
3. A bridge rejects unsafe detections and publishes a stable dock pose.
4. Nav2 Docking refines the pose continuously and drives the final approach.
5. The demo reports success, retries, cancellation, tag loss, and timeout states.

The target platform is Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and a standard CPU. Jetson-specific acceleration and real charging hardware are outside the first demo.

## Approaches Considered

### 1. Built-in `SimpleChargingDock` plus two small bridge nodes

Use Nav2's existing `opennav_docking::SimpleChargingDock` with `use_external_detection_pose: true`. Add one node for AprilTag validation and pose publication, and one node for Guard, Action, and Monitor integration.

This is the selected approach. Nav2 already supplies staging-pose navigation, final approach control, pose filtering, detection timeout, retry handling, docking timeout, and simulated charging detection. The project only owns the policy and integration logic specific to the demo.

### 2. Custom Nav2 Dock Plugin

Implement a C++ `ChargingDock` plugin that subscribes directly to AprilTag detections and owns all validation and pose conversion.

This gives tighter control but duplicates behavior already present in `SimpleChargingDock`, increases plugin and lifecycle complexity, and makes the demo harder to inspect. It should only be considered later if real hardware requires custom contact, stall, or charging semantics.

### 3. Fake AprilTag pose publisher

Publish a scripted `PoseStamped` without processing a camera image.

This is the fastest way to demonstrate Nav2 Docking, but it does not prove the requested visual pipeline or rejection behavior. It may remain useful as a test fixture, not as the main demo.

## Package Boundary

Use a single `ament_python` package named `demo2_apriltag_docking`. It contains the two Python nodes, pure policy code, configuration, launch files, Gazebo assets, and tests. A second interface package is deliberately avoided: the demo reuses standard ROS messages, services, diagnostics, and `nav2_msgs/action/DockRobot`.

### `tag_policy.py`

Pure Python policy with no ROS dependency. It maps Tag IDs to dock records and decides whether a frame is publishable.

Inputs:

- Detected Tag IDs, Hamming distance, decision margin, timestamp, position, and yaw.
- Configured Tag-to-dock mapping.

Rules:

- Reject an empty frame.
- Reject any frame containing more than one Tag.
- Reject unknown Tag IDs.
- Require `hamming == 0` and `decision_margin >= 50.0` by default.
- Require three consecutive valid frames for the same Tag within 0.5 seconds.
- Reject a pose jump larger than 0.25 m or 20 degrees and restart confirmation.
- Limit accepted pose publication to 10 Hz while continuing to refresh often enough for Nav2's external-detection timeout.
- Emit state changes only when the state or rejection reason changes.

All thresholds are ROS parameters because camera noise and physical geometry need calibration.

### `tag_pose_bridge.py`

Subscribe to `apriltag_msgs/msg/AprilTagDetectionArray`. Use the detection metadata for the policy decision and TF published by `apriltag_ros` for the corresponding Tag pose. Publish the accepted pose as `geometry_msgs/msg/PoseStamped` on `/detected_dock_pose`, which is the native input of `SimpleChargingDock`.

The bridge also publishes a transient-local `std_msgs/msg/String` state and a `diagnostic_msgs/msg/DiagnosticArray` entry. It never invents a last-known pose after the Tag is lost; stopping publication intentionally lets Nav2 reject stale data.

### `docking_task_bridge.py`

Expose two standard services:

- `/demo2/start_docking` using `std_srvs/srv/Trigger`.
- `/demo2/cancel_docking` using `std_srvs/srv/Trigger`.

The target Tag is a parameter for this one-dock demo. The node resolves that Tag through the shared mapping, checks the Guard input, sends a `DockRobot` goal using the mapped `dock_id`, relays Action feedback, and cancels the action if Guard becomes false.

Guard integration uses `std_msgs/msg/Bool` on a configurable topic. `guard_required` is false in the standalone demo launch and true when connected to a real Guard publisher. When required, missing or stale Guard data is treated as denied.

Monitor integration uses `/demo2/docking_state` and `/diagnostics`. States are `IDLE`, `WAITING_FOR_TAG`, `NAV_TO_STAGING`, `INITIAL_PERCEPTION`, `CONTROLLING`, `WAIT_FOR_CHARGE`, `RETRY`, `SUCCEEDED`, `CANCELED`, `GUARD_DENIED`, and `FAILED`.

## Data Flow

```text
Gazebo camera image + CameraInfo
        |
        v
apriltag_ros ---- detections metadata ----> TagPoseBridge
        |                                      |
        +-------------- TF pose ---------------+
                                               |
                                               v
                                  /detected_dock_pose
                                               |
                                               v
DockingTaskBridge -- DockRobot Action --> Nav2 Docking Server
       ^                                       |
       |                                       v
 Guard Bool                         SimpleChargingDock
       |                                       |
       +------ state / diagnostics <-----------+
```

The dock database stores the approximate fixed-frame dock pose used to compute the staging pose. Visual data only refines the final approach.

## Dock Mapping

The demo has one record:

```yaml
docks:
  0:
    dock_id: demo_charge_dock
    dock_type: charging_dock
    tag_frame: tag36h11:0
```

The same mapping is loaded by both bridge nodes. Startup fails with a clear error if the configured target Tag is absent, dock IDs are duplicated, or required fields are missing.

## Failure Behavior

- **Low confidence:** Reject the frame, preserve the current action, and report `LOW_MARGIN` or `HAMMING` once per state transition.
- **Unknown Tag:** Reject it and report `UNKNOWN_TAG`; never switch targets implicitly.
- **Multiple Tags:** Reject the complete frame as ambiguous.
- **Tag loss:** Publish no pose. After 0.5 seconds report `TAG_LOST`; Nav2's `external_detection_timeout` causes perception/control retry.
- **Pose jump:** Reject the sample, reset the three-frame confirmation window, and report `POSE_JUMP`.
- **Docking timeout:** Let Nav2 enforce `initial_perception_timeout`, `dock_approach_timeout`, and `wait_charge_timeout`; relay the resulting Action error code.
- **Guard denial:** Do not start a goal. If denial occurs during docking, request Action cancellation and report `GUARD_DENIED`.
- **Repeated requests:** Reject a second start request while an Action goal is active.

## Simulation

The world contains a TurtleBot3, a front-facing RGB camera, a flat charging-dock plate, and the official `tag36h11` ID 0 image. The texture is downloaded from `AprilRobotics/apriltag-imgs` during workspace preparation and checked into the package so the demo does not require network access at runtime.

`SimpleChargingDock` runs with `use_battery_status: false`; its existing distance check represents a successful connection in simulation.

## Verification

Automated tests cover:

- Mapping validation.
- Low margin, Hamming, unknown, and multi-Tag rejection.
- Three-frame confirmation and publication rate limiting.
- Tag loss and pose-jump reset.
- Task-state mapping from DockRobot feedback and result codes.
- Guard denial, cancellation, and duplicate start suppression.

Manual Gazebo acceptance covers:

1. Successful navigation, detection, and docking.
2. Covered Tag causing detection retry and timeout.
3. Wrong Tag causing `UNKNOWN_TAG` without motion into the dock.
4. Two visible Tags causing `MULTI_TAG` rejection.
5. Guard denial before start and Guard cancellation during final approach.

## Deliberate Exclusions

- Real battery current, charging contacts, and motor-stall detection.
- A custom Nav2 Dock Plugin.
- Isaac ROS or Jetson acceleration.
- A multi-dock operator UI or custom ROS Action definition.
- Production safety certification or obstacle-specific recovery tuning.

These are not required to prove the Demo 2 architecture and can be added after the simulated path is stable.
