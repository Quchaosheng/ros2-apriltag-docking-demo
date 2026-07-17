# AprilTag Visual Docking Demo Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Gazebo demo in which TurtleBot3 navigates to a charging-dock staging pose, validates an AprilTag detection, and completes the final approach through Nav2 Docking.

**Architecture:** One `ament_python` package owns a pure Tag validation policy, an AprilTag-to-`PoseStamped` bridge, and a Guard/Action/Monitor task bridge. The package reuses `apriltag_ros` for vision and `opennav_docking::SimpleChargingDock` for staging navigation, pose filtering, retries, control, timeout handling, and simulated docking completion.

**Tech Stack:** Ubuntu 24.04, ROS 2 Jazzy, Python 3, `rclpy`, `apriltag_ros`, `apriltag_msgs`, Nav2, `opennav_docking`, TurtleBot3, Gazebo Harmonic, `pytest`, `ament_pytest`.

---

## File Map

Create one ROS package under `src/demo2_apriltag_docking`.

- `package.xml`: runtime and test dependencies.
- `setup.py`, `setup.cfg`, `resource/demo2_apriltag_docking`: `ament_python` packaging and console entry points.
- `demo2_apriltag_docking/tag_policy.py`: ROS-independent mapping, confidence, debounce, rate-limit, loss, and jump rules.
- `demo2_apriltag_docking/tag_pose_bridge.py`: AprilTag detections plus TF to `/detected_dock_pose`.
- `demo2_apriltag_docking/docking_task_bridge.py`: Guard input, start/cancel services, DockRobot Action client, and monitor output.
- `demo2_apriltag_docking/monitor.py`: shared state names and diagnostic message construction.
- `config/docks.yaml`: Tag ID to dock ID/type/frame mapping.
- `config/apriltag.yaml`: detector family, size, Hamming, and pose-estimation configuration.
- `config/nav2_docking.yaml`: Docking Server and `SimpleChargingDock` parameters.
- `config/dock_database.yaml`: approximate fixed-frame dock pose used for staging navigation.
- `launch/demo.launch.py`: Gazebo, TurtleBot3, Nav2, AprilTag, bridges, and Docking Server composition.
- `models/apriltag_dock/model.config`, `models/apriltag_dock/model.sdf`: charging-dock visual model.
- `models/apriltag_dock/materials/textures/tag36_11_00000.png`: official AprilTag texture.
- `worlds/docking_demo.sdf`: small unobstructed test world.
- `test/test_tag_policy.py`: pure policy unit tests.
- `test/test_task_policy.py`: pure task/Guard state tests.
- `test/test_config.py`: mapping and YAML contract tests.
- `README.md`: installation, launch, controls, expected topics, and failure demos.

## Task 1: Create the ROS 2 Package Skeleton

**Files:**
- Create: `src/demo2_apriltag_docking/package.xml`
- Create: `src/demo2_apriltag_docking/setup.py`
- Create: `src/demo2_apriltag_docking/setup.cfg`
- Create: `src/demo2_apriltag_docking/resource/demo2_apriltag_docking`
- Create: `src/demo2_apriltag_docking/demo2_apriltag_docking/__init__.py`

- [ ] **Step 1: Create the package directories**

Run:

```bash
mkdir -p src/demo2_apriltag_docking/{demo2_apriltag_docking,resource,config,launch,models,worlds,test}
```

Expected: all directories exist and `git status --short` only shows new project files.

- [ ] **Step 2: Add package metadata**

Use package name `demo2_apriltag_docking`, version `0.1.0`, and Apache-2.0. Declare these runtime dependencies:

```xml
<exec_depend>apriltag_msgs</exec_depend>
<exec_depend>apriltag_ros</exec_depend>
<exec_depend>diagnostic_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<exec_depend>nav2_msgs</exec_depend>
<exec_depend>opennav_docking</exec_depend>
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>std_srvs</exec_depend>
<exec_depend>tf2_ros</exec_depend>
<test_depend>ament_pytest</test_depend>
<test_depend>python3-pytest</test_depend>
<test_depend>python3-yaml</test_depend>
```

- [ ] **Step 3: Register console entry points and data files**

Add exactly these entry points in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'tag_pose_bridge = demo2_apriltag_docking.tag_pose_bridge:main',
        'docking_task_bridge = demo2_apriltag_docking.docking_task_bridge:main',
    ],
},
```

Install `config`, `launch`, `models`, and `worlds` recursively so `get_package_share_directory()` can locate every runtime asset.

- [ ] **Step 4: Build the empty package**

Run:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select demo2_apriltag_docking
```

Expected: package builds without errors and installs its share directory.

- [ ] **Step 5: Commit the package skeleton**

```bash
git add src/demo2_apriltag_docking
git commit -m "build: add ROS 2 docking demo package"
```

## Task 2: Implement Tag Mapping and Detection Policy with TDD

**Files:**
- Create: `src/demo2_apriltag_docking/demo2_apriltag_docking/tag_policy.py`
- Create: `src/demo2_apriltag_docking/test/test_tag_policy.py`

- [ ] **Step 1: Write failing tests for mapping and hard rejection**

Define the public types used by the tests:

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class DockSpec:
    tag_id: int
    dock_id: str
    dock_type: str
    tag_frame: str

@dataclass(frozen=True)
class Detection:
    tag_id: int
    hamming: int
    decision_margin: float
    stamp: float
    x: float
    y: float
    yaw: float
```

Add tests asserting these exact outcomes:

```python
assert gate.evaluate([], now=1.0).reason == 'NO_TAG'
assert gate.evaluate([unknown], now=1.0).reason == 'UNKNOWN_TAG'
assert gate.evaluate([low_margin], now=1.0).reason == 'LOW_MARGIN'
assert gate.evaluate([bad_hamming], now=1.0).reason == 'HAMMING'
assert gate.evaluate([valid, second], now=1.0).reason == 'MULTI_TAG'
```

- [ ] **Step 2: Run the tests and verify they fail**

Run:

```bash
pytest -q src/demo2_apriltag_docking/test/test_tag_policy.py
```

Expected: collection fails because `tag_policy` does not exist.

- [ ] **Step 3: Implement mapping validation and hard rejection**

Implement `load_dock_specs(raw: dict) -> dict[int, DockSpec]` and `TagGate.evaluate(detections, now)`. Reject missing fields, duplicate Tag IDs, duplicate dock IDs, empty strings, negative Tag IDs, and non-numeric thresholds with `ValueError`.

Use a result object with this stable API:

```python
@dataclass(frozen=True)
class GateResult:
    accepted: bool
    reason: str
    detection: Detection | None = None
    dock: DockSpec | None = None
```

- [ ] **Step 4: Add failing tests for debounce, duplicate suppression, jump, and loss**

Test these configured values:

```python
gate = TagGate(
    specs={0: dock},
    min_margin=50.0,
    max_hamming=0,
    confirmations=3,
    confirmation_window=0.5,
    publish_period=0.1,
    loss_timeout=0.5,
    max_translation_jump=0.25,
    max_yaw_jump=0.349066,
)
```

Assert that the first two valid samples return `CONFIRMING`, the third returns `ACCEPTED`, a sample at 0.05 seconds returns `RATE_LIMITED`, a 0.30 m jump returns `POSE_JUMP`, and `gate.loss_reason(now=last_stamp + 0.51)` returns `TAG_LOST`.

- [ ] **Step 5: Run the focused tests and verify they fail**

Run:

```bash
pytest -q src/demo2_apriltag_docking/test/test_tag_policy.py -k "debounce or rate or jump or loss"
```

Expected: failures show the temporal rules are not implemented.

- [ ] **Step 6: Implement the minimum temporal state**

Store only the active Tag ID, confirmation count, first confirmation time, last accepted detection, last publication time, and last seen time. Normalize yaw deltas into `[-pi, pi]`. Reset confirmation after an ID change, window expiry, hard rejection, or pose jump.

- [ ] **Step 7: Run the policy tests**

Run:

```bash
pytest -q src/demo2_apriltag_docking/test/test_tag_policy.py
```

Expected: all tests pass.

- [ ] **Step 8: Commit the policy**

```bash
git add src/demo2_apriltag_docking/demo2_apriltag_docking/tag_policy.py src/demo2_apriltag_docking/test/test_tag_policy.py
git commit -m "feat: add AprilTag docking validation policy"
```

## Task 3: Implement the AprilTag Pose Bridge

**Files:**
- Create: `src/demo2_apriltag_docking/demo2_apriltag_docking/monitor.py`
- Create: `src/demo2_apriltag_docking/demo2_apriltag_docking/tag_pose_bridge.py`
- Modify: `src/demo2_apriltag_docking/setup.py`

- [ ] **Step 1: Add diagnostic helpers**

Implement one helper:

```python
def make_status(node_name: str, level: int, message: str, values: dict[str, object]):
    status = DiagnosticStatus()
    status.name = node_name
    status.hardware_id = 'demo2_apriltag_docking'
    status.level = level
    status.message = message
    status.values = [KeyValue(key=str(k), value=str(v)) for k, v in values.items()]
    return status
```

Keep state transition deduplication in the caller; the helper only builds messages.

- [ ] **Step 2: Implement node parameters and interfaces**

`TagPoseBridge` must declare these parameters with these defaults:

```yaml
dock_mapping_file: ""
detections_topic: /apriltag/detections
output_pose_topic: /detected_dock_pose
state_topic: /demo2/tag_state
min_decision_margin: 50.0
max_hamming: 0
confirmations: 3
confirmation_window: 0.5
publish_rate_hz: 10.0
loss_timeout: 0.5
max_translation_jump: 0.25
max_yaw_jump_deg: 20.0
tf_timeout: 0.2
```

Create subscriptions/publishers for `AprilTagDetectionArray`, `PoseStamped`, `String`, and `DiagnosticArray`. Use reliable depth 10 for detections and pose, and transient-local reliable depth 1 for state.

- [ ] **Step 3: Convert detections without re-estimating pose**

For each `AprilTagDetectionArray` callback:

1. Convert metadata into policy candidates.
2. Use the configured `tag_frame` for the one candidate.
3. Look up `header.frame_id -> tag_frame` at the image timestamp with `tf2_ros.Buffer.lookup_transform()`.
4. Convert the transform into `Detection.x`, `y`, and yaw.
5. Pass the candidate to `TagGate`.
6. Publish a `PoseStamped` only for `ACCEPTED`.

The published pose header must retain the camera/image frame and timestamp so `SimpleChargingDock` can transform it into its fixed frame.

- [ ] **Step 4: Add loss and state timers**

Run a 10 Hz timer. If `TagGate.loss_reason()` changes to `TAG_LOST`, publish that state and one warning diagnostic. Do not republish an old pose. Publish state and diagnostic messages only when reason changes, except accepted diagnostics may refresh once per second.

- [ ] **Step 5: Handle TF and configuration failures**

Report `TF_UNAVAILABLE` when the Tag transform is missing at the image timestamp. Treat malformed mapping YAML as a startup error and call `rclpy.shutdown()` after logging the exception. Never fall back to the latest TF silently because it can create pose jumps during robot motion.

- [ ] **Step 6: Build and lint the node**

Run:

```bash
python3 -m py_compile src/demo2_apriltag_docking/demo2_apriltag_docking/*.py
colcon build --symlink-install --packages-select demo2_apriltag_docking
```

Expected: compilation and package build pass.

- [ ] **Step 7: Commit the pose bridge**

```bash
git add src/demo2_apriltag_docking/demo2_apriltag_docking src/demo2_apriltag_docking/setup.py
git commit -m "feat: bridge validated AprilTags to dock poses"
```

## Task 4: Implement Guard, Action, and Monitor Integration

**Files:**
- Create: `src/demo2_apriltag_docking/demo2_apriltag_docking/docking_task_bridge.py`
- Create: `src/demo2_apriltag_docking/test/test_task_policy.py`

- [ ] **Step 1: Write failing pure-state tests**

Extract a ROS-independent `TaskPolicy` in `docking_task_bridge.py` with inputs for guard state, guard timestamp, action-active state, and current time. Test:

```python
assert policy.can_start(now=1.0) == (False, 'GUARD_DENIED')
assert allowed_policy.can_start(now=1.0) == (True, 'READY')
assert active_policy.can_start(now=1.0) == (False, 'ALREADY_ACTIVE')
assert stale_guard_policy.can_start(now=3.1) == (False, 'GUARD_STALE')
```

Also test the exact DockRobot feedback mapping:

```python
{
    1: 'NAV_TO_STAGING',
    2: 'INITIAL_PERCEPTION',
    3: 'CONTROLLING',
    4: 'WAIT_FOR_CHARGE',
    5: 'RETRY',
}
```

- [ ] **Step 2: Run tests and verify failure**

Run:

```bash
pytest -q src/demo2_apriltag_docking/test/test_task_policy.py
```

Expected: failure because `TaskPolicy` is missing.

- [ ] **Step 3: Implement the pure task policy**

Use fail-safe behavior only when `guard_required` is true. A required Guard is allowed only when its last message is true and no older than `guard_timeout`.

- [ ] **Step 4: Implement ROS interfaces**

Declare these parameters:

```yaml
dock_mapping_file: ""
target_tag_id: 0
dock_action_name: /dock_robot
start_service: /demo2/start_docking
cancel_service: /demo2/cancel_docking
guard_topic: /guard/docking_allowed
guard_required: false
guard_timeout: 2.0
state_topic: /demo2/docking_state
max_staging_time: 60.0
navigate_to_staging_pose: true
```

Create a `DockRobot` Action client, Trigger services, Guard subscription, transient-local state publisher, and diagnostics publisher.

- [ ] **Step 5: Send and monitor DockRobot goals**

Resolve `target_tag_id` to its `DockSpec` and send:

```python
goal = DockRobot.Goal()
goal.use_dock_id = True
goal.dock_id = dock.dock_id
goal.max_staging_time = self.max_staging_time
goal.navigate_to_staging_pose = self.navigate_to_staging_pose
```

Map feedback states through the tested dictionary. On result, publish `SUCCEEDED`, `CANCELED`, or `FAILED` plus `error_code`, `error_msg`, and `num_retries` in diagnostics.

- [ ] **Step 6: Enforce Guard and duplicate-request behavior**

Reject start requests unless `TaskPolicy.can_start()` returns true. When a required Guard changes from true to false during an active goal, call `cancel_goal_async()` once and publish `GUARD_DENIED`. Reject repeated start requests while the current goal future or goal handle is active.

- [ ] **Step 7: Run tests and build**

Run:

```bash
pytest -q src/demo2_apriltag_docking/test/test_task_policy.py
colcon build --symlink-install --packages-select demo2_apriltag_docking
```

Expected: tests and build pass.

- [ ] **Step 8: Commit task integration**

```bash
git add src/demo2_apriltag_docking/demo2_apriltag_docking/docking_task_bridge.py src/demo2_apriltag_docking/test/test_task_policy.py
git commit -m "feat: integrate docking guard action and monitor"
```

## Task 5: Configure AprilTag and Nav2 Docking

**Files:**
- Create: `src/demo2_apriltag_docking/config/docks.yaml`
- Create: `src/demo2_apriltag_docking/config/apriltag.yaml`
- Create: `src/demo2_apriltag_docking/config/nav2_docking.yaml`
- Create: `src/demo2_apriltag_docking/config/dock_database.yaml`
- Create: `src/demo2_apriltag_docking/test/test_config.py`

- [ ] **Step 1: Write failing configuration-contract tests**

Load all YAML files with `yaml.safe_load()` and assert:

```python
assert docks['docks'][0]['dock_id'] == 'demo_charge_dock'
assert docks['docks'][0]['dock_type'] == 'charging_dock'
assert docks['docks'][0]['tag_frame'] == 'tag36h11:0'
assert apriltag['apriltag']['ros__parameters']['family'] == '36h11'
assert docking['docking_server']['ros__parameters']['max_retries'] == 2
assert database['docks']['demo_charge_dock']['type'] == 'charging_dock'
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
pytest -q src/demo2_apriltag_docking/test/test_config.py
```

Expected: failure because YAML files do not exist.

- [ ] **Step 3: Add the Tag mapping and detector configuration**

Use this mapping:

```yaml
docks:
  0:
    dock_id: demo_charge_dock
    dock_type: charging_dock
    tag_frame: tag36h11:0
```

Configure `apriltag_ros` with `family: 36h11`, `size: 0.16`, `max_hamming: 0`, `pose_estimation_method: pnp`, and no `tag.ids` allow-list so the bridge can explicitly report unknown Tags.

- [ ] **Step 4: Configure Docking Server and built-in plugin**

Use these demo values under `docking_server.ros__parameters`:

```yaml
controller_frequency: 20.0
initial_perception_timeout: 5.0
wait_charge_timeout: 3.0
dock_approach_timeout: 30.0
rotate_to_dock_timeout: 10.0
max_retries: 2
base_frame: base_link
fixed_frame: odom
dock_plugins: [charging_dock]
charging_dock:
  plugin: opennav_docking::SimpleChargingDock
  dock_direction: forward
  rotate_to_dock: false
  use_external_detection_pose: true
  external_detection_timeout: 0.8
  external_detection_translation_x: -0.20
  external_detection_translation_y: 0.0
  external_detection_rotation_yaw: 0.0
  external_detection_rotation_pitch: 1.57
  external_detection_rotation_roll: -1.57
  filter_coef: 0.2
  staging_x_offset: -0.7
  staging_yaw_offset: 0.0
  use_battery_status: false
  docking_threshold: 0.05
```

Remap the plugin's relative `detected_dock_pose` subscription to `/detected_dock_pose` in launch.

- [ ] **Step 5: Add the approximate dock database**

Use Nav2's native database schema and place `demo_charge_dock` at `(x=2.0, y=0.0, yaw=0.0)` in `odom`:

```yaml
docks:
  demo_charge_dock:
    type: charging_dock
    frame: odom
    pose: [2.0, 0.0, 0.0]
```

This pose is the global estimate used only to compute the staging pose; the camera pose refines the final approach. Inject the installed file path into the Docking Server's `dock_database` parameter from the launch file.

- [ ] **Step 6: Run configuration tests**

Run:

```bash
pytest -q src/demo2_apriltag_docking/test/test_config.py
```

Expected: all configuration contract tests pass.

- [ ] **Step 7: Commit configuration**

```bash
git add src/demo2_apriltag_docking/config src/demo2_apriltag_docking/test/test_config.py
git commit -m "config: define AprilTag charging dock demo"
```

## Task 6: Add the Gazebo World, Dock Model, and Unified Launch

**Files:**
- Create: `src/demo2_apriltag_docking/models/apriltag_dock/model.config`
- Create: `src/demo2_apriltag_docking/models/apriltag_dock/model.sdf`
- Create: `src/demo2_apriltag_docking/models/apriltag_dock/materials/textures/tag36_11_00000.png`
- Create: `src/demo2_apriltag_docking/worlds/docking_demo.sdf`
- Create: `src/demo2_apriltag_docking/launch/demo.launch.py`

- [ ] **Step 1: Add the official Tag texture**

Run:

```bash
curl -L https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_00000.png \
  -o src/demo2_apriltag_docking/models/apriltag_dock/materials/textures/tag36_11_00000.png
```

Expected: `file` reports a PNG image and Git shows one new binary asset.

- [ ] **Step 2: Create the dock model**

Build a static SDF model with a 0.40 m wide, 0.30 m high, 0.03 m thick plate. Put a 0.16 m square textured visual on its robot-facing side, centered 0.18 m above the floor. Use a plain dark-gray material for the plate and an unlit material for the Tag so Gazebo lighting does not wash out the black/white border.

- [ ] **Step 3: Create the world**

Use an empty 6 m by 4 m floor, normal lighting, the dock at `(2.0, 0.0, 0.0)`, and the robot start pose near `(0.0, 0.0, 0.0)`. Keep the first demo free of obstacles so failures are attributable to perception or docking.

- [ ] **Step 4: Create the unified launch file**

Declare these launch arguments:

```text
use_sim_time:=true
target_tag_id:=0
guard_required:=false
headless:=false
rviz:=true
```

Set `TURTLEBOT3_MODEL=waffle_pi`, launch Gazebo, spawn TurtleBot3 Waffle Pi with its RGB camera, start Nav2, start `apriltag_ros` with image and CameraInfo remaps, start `opennav_docking`, and start both bridge nodes. Add a `TimerAction` only where an upstream service must be available before the dependent node starts; do not use sleeps inside nodes.

- [ ] **Step 5: Verify the ROS graph before docking**

Run:

```bash
ros2 launch demo2_apriltag_docking demo.launch.py
ros2 topic list | grep -E "apriltag|detected_dock_pose|demo2|diagnostics"
ros2 action list | grep dock_robot
ros2 service list | grep /demo2
```

Expected: detections, pose, state, diagnostics, DockRobot Action, and start/cancel services are present.

- [ ] **Step 6: Verify live visual detection**

Run:

```bash
ros2 topic echo /apriltag/detections --once
ros2 topic echo /detected_dock_pose --once
```

Expected: Tag ID 0 has `hamming: 0`, margin above 50, and a pose is published after three consistent frames.

- [ ] **Step 7: Commit simulation and launch assets**

```bash
git add src/demo2_apriltag_docking/models src/demo2_apriltag_docking/worlds src/demo2_apriltag_docking/launch
git commit -m "feat: add Gazebo AprilTag docking scenario"
```

## Task 7: Run End-to-End Acceptance and Document the Demo

**Files:**
- Create: `README.md`
- Modify: `src/demo2_apriltag_docking/launch/demo.launch.py` only if acceptance exposes launch defects.

- [ ] **Step 1: Run the complete automated suite**

Run:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Expected: build succeeds, all tests pass, and `colcon test-result` reports zero failures.

- [ ] **Step 2: Verify successful docking**

With the demo launch running, call:

```bash
ros2 service call /demo2/start_docking std_srvs/srv/Trigger "{}"
ros2 topic echo /demo2/docking_state
```

Expected state sequence: `NAV_TO_STAGING`, `INITIAL_PERCEPTION`, `CONTROLLING`, `WAIT_FOR_CHARGE`, `SUCCEEDED`. The robot stops at the dock and the Action result has `success: true`.

- [ ] **Step 3: Verify low-confidence or lost-Tag handling**

Cover or rotate the Tag away before starting. Call the start service again.

Expected: `/demo2/tag_state` reports `NO_TAG` or `TAG_LOST`; Nav2 reports retries and ultimately `FAILED` with `FAILED_TO_DETECT_DOCK` or `TIMEOUT`. No stale pose continues to publish.

- [ ] **Step 4: Verify unknown and multiple Tag rejection**

Temporarily place ID 1 in front of the camera, then place IDs 0 and 1 together.

Expected: the state reports `UNKNOWN_TAG` for ID 1 and `MULTI_TAG` for the pair. `/detected_dock_pose` remains silent for both cases.

- [ ] **Step 5: Verify Guard behavior**

Launch with `guard_required:=true`. Publish false and attempt to start:

```bash
ros2 topic pub --once --qos-durability transient_local /guard/docking_allowed std_msgs/msg/Bool "{data: false}"
ros2 service call /demo2/start_docking std_srvs/srv/Trigger "{}"
```

Expected: request fails with `GUARD_DENIED` and no Action goal starts. Publish true, start docking, then publish false during `CONTROLLING`; expected result is one Action cancellation and state `GUARD_DENIED`.

- [ ] **Step 6: Write the README**

Document:

- Supported platform and dependency installation.
- `rosdep`, `colcon build`, and launch commands.
- Start and cancel service commands.
- Guard topic behavior.
- Tag mapping and tunable thresholds.
- Expected success state sequence.
- Five failure demonstrations and their expected monitor output.
- A short architecture diagram matching the design document.

- [ ] **Step 7: Re-run verification after documentation and launch cleanup**

Run:

```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
git status --short
```

Expected: zero test failures and only intended README or launch changes remain.

- [ ] **Step 8: Commit the completed demo documentation**

```bash
git add README.md src/demo2_apriltag_docking/launch/demo.launch.py
git commit -m "docs: add AprilTag docking demo guide"
```

## Final Acceptance Criteria

- A single launch command starts Gazebo, TurtleBot3, Nav2, AprilTag detection, Docking Server, and both bridges.
- The robot navigates to the staging pose and docks using live camera detections.
- Tag ID maps deterministically to one dock ID and dock type.
- Low-confidence, unknown, and multiple Tags never produce a dock pose.
- Three-frame confirmation, rate limiting, Tag loss, and pose-jump behavior are observable and tested.
- Docking retries and timeouts come from Nav2 and are relayed to monitor diagnostics.
- Guard denial prevents start and cancels an active DockRobot goal.
- `colcon test-result --verbose` reports zero failures.
