# README Demo Video Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Record a real AprilTag docking run, publish compact README media, and push it to GitHub.

**Architecture:** A temporary ROS 2 Python recorder subscribes to the existing camera, detection, odometry, and state topics. It renders live data into a 1280x720 OpenCV video, requests docking only after the lifecycle node is active, and exits only on `SUCCEEDED`; only generated media and README changes remain in the repository.

**Tech Stack:** ROS 2 Jazzy, Gazebo Harmonic, rclpy, cv_bridge, OpenCV, GitHub Markdown

---

### Task 1: Validate The Recording Runtime

**Files:**
- Create temporarily: `.tmp_record_demo.py`
- Create temporarily: `.tmp_run_recording.sh`

- [ ] **Step 1: Verify required Python APIs**

Run a short Python import check after sourcing Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
python3 -c "import cv2, rclpy; from cv_bridge import CvBridge"
```

Expected: exit code 0.

- [ ] **Step 2: Verify MP4 encoding before launching Gazebo**

Create a 30-frame `1280x720` color-bar clip using `cv2.VideoWriter` with `mp4v`, reopen it with `cv2.VideoCapture`, and assert its width, height, and frame count.

Expected: the clip opens and reports 1280, 720, and 30 frames.

- [ ] **Step 3: Remove the preflight clip**

Delete only the generated preflight file after the assertions pass.

### Task 2: Record A Real Docking Run

**Files:**
- Create temporarily: `.tmp_record_demo.py`
- Create temporarily: `.tmp_run_recording.sh`
- Create: `docs/demo/apriltag_docking_demo.mp4`
- Create: `docs/demo/apriltag_docking_demo.png`

- [ ] **Step 1: Implement the temporary recorder**

The recorder must use these live subscriptions and clients:

```python
subscriptions = {
    '/camera/image_raw': Image,
    '/apriltag/detections': AprilTagDetectionArray,
    '/odom': Odometry,
    '/demo2/tag_state': String,
    '/demo2/docking_state': String,
}
start_client = node.create_client(Trigger, '/demo2/start_docking')
state_client = node.create_client(GetState, '/docking_server/get_state')
```

It must request docking only when the Tag state is `ACCEPTED`, a dock pose has been observed, the Docking Action server is ready, and the lifecycle state is `active`.

- [ ] **Step 2: Render the evidence layout**

For every camera frame, scale the live image into the left side of a `1280x720` canvas. Render an English right-side panel containing `TAG DETECTION`, `DOCKING PIPELINE`, Tag ID, Hamming distance, decision margin, Tag state, docking state, elapsed time, and a bounded odometry trajectory.

Use green only for accepted/success states, amber for confirmation/retry states, and red for failure/loss states. Do not replace missing values with fabricated data.

- [ ] **Step 3: Run the headless simulation and recorder**

```bash
source /opt/ros/jazzy/setup.bash
source /home/qucha/demo2_fix_ws_20260718/install/setup.bash
ros2 launch demo2_apriltag_docking demo.launch.py headless:=true rviz:=false
```

Run the recorder in the same sourced environment. Expected terminal evidence includes a real Tag ID 0 detection, `DOCKING_REQUESTED`, recovery states when they occur, and final `E2E_SUCCEEDED`.

- [ ] **Step 4: Save the poster**

Choose a late frame that shows `SUCCEEDED`, write it as `docs/demo/apriltag_docking_demo.png`, and retain two seconds of successful video before closing the writer.

- [ ] **Step 5: Clean recording processes and temporary scripts**

Stop only the launch and Gazebo processes created by this run. Delete `.tmp_record_demo.py`, `.tmp_run_recording.sh`, raw frames, and logs.

### Task 3: Verify And Publish The Media

**Files:**
- Modify: `README.md`
- Verify: `docs/demo/apriltag_docking_demo.mp4`
- Verify: `docs/demo/apriltag_docking_demo.png`

- [ ] **Step 1: Inspect the generated media programmatically**

Use `cv2.VideoCapture` to assert 1280x720 resolution, a duration between 20 and 90 seconds, and at least one readable frame. Confirm the MP4 is below GitHub's 100 MB file limit.

- [ ] **Step 2: Inspect representative frames visually**

Extract the first, middle, and final frames and view them. Confirm the camera is not blank, telemetry text fits, the trajectory remains inside its viewport, and the final frame reads `SUCCEEDED`.

- [ ] **Step 3: Add the README demo section**

Insert this concise section after the architecture description:

```markdown
## Demo Video

[![AprilTag docking demo](docs/demo/apriltag_docking_demo.png)](docs/demo/apriltag_docking_demo.mp4)

The recording shows a live Gazebo camera feed, AprilTag validation, Nav2 staging, visual recovery, and a successful docking result.
```

- [ ] **Step 4: Run repository verification**

Run `colcon test`, `colcon test-result --verbose`, `gz sdf -k`, and `git diff --check`.

Expected: 51 tests, no errors or failures, valid SDF, and a clean diff check.

- [ ] **Step 5: Commit and push**

```bash
git add README.md docs/demo
git commit -m "docs: add AprilTag docking demo video"
git push origin feat/docking-review-fixes
```
