# README Demo Video Design

## Goal

Publish a concise GitHub README video that proves the AprilTag docking demo runs in ROS 2 Jazzy and Gazebo and reaches `SUCCEEDED`.

## Presentation

- Use a 1280x720, silent, English video lasting about 45-60 seconds.
- Show the live robot camera as the primary view.
- Add a restrained telemetry panel with Tag ID, Hamming distance, decision margin, Tag state, docking state, and a live robot trajectory.
- Preserve real loss, confirmation, retry, and recovery states when they occur.
- End only after `/demo2/docking_state` reports `SUCCEEDED`.

## Data Flow

A temporary ROS 2 recorder subscribes to the camera, AprilTag detections, odometry, Tag state, and docking state. It starts docking after the Tag bridge and Docking Action server are ready, draws the latest telemetry on each received camera frame, and stops shortly after success.

The recorder must not synthesize detections, poses, states, or success. Missing data is shown as unavailable rather than replaced with scripted values.

## Outputs

- `docs/demo/apriltag_docking_demo.mp4`: compressed full recording.
- `docs/demo/apriltag_docking_demo.png`: README poster frame.
- `docs/demo/apriltag_docking_demo.gif`: short inline preview when the size remains practical.
- README section linking the poster or preview to the MP4.

Keep generated media small enough for normal GitHub use. Do not commit temporary recording scripts, raw frame dumps, logs, or ROS build output.

## Verification

- Confirm the recorded run contains a real Tag ID 0 detection and final `SUCCEEDED` state.
- Inspect the first, middle, and final frames for readable English text and correct layout.
- Verify the MP4 opens, has the expected duration and resolution, and remains within GitHub's file limit.
- Run the existing ROS 2 test suite before pushing the media and README update.
