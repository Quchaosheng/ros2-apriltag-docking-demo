# Docking Reliability Design

## Goal

Make the Gazebo docking demo reproducible and materially reduce end-of-approach AprilTag loss without increasing the retry count or bypassing visual docking.

## Changes

- Start Gazebo with a fixed random seed of `42`.
- Keep Nav2 staging, external AprilTag perception, three-frame confirmation, and `max_retries: 2` enabled.
- Increase the distance-based simulated docking threshold from `0.05 m` to the smallest value in the `0.10-0.15 m` range that completes reliably before the 0.16 m Tag leaves the camera field of view.
- Keep the existing `-0.20 m` detection translation offset, so the threshold remains relative to the simulated robot-to-dock contact pose rather than the Tag plane itself.
- Require three consecutive cold-start runs to finish with `SUCCEEDED` before accepting the tuning.
- Remove internal `docs/superpowers/` planning artifacts from the final public tree.

## Boundaries

Do not increase `max_retries`, disable staging navigation, synthesize detections, publish ground-truth dock poses, or weaken Tag confidence and confirmation gates. Do not add a permanent recorder or test harness unless the existing launch and temporary monitor cannot provide reliable evidence.

## Verification

- Add configuration and launch tests for the selected threshold and fixed seed.
- Run the full Jazzy test suite and SDF validation.
- Run three consecutive headless Gazebo cold starts with real camera detections and require final `SUCCEEDED` each time.
- Confirm no demo processes or temporary scripts remain, then push the clean branch.
