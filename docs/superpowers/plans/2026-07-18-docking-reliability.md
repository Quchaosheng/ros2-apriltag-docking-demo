# Docking Reliability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the real Gazebo docking run deterministic and complete three consecutive cold starts without increasing retries.

**Architecture:** Keep the current Nav2 and AprilTag pipeline unchanged. Fix Gazebo's seed and stop the distance-based simulated docking slightly earlier, before the Tag grows beyond the camera view; validate with focused config tests and repeated real E2E runs.

**Tech Stack:** ROS 2 Jazzy, Gazebo Harmonic, Nav2 Docking, pytest, rclpy

---

### Task 1: Lock Deterministic Simulation Parameters

**Files:**
- Modify: `src/demo2_apriltag_docking/test/test_config.py`
- Modify: `src/demo2_apriltag_docking/test/test_simulation_assets.py`
- Modify: `src/demo2_apriltag_docking/config/nav2_docking.yaml`
- Modify: `src/demo2_apriltag_docking/config/turtlebot3_waffle_pi_nav2.yaml`
- Modify: `src/demo2_apriltag_docking/launch/demo.launch.py`

- [ ] Add failing assertions that both Docking Server configurations use `docking_threshold: 0.12`, preserve `max_retries: 2`, and that the launch file passes `--seed 42` to Gazebo.
- [ ] Run the focused tests and confirm they fail on the current `0.05` threshold and missing seed.
- [ ] Change only the two threshold values and the Gazebo server argument.
- [ ] Run the focused tests, full Python suite, and flake8 until clean.

### Task 2: Prove Repeated Real Docking

**Files:**
- Create temporarily: `.tmp_reliability_monitor.py`
- Create temporarily: `.tmp_run_reliability.sh`

- [ ] Sync the package into `/home/qucha/demo2_fix_ws_20260718`, rebuild it, and confirm all tests pass.
- [ ] Create a temporary persistent rclpy monitor using real detection, pose, lifecycle, Action, start-service, and docking-state interfaces.
- [ ] Launch with a fresh Gazebo process group, wait for readiness, request docking, and require `SUCCEEDED`.
- [ ] Repeat three cold starts consecutively. Record Tag ID, Hamming distance, margin, docking state sequence, retries or loss recovery, wall duration, and final result for each run.
- [ ] If any run fails, inspect its Docking Server log before changing one parameter at a time; do not increase retries or weaken perception gates.
- [ ] Delete the temporary monitor, logs, and failed media, and confirm no demo processes remain.

### Task 3: Clean And Publish The Public Repository

**Files:**
- Delete: `docs/superpowers/`

- [ ] Remove the internal planning/specification tree from the final public worktree.
- [ ] Run fresh `colcon test`, `colcon test-result --verbose`, `gz sdf -k`, and `git diff --check`.
- [ ] Review the complete diff for production-only scope and concise comments.
- [ ] Commit the reliability changes and documentation cleanup without rewriting prior history.
- [ ] Push through GitHub SSH over port 443, verify local and remote SHA equality, and leave the worktree clean.
