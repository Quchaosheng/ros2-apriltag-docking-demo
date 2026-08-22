# C++ refactor

The online ROS 2 bridge nodes are implemented in C++ while the launch files,
Gazebo assets, topic names, parameters, and Nav2 docking configuration remain
unchanged.

## C++ nodes

- `tag_pose_bridge`: validates AprilTag observations, TF freshness, camera
  calibration, confirmation windows, pose jumps, and tag loss before publishing
  `/detected_dock_pose`.
- `docking_task_bridge`: owns the `DockRobot` action client, start/cancel
  services, Guard policy, feedback states, and diagnostics.

The policy classes do not depend on ROS 2. `TagGate`, `QualityPolicy`, and
`TaskPolicy` can therefore be tested on Windows with a standard C++17 toolchain.
ROS 2 adapters are tested in the documented Ubuntu 24.04 / ROS 2 Jazzy CI
environment.

## Build on Ubuntu

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select demo2_apriltag_docking --symlink-install
colcon test --packages-select demo2_apriltag_docking \
  --event-handlers console_direct+ --return-code-on-test-failure
```

The existing `demo.launch.py` starts the C++ executables using the same
executable names as the original Python entry points. The offline
`analyze_docking_bag` utility remains Python because it is a rosbag reporting
tool and is independent of the live control path.
