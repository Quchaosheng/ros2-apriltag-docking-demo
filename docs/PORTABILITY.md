# Portability

The tag policy and docking-gate code is Python and does not require a ROS 2
process to unit test. Keep this logic portable so it can be reused by a
simulator, a bag-file evaluator, or a platform adapter.

- Python policy tests: portable across Linux, WSL2, Windows, and macOS.
- ROS 2 integration and Nav2 launch tests: validated on the documented Linux
  ROS 2 environment.
- Camera, TF, Gazebo, and Nav2 adapters remain platform-dependent and should
  be tested in the target ROS 2 distribution.

When adding a platform adapter, preserve the `Detection`, `DockSpec`, and
`GateResult` contracts so the safety gate remains independently testable.
