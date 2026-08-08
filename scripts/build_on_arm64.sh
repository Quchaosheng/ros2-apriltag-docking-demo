#!/usr/bin/env bash
set -Eeuo pipefail

[[ "$(uname -m)" == "aarch64" || "$(uname -m)" == "arm64" ]] || {
  echo "This script must run on an ARM64 board (aarch64/arm64)." >&2
  exit 1
}

ros_distro=${ROS_DISTRO:-jazzy}
source "/opt/ros/${ros_distro}/setup.bash"
rosdep update --rosdistro "${ros_distro}" >/dev/null
rosdep install --from-paths src --ignore-src --rosdistro "${ros_distro}" -r -y
colcon build --packages-select demo2_apriltag_docking --symlink-install \
  --cmake-args -DBUILD_TESTING=ON
