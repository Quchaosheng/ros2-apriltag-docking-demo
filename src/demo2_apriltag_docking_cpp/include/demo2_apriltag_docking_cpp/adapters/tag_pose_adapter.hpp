#pragma once

#include "demo2_apriltag_docking_cpp/core/quality_policy.hpp"
#include "demo2_apriltag_docking_cpp/core/tag_policy.hpp"

#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace demo2_apriltag_docking_cpp::adapters {

using DiagnosticValues = std::vector<std::pair<std::string, std::string>>;

struct ReportDecision {
  bool publish_state;
  bool publish_diagnostic;
};

class ReportGate {
public:
  ReportDecision evaluate(
    const std::string & state,
    double now,
    std::optional<double> refresh_seconds = std::nullopt);

private:
  std::optional<std::string> last_state_;
  std::optional<double> last_diagnostic_time_;
};

double stamp_seconds(const builtin_interfaces::msg::Time & stamp);
std::string diagnostic_number(double value);
DiagnosticValues temporal_values(const core::TemporalQuality & timing);

core::CameraCalibration camera_info_to_calibration(
  const sensor_msgs::msg::CameraInfo & message);

geometry_msgs::msg::PoseStamped to_pose_message(
  const geometry_msgs::msg::TransformStamped & transform);

core::Detection metadata_only(
  const apriltag_msgs::msg::AprilTagDetection & detection,
  double stamp);

core::Detection to_policy_detection(
  const apriltag_msgs::msg::AprilTagDetection & detection,
  const geometry_msgs::msg::TransformStamped & transform,
  double stamp);

diagnostic_msgs::msg::DiagnosticStatus make_status(
  const std::string & node_name,
  std::uint8_t level,
  const std::string & state,
  const DiagnosticValues & values);

}  // namespace demo2_apriltag_docking_cpp::adapters
