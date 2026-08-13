#include "demo2_apriltag_docking_cpp/adapters/tag_pose_adapter.hpp"

#include <diagnostic_msgs/msg/key_value.hpp>

#include <charconv>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <system_error>

namespace demo2_apriltag_docking_cpp::adapters {
namespace {

double round_three(double value)
{
  return std::nearbyint(value * 1000.0) / 1000.0;
}

}  // namespace

ReportDecision ReportGate::evaluate(
  const std::string & state,
  double now,
  std::optional<double> refresh_seconds)
{
  const bool repeated = last_state_.has_value() && *last_state_ == state;
  const bool refresh_due = refresh_seconds.has_value() &&
    last_diagnostic_time_.has_value() &&
    now - *last_diagnostic_time_ >= *refresh_seconds;
  if (repeated && !refresh_due) {
    return {false, false};
  }

  if (!repeated) {
    last_state_ = state;
  }
  last_diagnostic_time_ = now;
  return {!repeated, true};
}

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::string diagnostic_number(double value)
{
  char buffer[64];
  const auto result = std::to_chars(
    std::begin(buffer), std::end(buffer), value, std::chars_format::general);
  if (result.ec != std::errc()) {
    throw std::runtime_error("failed to format diagnostic number");
  }
  std::string text(buffer, result.ptr);
  if (std::isfinite(value) && text.find_first_of(".eE") == std::string::npos) {
    text += ".0";
  }
  return text;
}

DiagnosticValues temporal_values(const core::TemporalQuality & timing)
{
  DiagnosticValues values;
  if (timing.detection_age_ms.has_value()) {
    values.emplace_back(
      "detection_age_ms", diagnostic_number(round_three(*timing.detection_age_ms)));
  }
  if (timing.tf_age_ms.has_value()) {
    values.emplace_back("tf_age_ms", diagnostic_number(round_three(*timing.tf_age_ms)));
  }
  return values;
}

core::CameraCalibration camera_info_to_calibration(
  const sensor_msgs::msg::CameraInfo & message)
{
  return core::validate_camera_calibration(
    static_cast<int>(message.width),
    static_cast<int>(message.height),
    message.k,
    message.distortion_model);
}

geometry_msgs::msg::PoseStamped to_pose_message(
  const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

core::Detection metadata_only(
  const apriltag_msgs::msg::AprilTagDetection & detection,
  double stamp)
{
  return {
    detection.id,
    detection.hamming,
    static_cast<double>(detection.decision_margin),
    stamp,
    0.0,
    0.0,
    0.0,
  };
}

core::Detection to_policy_detection(
  const apriltag_msgs::msg::AprilTagDetection & detection,
  const geometry_msgs::msg::TransformStamped & transform,
  double stamp)
{
  const auto & rotation = transform.transform.rotation;
  return {
    detection.id,
    detection.hamming,
    static_cast<double>(detection.decision_margin),
    stamp,
    transform.transform.translation.x,
    transform.transform.translation.y,
    core::yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
  };
}

diagnostic_msgs::msg::DiagnosticStatus make_status(
  const std::string & node_name,
  std::uint8_t level,
  const std::string & state,
  const DiagnosticValues & values)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = node_name;
  status.hardware_id = "demo2_apriltag_docking";
  status.level = level;
  status.message = state;
  status.values.reserve(values.size());
  for (const auto & [key, value] : values) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(std::move(item));
  }
  return status;
}

}  // namespace demo2_apriltag_docking_cpp::adapters
