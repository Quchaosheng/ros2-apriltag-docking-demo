#include "demo2_apriltag_docking/quality_policy.hpp"

#include <cmath>
#include <stdexcept>

namespace demo2_apriltag_docking
{
namespace
{

bool finite_nonnegative(const double value)
{
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

TemporalQuality validate_observation_timing(
  const double now,
  const double detection_stamp,
  const double tf_stamp,
  const double max_detection_age,
  const double max_tf_age,
  const double max_future_skew)
{
  if (!finite_nonnegative(now) || !finite_nonnegative(detection_stamp) ||
    !finite_nonnegative(tf_stamp))
  {
    return {false, "INVALID_TIMESTAMP", std::nullopt, std::nullopt};
  }
  if (!finite_nonnegative(max_detection_age) || !finite_nonnegative(max_tf_age) ||
    !finite_nonnegative(max_future_skew))
  {
    throw std::invalid_argument("temporal limits must be finite and non-negative");
  }

  const double detection_age_ms = (now - detection_stamp) * 1000.0;
  const double tf_age_ms = (now - tf_stamp) * 1000.0;
  if (detection_age_ms < -max_future_skew * 1000.0) {
    return {false, "DETECTION_FUTURE", detection_age_ms, tf_age_ms};
  }
  if (tf_age_ms < -max_future_skew * 1000.0) {
    return {false, "TF_FUTURE", detection_age_ms, tf_age_ms};
  }
  if (detection_age_ms > max_detection_age * 1000.0) {
    return {false, "DETECTION_STALE", detection_age_ms, tf_age_ms};
  }
  if (tf_age_ms > max_tf_age * 1000.0) {
    return {false, "TF_STALE", detection_age_ms, tf_age_ms};
  }
  return {true, "ACCEPTED", detection_age_ms, tf_age_ms};
}

CameraCalibration validate_camera_calibration(
  const int width,
  const int height,
  const std::optional<std::array<double, 9>> & matrix,
  const std::string & distortion_model)
{
  if (width <= 0 || height <= 0) {
    return {false, "IMAGE_SIZE_INVALID", width, height, std::nullopt, std::nullopt};
  }
  if (distortion_model.empty()) {
    return {false, "DISTORTION_MODEL_MISSING", width, height, std::nullopt, std::nullopt};
  }
  if (!matrix.has_value()) {
    return {false, "INTRINSICS_INVALID", width, height, std::nullopt, std::nullopt};
  }
  for (const double value : *matrix) {
    if (!std::isfinite(value)) {
      return {false, "INTRINSICS_INVALID", width, height, std::nullopt, std::nullopt};
    }
  }
  const double fx = (*matrix)[0];
  const double fy = (*matrix)[4];
  const double cx = (*matrix)[2];
  const double cy = (*matrix)[5];
  if (fx <= 0.0 || fy <= 0.0) {
    return {false, "FOCAL_LENGTH_INVALID", width, height, fx, fy};
  }
  if (cx < 0.0 || cx > static_cast<double>(width) ||
    cy < 0.0 || cy > static_cast<double>(height))
  {
    return {false, "PRINCIPAL_POINT_INVALID", width, height, fx, fy};
  }
  return {true, "ACCEPTED", width, height, fx, fy};
}

double yaw_from_quaternion(const double x, const double y, const double z, const double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

std::tuple<double, double> pose_delta(
  const std::array<double, 4> & previous,
  const std::array<double, 4> & current)
{
  const double translation = std::sqrt(
    std::pow(current[0] - previous[0], 2.0) +
    std::pow(current[1] - previous[1], 2.0) +
    std::pow(current[2] - previous[2], 2.0));
  const double yaw = std::atan2(
    std::sin(current[3] - previous[3]),
    std::cos(current[3] - previous[3]));
  return {translation, std::abs(yaw)};
}

}  // namespace demo2_apriltag_docking
