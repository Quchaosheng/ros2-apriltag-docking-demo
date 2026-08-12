#include "demo2_apriltag_docking_cpp/core/quality_policy.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace demo2_apriltag_docking_cpp::core {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;

bool finite_nonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

bool blank(const std::string & value)
{
  return value.find_first_not_of(" \t\n\r\f\v") == std::string::npos;
}

double wrap_yaw(double value)
{
  double wrapped = std::fmod(value + kPi, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped - kPi;
}

}  // namespace

TemporalQuality validate_observation_timing(
  double now,
  double detection_stamp,
  double tf_stamp,
  double max_detection_age,
  double max_tf_age,
  double max_future_skew)
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

  const double detection_age = (now - detection_stamp) * 1000.0;
  const double tf_age = (now - tf_stamp) * 1000.0;
  if (detection_age < -max_future_skew * 1000.0) {
    return {false, "DETECTION_FUTURE", detection_age, tf_age};
  }
  if (tf_age < -max_future_skew * 1000.0) {
    return {false, "TF_FUTURE", detection_age, tf_age};
  }
  if (detection_age > max_detection_age * 1000.0) {
    return {false, "DETECTION_STALE", detection_age, tf_age};
  }
  if (tf_age > max_tf_age * 1000.0) {
    return {false, "TF_STALE", detection_age, tf_age};
  }
  return {true, "ACCEPTED", detection_age, tf_age};
}

CameraCalibration validate_camera_calibration(
  int width,
  int height,
  const std::optional<std::array<double, 9>> & matrix,
  const std::string & distortion_model)
{
  if (width <= 0 || height <= 0) {
    return {false, "IMAGE_SIZE_INVALID", width, height, std::nullopt, std::nullopt};
  }
  if (blank(distortion_model)) {
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

double yaw_from_quaternion(double x, double y, double z, double w)
{
  const double sin_yaw = 2.0 * (w * z + x * y);
  const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(sin_yaw, cos_yaw);
}

std::pair<double, double> pose_delta(
  const std::array<double, 4> & previous,
  const std::array<double, 4> & current)
{
  const double dx = current[0] - previous[0];
  const double dy = current[1] - previous[1];
  const double dz = current[2] - previous[2];
  const double translation = std::sqrt(dx * dx + dy * dy + dz * dz);
  return {translation, std::abs(wrap_yaw(current[3] - previous[3]))};
}

}  // namespace demo2_apriltag_docking_cpp::core
