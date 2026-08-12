#pragma once

#include <array>
#include <optional>
#include <string>
#include <utility>

namespace demo2_apriltag_docking_cpp::core {

struct TemporalQuality {
  bool accepted;
  std::string reason;
  std::optional<double> detection_age_ms;
  std::optional<double> tf_age_ms;
};

struct CameraCalibration {
  bool valid;
  std::string reason;
  std::optional<int> width;
  std::optional<int> height;
  std::optional<double> fx;
  std::optional<double> fy;
};

TemporalQuality validate_observation_timing(
  double now,
  double detection_stamp,
  double tf_stamp,
  double max_detection_age,
  double max_tf_age,
  double max_future_skew);

CameraCalibration validate_camera_calibration(
  int width,
  int height,
  const std::optional<std::array<double, 9>> & matrix,
  const std::string & distortion_model);

double yaw_from_quaternion(double x, double y, double z, double w);

std::pair<double, double> pose_delta(
  const std::array<double, 4> & previous,
  const std::array<double, 4> & current);

}  // namespace demo2_apriltag_docking_cpp::core
