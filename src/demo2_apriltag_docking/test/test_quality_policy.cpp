#include "demo2_apriltag_docking/quality_policy.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace demo2_apriltag_docking
{
namespace
{

TEST(QualityPolicy, AcceptsFreshObservation)
{
  const auto result = validate_observation_timing(10.0, 9.9, 9.85, 0.25, 0.25, 0.05);
  EXPECT_TRUE(result.accepted);
  EXPECT_NEAR(*result.detection_age_ms, 100.0, 1e-9);
  EXPECT_NEAR(*result.tf_age_ms, 150.0, 1e-9);
}

TEST(QualityPolicy, RejectsStaleAndFutureObservation)
{
  EXPECT_STREQ(
    validate_observation_timing(10.0, 9.7, 9.9, 0.25, 0.25, 0.05).reason,
    "DETECTION_STALE");
  EXPECT_STREQ(
    validate_observation_timing(10.0, 10.1, 10.0, 0.25, 0.25, 0.05).reason,
    "DETECTION_FUTURE");
}

TEST(QualityPolicy, ValidatesCameraCalibration)
{
  const std::array<double, 9> matrix{500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
  const auto valid = validate_camera_calibration(640, 480, matrix, "plumb_bob");
  EXPECT_TRUE(valid.valid);
  EXPECT_DOUBLE_EQ(*valid.fx, 500.0);

  const std::array<double, 9> invalid_matrix{0.0, 0.0, 320.0, 0.0, 0.0, 240.0, 0.0, 0.0, 1.0};
  EXPECT_STREQ(
    validate_camera_calibration(640, 480, invalid_matrix, "plumb_bob").reason,
    "FOCAL_LENGTH_INVALID");
}

TEST(QualityPolicy, WrapsYawDeltaAcrossPi)
{
  constexpr double pi = 3.14159265358979323846;
  const auto [translation, yaw] = pose_delta(
    std::array<double, 4>{0.0, 0.0, 0.0, 179.0 * pi / 180.0},
    std::array<double, 4>{0.1, 0.0, 0.0, -179.0 * pi / 180.0});
  EXPECT_DOUBLE_EQ(translation, 0.1);
  EXPECT_NEAR(yaw, 2.0 * pi / 180.0, 1e-12);
}

}  // namespace
}  // namespace demo2_apriltag_docking
