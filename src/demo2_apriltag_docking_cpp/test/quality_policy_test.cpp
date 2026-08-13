#include "demo2_apriltag_docking_cpp/core/quality_policy.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace core = demo2_apriltag_docking_cpp::core;
constexpr double kPi = 3.141592653589793238462643383279502884;

TEST(QualityPolicy, AcceptsFreshObservation)
{
  const auto result = core::validate_observation_timing(10.0, 9.9, 9.85, 0.25, 0.25, 0.05);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.reason, "ACCEPTED");
  ASSERT_TRUE(result.detection_age_ms.has_value());
  ASSERT_TRUE(result.tf_age_ms.has_value());
  EXPECT_NEAR(*result.detection_age_ms, 100.0, 1e-9);
  EXPECT_NEAR(*result.tf_age_ms, 150.0, 1e-9);
}

TEST(QualityPolicy, RejectsStaleAndFutureObservation)
{
  EXPECT_EQ(
    core::validate_observation_timing(10.0, 9.7, 9.9, 0.25, 0.25, 0.05).reason,
    "DETECTION_STALE");
  EXPECT_EQ(
    core::validate_observation_timing(10.0, 10.1, 10.0, 0.25, 0.25, 0.05).reason,
    "DETECTION_FUTURE");
}

TEST(QualityPolicy, ValidatesCalibration)
{
  const std::array<double, 9> matrix{500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
  const auto valid = core::validate_camera_calibration(640, 480, matrix, "plumb_bob");
  EXPECT_TRUE(valid.valid);
  EXPECT_DOUBLE_EQ(*valid.fx, 500.0);
  EXPECT_EQ(
    core::validate_camera_calibration(640, 480, std::nullopt, "plumb_bob").reason,
    "INTRINSICS_INVALID");
}

TEST(QualityPolicy, WrapsYawAcrossPi)
{
  const auto delta = core::pose_delta(
    {0.0, 0.0, 0.0, 179.0 * kPi / 180.0},
    {0.1, 0.0, 0.0, -179.0 * kPi / 180.0});
  EXPECT_DOUBLE_EQ(delta.first, 0.1);
  EXPECT_NEAR(delta.second, 2.0 * kPi / 180.0, 1e-12);
}
