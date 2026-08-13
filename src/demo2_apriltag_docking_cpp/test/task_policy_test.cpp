#include "demo2_apriltag_docking_cpp/core/task_policy.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace core = demo2_apriltag_docking_cpp::core;

namespace {

void expect_invalid_max_staging_time(double value)
{
  try {
    core::validate_max_staging_time(value);
    FAIL() << "expected invalid max_staging_time";
  } catch (const std::invalid_argument & error) {
    EXPECT_STREQ(
      error.what(),
      "max_staging_time must be finite and representable as a positive float32");
  }
}

}  // namespace

TEST(TaskPolicy, RequiredGuardDeniesWithoutMessage)
{
  core::TaskPolicy policy(true, 2.0);
  EXPECT_EQ(policy.can_start(1.0), std::make_pair(false, std::string("GUARD_DENIED")));
}

TEST(TaskPolicy, OptionalGuardAllowsWithoutMessage)
{
  core::TaskPolicy policy(false, 2.0);
  EXPECT_EQ(policy.can_start(1.0), std::make_pair(true, std::string("READY")));
}

TEST(TaskPolicy, FreshAndFalseGuards)
{
  core::TaskPolicy policy(true, 2.0);
  policy.update_guard(true, 1.0);
  EXPECT_EQ(policy.can_start(2.0), std::make_pair(true, std::string("READY")));
  policy.update_guard(false, 2.0);
  EXPECT_EQ(policy.can_start(2.0), std::make_pair(false, std::string("GUARD_DENIED")));
}

TEST(TaskPolicy, StaleGuardAndDuplicateStart)
{
  core::TaskPolicy policy(true, 2.0);
  policy.update_guard(true, 1.0);
  EXPECT_EQ(policy.can_start(3.1), std::make_pair(false, std::string("GUARD_STALE")));
  policy.mark_active();
  EXPECT_EQ(policy.can_start(1.0), std::make_pair(false, std::string("ALREADY_ACTIVE")));
  EXPECT_EQ(policy.cancel_reason(3.1), std::optional<std::string>("GUARD_STALE"));
}

TEST(TaskPolicy, OptionalGuardNeverCancels)
{
  core::TaskPolicy policy(false, 2.0);
  policy.mark_active();
  EXPECT_FALSE(policy.cancel_reason(100.0).has_value());
  policy.mark_idle();
  EXPECT_FALSE(policy.action_active());
}

TEST(TaskPolicy, RejectsInvalidTimeout)
{
  EXPECT_THROW(core::TaskPolicy(true, 0.0), std::invalid_argument);
  EXPECT_THROW(core::TaskPolicy(true, -1.0), std::invalid_argument);
  EXPECT_THROW(
    core::TaskPolicy(true, std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  EXPECT_THROW(
    core::TaskPolicy(true, std::numeric_limits<double>::infinity()),
    std::invalid_argument);
  EXPECT_THROW(
    core::TaskPolicy(true, -std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

TEST(TaskPolicy, RejectsMaxStagingTimeOutsidePositiveFloat32)
{
  expect_invalid_max_staging_time(0.0);
  expect_invalid_max_staging_time(-1.0);
  expect_invalid_max_staging_time(std::numeric_limits<double>::quiet_NaN());
  expect_invalid_max_staging_time(std::numeric_limits<double>::infinity());
  expect_invalid_max_staging_time(-std::numeric_limits<double>::infinity());
  expect_invalid_max_staging_time(std::numeric_limits<double>::max());
  expect_invalid_max_staging_time(std::numeric_limits<double>::denorm_min());
  expect_invalid_max_staging_time(
    std::nextafter(
      static_cast<double>(std::numeric_limits<float>::max()),
      std::numeric_limits<double>::infinity()));
}

TEST(TaskPolicy, AcceptsPositiveFloat32MaxStagingTime)
{
  EXPECT_EQ(core::validate_max_staging_time(60.0), 60.0F);
  EXPECT_EQ(
    core::validate_max_staging_time(std::numeric_limits<float>::max()),
    std::numeric_limits<float>::max());
  EXPECT_EQ(
    core::validate_max_staging_time(std::numeric_limits<float>::denorm_min()),
    std::numeric_limits<float>::denorm_min());
}
