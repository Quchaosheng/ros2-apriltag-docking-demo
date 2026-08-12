#include "demo2_apriltag_docking_cpp/core/task_policy.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace core = demo2_apriltag_docking_cpp::core;

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
}

