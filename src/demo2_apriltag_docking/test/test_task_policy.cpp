#include "demo2_apriltag_docking/task_policy.hpp"

#include <gtest/gtest.h>

namespace demo2_apriltag_docking
{
namespace
{

TEST(TaskPolicy, RequiresFreshTrueGuard)
{
  TaskPolicy policy(true, 2.0);
  EXPECT_EQ(policy.can_start(1.0), std::make_pair(false, std::string("GUARD_DENIED")));
  policy.update_guard(true, 1.0);
  EXPECT_EQ(policy.can_start(2.0), std::make_pair(true, std::string("READY")));
  EXPECT_EQ(policy.can_start(3.1), std::make_pair(false, std::string("GUARD_STALE")));
}

TEST(TaskPolicy, PreventsDuplicateAndCancelsActiveGoal)
{
  TaskPolicy policy(false, 2.0);
  policy.mark_active();
  EXPECT_TRUE(policy.action_active());
  EXPECT_EQ(policy.can_start(1.0), std::make_pair(false, std::string("ALREADY_ACTIVE")));
  policy.mark_idle();
  EXPECT_FALSE(policy.action_active());
}

}  // namespace
}  // namespace demo2_apriltag_docking
