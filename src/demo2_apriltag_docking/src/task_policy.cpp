#include "demo2_apriltag_docking/task_policy.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace demo2_apriltag_docking
{

TaskPolicy::TaskPolicy(const bool guard_required, const double guard_timeout)
: guard_required_(guard_required), guard_timeout_(guard_timeout)
{
  if (!std::isfinite(guard_timeout_) || guard_timeout_ <= 0.0) {
    throw std::invalid_argument("guard_timeout must be > 0");
  }
}

bool TaskPolicy::action_active() const noexcept {return action_active_;}
void TaskPolicy::mark_active() noexcept {action_active_ = true;}
void TaskPolicy::mark_idle() noexcept {action_active_ = false;}
void TaskPolicy::update_guard(const bool allowed, const double stamp) noexcept
{
  guard_allowed_ = allowed;
  guard_stamp_ = stamp;
}

std::pair<bool, std::string> TaskPolicy::can_start(const double now) const
{
  if (action_active_) {
    return {false, "ALREADY_ACTIVE"};
  }
  const auto reason = guard_reason(now);
  if (reason.has_value()) {
    return {false, *reason};
  }
  return {true, "READY"};
}

std::optional<std::string> TaskPolicy::cancel_reason(const double now) const
{
  if (!action_active_) {
    return std::nullopt;
  }
  return guard_reason(now);
}

std::optional<std::string> TaskPolicy::guard_reason(const double now) const
{
  if (!guard_required_) {
    return std::nullopt;
  }
  if (guard_allowed_ != true || !guard_stamp_.has_value()) {
    return "GUARD_DENIED";
  }
  if (now - *guard_stamp_ > guard_timeout_) {
    return "GUARD_STALE";
  }
  return std::nullopt;
}

}  // namespace demo2_apriltag_docking
