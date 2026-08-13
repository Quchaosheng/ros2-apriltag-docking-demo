#pragma once

#include <optional>
#include <string>
#include <utility>

namespace demo2_apriltag_docking_cpp::core {

float validate_max_staging_time(double value);

class TaskPolicy {
public:
  TaskPolicy(bool guard_required, double guard_timeout);

  bool action_active() const noexcept;
  void mark_active() noexcept;
  void mark_idle() noexcept;
  void update_guard(bool allowed, double stamp);

  std::pair<bool, std::string> can_start(double now) const;
  std::optional<std::string> cancel_reason(double now) const;

private:
  std::optional<std::string> guard_reason(double now) const;

  bool guard_required_;
  double guard_timeout_;
  std::optional<bool> guard_allowed_;
  std::optional<double> guard_stamp_;
  bool action_active_{false};
};

}  // namespace demo2_apriltag_docking_cpp::core
