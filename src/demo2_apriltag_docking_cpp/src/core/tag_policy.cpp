#include "demo2_apriltag_docking_cpp/core/tag_policy.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace demo2_apriltag_docking_cpp::core {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;

bool finite(double value)
{
  return std::isfinite(value);
}

double wrapped_yaw_delta(double value)
{
  double wrapped = std::fmod(value + kPi, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped - kPi;
}

}  // namespace

TagGate::TagGate(
  std::unordered_map<int, DockSpec> specs,
  double min_margin,
  int max_hamming,
  int confirmations,
  double confirmation_window,
  double publish_period,
  double loss_timeout,
  double max_translation_jump,
  double max_yaw_jump)
: specs_(std::move(specs)),
  min_margin_(min_margin),
  max_hamming_(max_hamming),
  confirmations_(confirmations),
  confirmation_window_(confirmation_window),
  publish_period_(publish_period),
  loss_timeout_(loss_timeout),
  max_translation_jump_(max_translation_jump),
  max_yaw_jump_(max_yaw_jump)
{
  if (specs_.empty()) {
    throw std::invalid_argument("specs must be a non-empty mapping");
  }
  if (!finite(min_margin_)) {
    throw std::invalid_argument("min_margin must be finite");
  }
  if (confirmations_ < 1) {
    throw std::invalid_argument("confirmations must be >= 1");
  }
  if (max_hamming_ < 0) {
    throw std::invalid_argument("max_hamming must be >= 0");
  }
  if (!finite(loss_timeout_) || loss_timeout_ <= 0.0) {
    throw std::invalid_argument("loss_timeout must be > 0");
  }
  if (!finite(confirmation_window_) || confirmation_window_ <= 0.0) {
    throw std::invalid_argument("confirmation_window must be > 0");
  }
  if (!finite(max_translation_jump_) || max_translation_jump_ <= 0.0) {
    throw std::invalid_argument("max_translation_jump must be > 0");
  }
  if (!finite(max_yaw_jump_) || max_yaw_jump_ <= 0.0) {
    throw std::invalid_argument("max_yaw_jump must be > 0");
  }
  if (!finite(publish_period_) || publish_period_ < 0.0) {
    throw std::invalid_argument("publish_period must be >= 0");
  }
}

GateResult TagGate::evaluate(const std::vector<Detection> & detections, double now)
{
  if (detections.empty()) {
    return reject("NO_TAG");
  }
  if (detections.size() != 1U) {
    return reject("MULTI_TAG");
  }

  const Detection & detection = detections.front();
  const auto dock_it = specs_.find(detection.tag_id);
  if (dock_it == specs_.end()) {
    return reject("UNKNOWN_TAG");
  }
  if (detection.hamming > max_hamming_) {
    return reject("HAMMING");
  }
  if (detection.decision_margin < min_margin_) {
    return reject("LOW_MARGIN");
  }

  if (last_seen_.has_value() && now - *last_seen_ > loss_timeout_) {
    last_accepted_.reset();
    reset_confirmation();
  }
  last_seen_ = now;

  if (is_pose_jump(detection)) {
    return reject("POSE_JUMP");
  }

  const DockSpec & dock = dock_it->second;
  if (confirmed_ && active_tag_id_ != detection.tag_id) {
    reset_confirmation();
  }

  if (!confirmed_) {
    const bool expired = !confirmation_started_.has_value() ||
      now - *confirmation_started_ > confirmation_window_;
    if (active_tag_id_ != detection.tag_id || expired) {
      active_tag_id_ = detection.tag_id;
      confirmation_count_ = 1;
      confirmation_started_ = now;
    } else {
      ++confirmation_count_;
    }

    if (confirmation_count_ < confirmations_) {
      return {false, "CONFIRMING", detection, dock};
    }
    confirmed_ = true;
  }

  if (last_publication_.has_value() && now - *last_publication_ < publish_period_) {
    return {false, "RATE_LIMITED", detection, dock};
  }

  last_accepted_ = detection;
  last_publication_ = now;
  return {true, "ACCEPTED", detection, dock};
}

std::optional<std::string> TagGate::loss_reason(double now)
{
  if (!last_seen_.has_value() || now - *last_seen_ <= loss_timeout_) {
    return std::nullopt;
  }
  last_seen_.reset();
  last_accepted_.reset();
  reset_confirmation();
  return "TAG_LOST";
}

GateResult TagGate::reject(const std::string & reason)
{
  reset_confirmation();
  return {false, reason, std::nullopt, std::nullopt};
}

void TagGate::reset_confirmation()
{
  active_tag_id_.reset();
  confirmation_count_ = 0;
  confirmation_started_.reset();
  confirmed_ = false;
}

bool TagGate::is_pose_jump(const Detection & detection) const
{
  if (!last_accepted_.has_value() || last_accepted_->tag_id != detection.tag_id) {
    return false;
  }
  const double translation = std::hypot(
    detection.x - last_accepted_->x,
    detection.y - last_accepted_->y);
  const double yaw_delta = wrapped_yaw_delta(detection.yaw - last_accepted_->yaw);
  return translation > max_translation_jump_ || std::abs(yaw_delta) > max_yaw_jump_;
}

}  // namespace demo2_apriltag_docking_cpp::core
