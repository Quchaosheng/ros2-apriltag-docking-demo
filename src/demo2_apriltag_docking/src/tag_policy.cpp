#include "demo2_apriltag_docking/tag_policy.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace demo2_apriltag_docking
{

const char * to_string(const GateReason reason) noexcept
{
  switch (reason) {
    case GateReason::kNoTag: return "NO_TAG";
    case GateReason::kMultiTag: return "MULTI_TAG";
    case GateReason::kUnknownTag: return "UNKNOWN_TAG";
    case GateReason::kHamming: return "HAMMING";
    case GateReason::kLowMargin: return "LOW_MARGIN";
    case GateReason::kConfirming: return "CONFIRMING";
    case GateReason::kPoseJump: return "POSE_JUMP";
    case GateReason::kRateLimited: return "RATE_LIMITED";
    case GateReason::kTagLost: return "TAG_LOST";
    case GateReason::kAccepted: return "ACCEPTED";
  }
  return "UNKNOWN";
}

TagGate::TagGate(
  std::unordered_map<int, DockSpec> specs,
  const double min_margin,
  const int max_hamming,
  const int confirmations,
  const double confirmation_window,
  const double publish_period,
  const double loss_timeout,
  const double max_translation_jump,
  const double max_yaw_jump)
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
    throw std::invalid_argument("specs must be non-empty");
  }
  if (!std::isfinite(min_margin_)) {
    throw std::invalid_argument("min_margin must be finite");
  }
  if (confirmations_ < 1) {
    throw std::invalid_argument("confirmations must be >= 1");
  }
  if (max_hamming_ < 0) {
    throw std::invalid_argument("max_hamming must be >= 0");
  }
  const auto require_positive = [](const double value, const char * name) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be > 0");
      }
    };
  require_positive(confirmation_window_, "confirmation_window");
  require_positive(loss_timeout_, "loss_timeout");
  require_positive(max_translation_jump_, "max_translation_jump");
  require_positive(max_yaw_jump_, "max_yaw_jump");
  if (!std::isfinite(publish_period_) || publish_period_ < 0.0) {
    throw std::invalid_argument("publish_period must be >= 0");
  }
}

GateResult TagGate::evaluate(const std::vector<Detection> & detections, const double now)
{
  if (detections.empty()) {
    return reject(GateReason::kNoTag);
  }
  if (detections.size() != 1U) {
    return reject(GateReason::kMultiTag);
  }

  const Detection & detection = detections.front();
  const auto dock_it = specs_.find(detection.tag_id);
  if (dock_it == specs_.end()) {
    return reject(GateReason::kUnknownTag);
  }
  if (detection.hamming > max_hamming_) {
    return reject(GateReason::kHamming);
  }
  if (detection.decision_margin < min_margin_) {
    return reject(GateReason::kLowMargin);
  }

  if (last_seen_.has_value() && now - *last_seen_ > loss_timeout_) {
    last_accepted_.reset();
    reset_confirmation();
  }
  last_seen_ = now;

  if (is_pose_jump(detection)) {
    return reject(GateReason::kPoseJump);
  }

  if (confirmed_ && active_tag_id_ != detection.tag_id) {
    reset_confirmation();
  }

  if (!confirmed_) {
    const bool expired =
      !confirmation_started_.has_value() ||
      now - *confirmation_started_ > confirmation_window_;
    if (active_tag_id_ != detection.tag_id || expired) {
      active_tag_id_ = detection.tag_id;
      confirmation_count_ = 1;
      confirmation_started_ = now;
    } else {
      ++confirmation_count_;
    }
    if (confirmation_count_ < confirmations_) {
      return GateResult{false, GateReason::kConfirming, detection, dock_it->second};
    }
    confirmed_ = true;
  }

  if (last_publication_.has_value() && now - *last_publication_ < publish_period_) {
    return GateResult{false, GateReason::kRateLimited, detection, dock_it->second};
  }

  last_accepted_ = detection;
  last_publication_ = now;
  return GateResult{true, GateReason::kAccepted, detection, dock_it->second};
}

std::optional<GateReason> TagGate::loss_reason(const double now)
{
  if (!last_seen_.has_value() || now - *last_seen_ <= loss_timeout_) {
    return std::nullopt;
  }
  last_seen_.reset();
  last_accepted_.reset();
  reset_confirmation();
  return GateReason::kTagLost;
}

GateResult TagGate::reject(const GateReason reason)
{
  reset_confirmation();
  return GateResult{false, reason, std::nullopt, std::nullopt};
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
  const double yaw_delta = std::atan2(
    std::sin(detection.yaw - last_accepted_->yaw),
    std::cos(detection.yaw - last_accepted_->yaw));
  return translation > max_translation_jump_ || std::abs(yaw_delta) > max_yaw_jump_;
}

}  // namespace demo2_apriltag_docking
