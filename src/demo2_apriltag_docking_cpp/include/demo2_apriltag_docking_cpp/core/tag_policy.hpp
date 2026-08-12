#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace demo2_apriltag_docking_cpp::core {

struct DockSpec {
  int tag_id;
  std::string dock_id;
  std::string dock_type;
  std::string tag_frame;
};

struct Detection {
  int tag_id;
  int hamming;
  double decision_margin;
  double stamp;
  double x;
  double y;
  double yaw;
};

struct GateResult {
  bool accepted;
  std::string reason;
  std::optional<Detection> detection;
  std::optional<DockSpec> dock;
};

class TagGate {
public:
  TagGate(
    std::unordered_map<int, DockSpec> specs,
    double min_margin,
    int max_hamming,
    int confirmations,
    double confirmation_window,
    double publish_period,
    double loss_timeout,
    double max_translation_jump,
    double max_yaw_jump);

  GateResult evaluate(const std::vector<Detection> & detections, double now);
  std::optional<std::string> loss_reason(double now);

private:
  GateResult reject(const std::string & reason);
  void reset_confirmation();
  bool is_pose_jump(const Detection & detection) const;

  std::unordered_map<int, DockSpec> specs_;
  double min_margin_;
  int max_hamming_;
  int confirmations_;
  double confirmation_window_;
  double publish_period_;
  double loss_timeout_;
  double max_translation_jump_;
  double max_yaw_jump_;

  std::optional<int> active_tag_id_;
  int confirmation_count_{0};
  std::optional<double> confirmation_started_;
  bool confirmed_{false};
  std::optional<Detection> last_accepted_;
  std::optional<double> last_publication_;
  std::optional<double> last_seen_;
};

}  // namespace demo2_apriltag_docking_cpp::core
