#include "demo2_apriltag_docking/quality_policy.hpp"
#include "demo2_apriltag_docking/tag_policy.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace demo2_apriltag_docking
{
namespace
{

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

Detection metadata_only(const apriltag_msgs::msg::AprilTagDetection & message, const double stamp)
{
  return Detection{
    static_cast<int>(message.id), static_cast<int>(message.hamming),
    static_cast<double>(message.decision_margin), stamp, 0.0, 0.0, 0.0};
}

Detection to_policy_detection(
  const apriltag_msgs::msg::AprilTagDetection & message,
  const geometry_msgs::msg::TransformStamped & transform,
  const double stamp)
{
  const auto & rotation = transform.transform.rotation;
  const auto & translation = transform.transform.translation;
  const double sin_yaw = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y);
  const double cos_yaw = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z);
  return Detection{
    static_cast<int>(message.id), static_cast<int>(message.hamming),
    static_cast<double>(message.decision_margin), stamp,
    translation.x, translation.y, std::atan2(sin_yaw, cos_yaw)};
}

geometry_msgs::msg::PoseStamped to_pose_message(
  const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

}  // namespace

class TagPoseBridge : public rclcpp::Node
{
public:
  TagPoseBridge()
  : Node("tag_pose_bridge"), tf_buffer_(this->get_clock())
  {
    declare_parameter<std::string>("dock_mapping_file", "");
    declare_parameter<std::string>("detections_topic", "/apriltag/detections");
    declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    declare_parameter<std::string>("output_pose_topic", "/detected_dock_pose");
    declare_parameter<std::string>("state_topic", "/demo2/tag_state");
    declare_parameter<double>("min_decision_margin", 50.0);
    declare_parameter<int>("max_hamming", 0);
    declare_parameter<int>("confirmations", 3);
    declare_parameter<double>("confirmation_window", 0.5);
    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<double>("loss_timeout", 0.5);
    declare_parameter<double>("max_translation_jump", 0.25);
    declare_parameter<double>("max_yaw_jump_deg", 20.0);
    declare_parameter<bool>("require_camera_calibration", true);
    declare_parameter<double>("max_detection_age", 0.25);
    declare_parameter<double>("max_tf_age", 0.25);
    declare_parameter<double>("max_future_skew", 0.05);

    const auto mapping_file = get_parameter("dock_mapping_file").as_string();
    if (mapping_file.empty()) {
      throw std::invalid_argument("dock_mapping_file must be set");
    }
    specs_ = load_dock_specs(mapping_file);
    const double publish_rate = get_parameter("publish_rate_hz").as_double();
    if (!std::isfinite(publish_rate) || publish_rate <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be > 0");
    }
    constexpr double pi = 3.14159265358979323846;
    gate_ = std::make_unique<TagGate>(
      specs_, get_parameter("min_decision_margin").as_double(),
      get_parameter("max_hamming").as_int(), get_parameter("confirmations").as_int(),
      get_parameter("confirmation_window").as_double(), 1.0 / publish_rate,
      get_parameter("loss_timeout").as_double(),
      get_parameter("max_translation_jump").as_double(),
      get_parameter("max_yaw_jump_deg").as_double() * pi / 180.0);

    rclcpp::QoS state_qos(rclcpp::KeepLast(1));
    state_qos.reliable().transient_local();
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      get_parameter("output_pose_topic").as_string(), 10);
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      get_parameter("state_topic").as_string(), state_qos);
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      get_parameter("camera_info_topic").as_string(), 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr message) {
        std::array<double, 9> matrix{};
        std::copy(message->k.begin(), message->k.end(), matrix.begin());
        camera_calibration_ = validate_camera_calibration(
          static_cast<int>(message->width), static_cast<int>(message->height),
          matrix, message->distortion_model);
      });
    detection_subscription_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      get_parameter("detections_topic").as_string(), 10,
      [this](const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr message) {
        on_detections(message);
      });
    loss_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {check_loss();});
  }

private:
  double now_seconds() const {return get_clock()->now().seconds();}

  void on_detections(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr & message)
  {
    const double now = now_seconds();
    const double stamp = stamp_seconds(message->header.stamp);
    const auto & detections = message->detections;
    if (detections.size() != 1U) {
      std::vector<Detection> metadata;
      metadata.reserve(detections.size());
      for (const auto & item : detections) {
        metadata.push_back(metadata_only(item, stamp));
      }
      const auto result = gate_->evaluate(metadata, now);
      report(to_string(result.reason), diagnostic_msgs::msg::DiagnosticStatus::WARN);
      return;
    }

    const auto & tag_message = detections.front();
    const auto dock_it = specs_.find(static_cast<int>(tag_message.id));
    if (dock_it == specs_.end()) {
      const auto result = gate_->evaluate({metadata_only(tag_message, stamp)}, now);
      report(to_string(result.reason), diagnostic_msgs::msg::DiagnosticStatus::WARN);
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        message->header.frame_id, dock_it->second.tag_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      report("TF_UNAVAILABLE", diagnostic_msgs::msg::DiagnosticStatus::WARN);
      RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    const double transform_stamp = stamp_seconds(transform.header.stamp);
    const auto timing = validate_observation_timing(
      now, stamp, transform_stamp, get_parameter("max_detection_age").as_double(),
      get_parameter("max_tf_age").as_double(), get_parameter("max_future_skew").as_double());
    if (!timing.accepted) {
      report(timing.reason, diagnostic_msgs::msg::DiagnosticStatus::WARN);
      return;
    }
    if (get_parameter("require_camera_calibration").as_bool()) {
      if (!camera_calibration_.has_value()) {
        report("CALIBRATION_UNAVAILABLE", diagnostic_msgs::msg::DiagnosticStatus::WARN);
        return;
      }
      if (!camera_calibration_->valid) {
        report(camera_calibration_->reason, diagnostic_msgs::msg::DiagnosticStatus::WARN);
        return;
      }
    }

    const auto result = gate_->evaluate(
      {to_policy_detection(tag_message, transform, transform_stamp)}, now);
    if (result.accepted) {
      pose_publisher_->publish(to_pose_message(transform));
      report("ACCEPTED", diagnostic_msgs::msg::DiagnosticStatus::OK);
    } else if (result.reason != GateReason::kRateLimited) {
      report(to_string(result.reason), diagnostic_msgs::msg::DiagnosticStatus::WARN);
    }
  }

  void check_loss()
  {
    const auto reason = gate_->loss_reason(now_seconds());
    if (reason.has_value()) {
      report(to_string(*reason), diagnostic_msgs::msg::DiagnosticStatus::WARN);
    }
  }

  void report(const std::string & state, const std::uint8_t level)
  {
    const double now = now_seconds();
    const bool repeated = state == last_state_;
    const bool refresh_due = state == "ACCEPTED" &&
      (last_diagnostic_time_ == 0.0 || now - last_diagnostic_time_ >= 1.0);
    if (repeated && !refresh_due) {
      return;
    }
    if (!repeated) {
      std_msgs::msg::String state_message;
      state_message.data = state;
      state_publisher_->publish(state_message);
      last_state_ = state;
    }
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = get_clock()->now().to_msg();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_name();
    status.hardware_id = "demo2_apriltag_docking";
    status.level = level;
    status.message = state;
    diagnostics.status.push_back(status);
    diagnostic_publisher_->publish(diagnostics);
    last_diagnostic_time_ = now;
  }

  std::unordered_map<int, DockSpec> specs_;
  std::unique_ptr<TagGate> gate_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_subscription_;
  rclcpp::TimerBase::SharedPtr loss_timer_;
  std::optional<CameraCalibration> camera_calibration_;
  std::string last_state_;
  double last_diagnostic_time_{0.0};
};

}  // namespace demo2_apriltag_docking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<demo2_apriltag_docking::TagPoseBridge>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    fprintf(stderr, "tag_pose_bridge: %s\n", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
