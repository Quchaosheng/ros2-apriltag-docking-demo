#include "demo2_apriltag_docking_cpp/adapters/dock_spec_loader.hpp"
#include "demo2_apriltag_docking_cpp/adapters/tag_pose_adapter.hpp"
#include "demo2_apriltag_docking_cpp/core/quality_policy.hpp"
#include "demo2_apriltag_docking_cpp/core/tag_policy.hpp"

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace demo2_apriltag_docking_cpp::nodes {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
using adapters::DiagnosticValues;

}  // namespace

class TagPoseBridge : public rclcpp::Node {
public:
  TagPoseBridge()
  : Node("tag_pose_bridge")
  {
    declare_parameters();

    const std::string mapping_file = get_parameter("dock_mapping_file").as_string();
    if (mapping_file.empty()) {
      throw std::invalid_argument("dock_mapping_file must be set");
    }
    specs_ = adapters::load_dock_specs(mapping_file);

    const double publish_rate = get_parameter("publish_rate_hz").as_double();
    gate_ = std::make_unique<core::TagGate>(
      specs_,
      get_parameter("min_decision_margin").as_double(),
      static_cast<int>(get_parameter("max_hamming").as_int()),
      static_cast<int>(get_parameter("confirmations").as_int()),
      get_parameter("confirmation_window").as_double(),
      1.0 / publish_rate,
      get_parameter("loss_timeout").as_double(),
      get_parameter("max_translation_jump").as_double(),
      get_parameter("max_yaw_jump_deg").as_double() * kPi / 180.0);

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      get_parameter("output_pose_topic").as_string(), rclcpp::QoS(10));
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      get_parameter("state_topic").as_string(), state_qos);
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
      get_clock(), tf2::durationFromSec(10.0));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      get_parameter("camera_info_topic").as_string(),
      rclcpp::QoS(10),
      std::bind(&TagPoseBridge::on_camera_info, this, std::placeholders::_1));
    detection_subscription_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      get_parameter("detections_topic").as_string(),
      rclcpp::QoS(10),
      std::bind(&TagPoseBridge::on_detections, this, std::placeholders::_1));
    loss_timer_ = rclcpp::create_timer(
      this,
      get_clock(),
      rclcpp::Duration::from_seconds(0.1),
      std::bind(&TagPoseBridge::check_loss, this));
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("dock_mapping_file", "");
    declare_parameter<std::string>("detections_topic", "/apriltag/detections");
    declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    declare_parameter<std::string>("output_pose_topic", "/detected_dock_pose");
    declare_parameter<std::string>("state_topic", "/demo2/tag_state");
    declare_parameter<double>("min_decision_margin", 50.0);
    declare_parameter<std::int64_t>("max_hamming", 0);
    declare_parameter<std::int64_t>("confirmations", 3);
    declare_parameter<double>("confirmation_window", 0.5);
    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<double>("loss_timeout", 0.5);
    declare_parameter<double>("max_translation_jump", 0.25);
    declare_parameter<double>("max_yaw_jump_deg", 20.0);
    declare_parameter<bool>("require_camera_calibration", true);
    declare_parameter<double>("max_detection_age", 0.25);
    declare_parameter<double>("max_tf_age", 0.25);
    declare_parameter<double>("max_future_skew", 0.05);
  }

  void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr message)
  {
    camera_calibration_ = adapters::camera_info_to_calibration(*message);
  }

  void on_detections(
    const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr message)
  {
    const double now = now_seconds();
    const double detection_stamp = adapters::stamp_seconds(message->header.stamp);

    if (message->detections.size() != 1U) {
      std::vector<core::Detection> detections;
      detections.reserve(message->detections.size());
      for (const auto & detection : message->detections) {
        detections.push_back(adapters::metadata_only(detection, detection_stamp));
      }
      report(gate_->evaluate(detections, now).reason, diagnostic_msgs::msg::DiagnosticStatus::WARN);
      return;
    }

    const auto & detection_message = message->detections.front();
    const auto dock = specs_.find(detection_message.id);
    if (dock == specs_.end()) {
      const auto result = gate_->evaluate(
        {adapters::metadata_only(detection_message, detection_stamp)}, now);
      report(
        result.reason,
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        {{"tag_id", std::to_string(detection_message.id)}});
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        message->header.frame_id, dock->second.tag_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      report(
        "TF_UNAVAILABLE",
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        {
          {"tag_id", std::to_string(detection_message.id)},
          {"error", error.what()},
        });
      return;
    }

    const double transform_stamp = adapters::stamp_seconds(transform.header.stamp);
    const auto timing = core::validate_observation_timing(
      now,
      detection_stamp,
      transform_stamp,
      get_parameter("max_detection_age").as_double(),
      get_parameter("max_tf_age").as_double(),
      get_parameter("max_future_skew").as_double());
    if (!timing.accepted) {
      report(
        timing.reason,
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        adapters::temporal_values(timing));
      return;
    }

    if (get_parameter("require_camera_calibration").as_bool()) {
      if (!camera_calibration_.has_value()) {
        report("CALIBRATION_UNAVAILABLE", diagnostic_msgs::msg::DiagnosticStatus::WARN);
        return;
      }
      if (!camera_calibration_->valid) {
        report(
          "CALIBRATION_INVALID",
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          {{"reason", camera_calibration_->reason}});
        return;
      }
    }

    const core::Detection detection = adapters::to_policy_detection(
      detection_message, transform, transform_stamp);
    const auto result = gate_->evaluate({detection}, now);
    if (result.accepted) {
      pose_publisher_->publish(adapters::to_pose_message(transform));
      DiagnosticValues values{
        {"tag_id", std::to_string(detection.tag_id)},
        {"dock_id", result.dock->dock_id},
        {"margin", adapters::diagnostic_number(detection.decision_margin)},
      };
      const auto timing_diagnostics = adapters::temporal_values(timing);
      values.insert(values.end(), timing_diagnostics.begin(), timing_diagnostics.end());
      report(
        "ACCEPTED",
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        std::move(values),
        1.0);
    } else if (result.reason != "RATE_LIMITED") {
      report(
        result.reason,
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        {{"tag_id", std::to_string(detection.tag_id)}});
    }
  }

  void check_loss()
  {
    const auto reason = gate_->loss_reason(now_seconds());
    if (reason.has_value()) {
      report(*reason, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    }
  }

  void report(
    const std::string & state,
    std::uint8_t level,
    DiagnosticValues values = {},
    std::optional<double> refresh_seconds = std::nullopt)
  {
    const auto decision = report_gate_.evaluate(state, now_seconds(), refresh_seconds);
    if (!decision.publish_diagnostic) {
      return;
    }

    if (decision.publish_state) {
      std_msgs::msg::String state_message;
      state_message.data = state;
      state_publisher_->publish(state_message);
    }

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = get_clock()->now();
    diagnostics.status.push_back(adapters::make_status(get_name(), level, state, values));
    diagnostic_publisher_->publish(diagnostics);
  }

  double now_seconds() const
  {
    return get_clock()->now().seconds();
  }

  std::unordered_map<int, core::DockSpec> specs_;
  std::unique_ptr<core::TagGate> gate_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr
    detection_subscription_;
  rclcpp::TimerBase::SharedPtr loss_timer_;
  std::optional<core::CameraCalibration> camera_calibration_;
  adapters::ReportGate report_gate_;
};

}  // namespace demo2_apriltag_docking_cpp::nodes

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<demo2_apriltag_docking_cpp::nodes::TagPoseBridge>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("tag_pose_bridge"), "%s", error.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
