#include "demo2_apriltag_docking_cpp/adapters/tag_pose_adapter.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <string>
#include <utility>

namespace adapters = demo2_apriltag_docking_cpp::adapters;
namespace core = demo2_apriltag_docking_cpp::core;

TEST(TagPoseAdapter, ConvertsMessageAndTransformToDetection)
{
  apriltag_msgs::msg::AprilTagDetection message;
  message.id = 7;
  message.hamming = 0;
  message.decision_margin = 71.5F;

  geometry_msgs::msg::TransformStamped transform;
  transform.transform.translation.x = 1.2;
  transform.transform.translation.y = -0.3;
  transform.transform.rotation.z = std::sin(3.14159265358979323846 / 4.0);
  transform.transform.rotation.w = std::cos(3.14159265358979323846 / 4.0);

  const auto detection = adapters::to_policy_detection(message, transform, 12.25);
  EXPECT_EQ(detection.tag_id, 7);
  EXPECT_EQ(detection.hamming, 0);
  EXPECT_DOUBLE_EQ(detection.decision_margin, 71.5);
  EXPECT_DOUBLE_EQ(detection.stamp, 12.25);
  EXPECT_DOUBLE_EQ(detection.x, 1.2);
  EXPECT_DOUBLE_EQ(detection.y, -0.3);
  EXPECT_NEAR(detection.yaw, 3.14159265358979323846 / 2.0, 1e-12);
}

TEST(TagPoseAdapter, CreatesMetadataOnlyDetection)
{
  apriltag_msgs::msg::AprilTagDetection message;
  message.id = 3;
  message.hamming = 1;
  message.decision_margin = 42.5F;

  const auto detection = adapters::metadata_only(message, 5.5);
  EXPECT_EQ(detection.tag_id, 3);
  EXPECT_EQ(detection.hamming, 1);
  EXPECT_DOUBLE_EQ(detection.decision_margin, 42.5);
  EXPECT_DOUBLE_EQ(detection.stamp, 5.5);
  EXPECT_DOUBLE_EQ(detection.x, 0.0);
  EXPECT_DOUBLE_EQ(detection.y, 0.0);
  EXPECT_DOUBLE_EQ(detection.yaw, 0.0);
}

TEST(TagPoseAdapter, CopiesTransformToPose)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "camera_rgb_frame";
  transform.header.stamp.sec = 7;
  transform.transform.translation.x = 1.2;
  transform.transform.translation.y = -0.3;
  transform.transform.translation.z = 0.4;
  transform.transform.rotation.w = 1.0;

  const auto pose = adapters::to_pose_message(transform);
  EXPECT_EQ(pose.header, transform.header);
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.2);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, -0.3);
  EXPECT_DOUBLE_EQ(pose.pose.position.z, 0.4);
  EXPECT_DOUBLE_EQ(pose.pose.orientation.w, 1.0);
}

TEST(TagPoseAdapter, ConvertsCameraInfoToCalibration)
{
  sensor_msgs::msg::CameraInfo message;
  message.width = 640U;
  message.height = 480U;
  message.k = {500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
  message.distortion_model = "plumb_bob";

  const auto valid = adapters::camera_info_to_calibration(message);
  EXPECT_TRUE(valid.valid);
  EXPECT_DOUBLE_EQ(*valid.fx, 500.0);
  EXPECT_DOUBLE_EQ(*valid.fy, 500.0);

  message.k[0] = 0.0;
  const auto invalid = adapters::camera_info_to_calibration(message);
  EXPECT_FALSE(invalid.valid);
  EXPECT_EQ(invalid.reason, "FOCAL_LENGTH_INVALID");
}

TEST(TagPoseAdapter, FormatsTimestampsAndTemporalDiagnostics)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 250000000U;
  EXPECT_DOUBLE_EQ(adapters::stamp_seconds(stamp), 12.25);

  const core::TemporalQuality timing{true, "ACCEPTED", 100.0004, 149.9996};
  const auto values = adapters::temporal_values(timing);
  ASSERT_EQ(values.size(), 2U);
  EXPECT_EQ(values[0], std::make_pair(std::string("detection_age_ms"), std::string("100.0")));
  EXPECT_EQ(values[1], std::make_pair(std::string("tf_age_ms"), std::string("150.0")));
  EXPECT_EQ(adapters::diagnostic_number(42.5), "42.5");
  EXPECT_EQ(adapters::diagnostic_number(0.0), "0.0");
}

TEST(TagPoseAdapter, PreservesDiagnosticContract)
{
  const adapters::DiagnosticValues values{{"tag_id", "0"}, {"margin", "42.5"}};
  const auto status = adapters::make_status(
    "tag_pose_bridge",
    diagnostic_msgs::msg::DiagnosticStatus::WARN,
    "LOW_MARGIN",
    values);

  EXPECT_EQ(status.name, "tag_pose_bridge");
  EXPECT_EQ(status.hardware_id, "demo2_apriltag_docking");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "LOW_MARGIN");
  ASSERT_EQ(status.values.size(), 2U);
  EXPECT_EQ(status.values[0].key, "tag_id");
  EXPECT_EQ(status.values[0].value, "0");
  EXPECT_EQ(status.values[1].key, "margin");
  EXPECT_EQ(status.values[1].value, "42.5");
}

TEST(TagPoseAdapter, SuppressesRepeatedReportsAndRefreshesDiagnostics)
{
  adapters::ReportGate gate;
  EXPECT_EQ(gate.evaluate("CONFIRMING", 1.0).publish_state, true);
  const auto repeated = gate.evaluate("CONFIRMING", 1.1);
  EXPECT_FALSE(repeated.publish_state);
  EXPECT_FALSE(repeated.publish_diagnostic);

  const auto accepted = gate.evaluate("ACCEPTED", 1.2, 1.0);
  EXPECT_TRUE(accepted.publish_state);
  EXPECT_TRUE(accepted.publish_diagnostic);

  const auto early_refresh = gate.evaluate("ACCEPTED", 2.19, 1.0);
  EXPECT_FALSE(early_refresh.publish_state);
  EXPECT_FALSE(early_refresh.publish_diagnostic);

  const auto due_refresh = gate.evaluate("ACCEPTED", 2.2, 1.0);
  EXPECT_FALSE(due_refresh.publish_state);
  EXPECT_TRUE(due_refresh.publish_diagnostic);
}
