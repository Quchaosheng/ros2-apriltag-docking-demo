#include "demo2_apriltag_docking_cpp/core/tag_policy.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace core = demo2_apriltag_docking_cpp::core;

namespace {

core::Detection detection(
  int tag_id = 0,
  int hamming = 0,
  double margin = 60.0,
  double x = 1.0,
  double y = 0.0,
  double yaw = 0.0)
{
  return {tag_id, hamming, margin, 1.0, x, y, yaw};
}

core::DockSpec dock(int tag_id = 0)
{
  return {tag_id, tag_id == 0 ? "demo_charge_dock" : "backup_dock", "charging_dock", "tag36h11:" + std::to_string(tag_id)};
}

core::TagGate make_gate()
{
  return core::TagGate(
    {{0, dock()}}, 50.0, 0, 3, 0.5, 0.1, 0.5, 0.25, 20.0 * 3.14159265358979323846 / 180.0);
}

}  // namespace

TEST(TagPolicy, RejectsBasicInvalidDetections)
{
  auto gate = make_gate();
  EXPECT_EQ(gate.evaluate({}, 1.0).reason, "NO_TAG");
  EXPECT_EQ(gate.evaluate({detection(99)}, 1.0).reason, "UNKNOWN_TAG");
  EXPECT_EQ(gate.evaluate({detection(0, 1)}, 1.0).reason, "HAMMING");
  EXPECT_EQ(gate.evaluate({detection(0, 0, 49.9)}, 1.0).reason, "LOW_MARGIN");
  EXPECT_EQ(gate.evaluate({detection(), detection()}, 1.0).reason, "MULTI_TAG");
}

TEST(TagPolicy, RequiresConsecutiveConfirmation)
{
  auto gate = make_gate();
  EXPECT_EQ(gate.evaluate({detection()}, 1.0).reason, "CONFIRMING");
  EXPECT_EQ(gate.evaluate({detection()}, 1.1).reason, "CONFIRMING");
  const auto accepted = gate.evaluate({detection()}, 1.2);
  EXPECT_TRUE(accepted.accepted);
  EXPECT_EQ(accepted.reason, "ACCEPTED");
  ASSERT_TRUE(accepted.dock.has_value());
  EXPECT_EQ(accepted.dock->dock_id, "demo_charge_dock");
}

TEST(TagPolicy, RateLimitsAndDetectsPoseJumps)
{
  auto gate = make_gate();
  gate.evaluate({detection()}, 1.0);
  gate.evaluate({detection()}, 1.1);
  EXPECT_EQ(gate.evaluate({detection()}, 1.2).reason, "ACCEPTED");
  EXPECT_EQ(gate.evaluate({detection(0, 0, 60.0, 1.0, 0.0)}, 1.25).reason, "RATE_LIMITED");
  EXPECT_EQ(gate.evaluate({detection(0, 0, 60.0, 1.0, 1.30)}, 1.4).reason, "POSE_JUMP");
  EXPECT_EQ(gate.evaluate({detection()}, 1.5).reason, "CONFIRMING");
}

TEST(TagPolicy, WrapsYawAndReportsLoss)
{
  auto gate = make_gate();
  gate.evaluate({detection(0, 0, 60.0, 1.0, 0.0, 179.0 * 3.14159265358979323846 / 180.0)}, 1.0);
  gate.evaluate({detection(0, 0, 60.0, 1.0, 0.0, 179.0 * 3.14159265358979323846 / 180.0)}, 1.1);
  gate.evaluate({detection(0, 0, 60.0, 1.0, 0.0, 179.0 * 3.14159265358979323846 / 180.0)}, 1.2);
  EXPECT_EQ(
    gate.evaluate({detection(0, 0, 60.0, 1.0, 0.0, -179.0 * 3.14159265358979323846 / 180.0)}, 1.4).reason,
    "ACCEPTED");
  EXPECT_FALSE(gate.loss_reason(1.5).has_value());
  const auto loss = gate.loss_reason(1.71);
  ASSERT_TRUE(loss.has_value());
  EXPECT_EQ(*loss, "TAG_LOST");
}

TEST(TagPolicy, RejectsInvalidParameters)
{
  EXPECT_THROW(
    core::TagGate({}, 50.0, 0, 3, 0.5, 0.1, 0.5, 0.25, 0.2),
    std::invalid_argument);
  EXPECT_THROW(
    core::TagGate({{0, dock()}}, 50.0, 0, 0, 0.5, 0.1, 0.5, 0.25, 0.2),
    std::invalid_argument);
  EXPECT_THROW(
    core::TagGate({{0, dock()}}, 50.0, 0, 3, 0.0, 0.1, 0.5, 0.25, 0.2),
    std::invalid_argument);
}
