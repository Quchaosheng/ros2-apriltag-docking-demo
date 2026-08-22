#include "demo2_apriltag_docking/tag_policy.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace demo2_apriltag_docking
{
namespace
{

DockSpec make_dock(const int tag_id = 0)
{
  return DockSpec{
    tag_id, "dock_" + std::to_string(tag_id), "charging_dock",
    "tag36h11:" + std::to_string(tag_id)};
}

Detection make_detection(
  const int tag_id = 0,
  const double stamp = 1.0,
  const double x = 1.0,
  const double y = 0.0,
  const double yaw = 0.0,
  const double margin = 60.0,
  const int hamming = 0)
{
  return Detection{tag_id, hamming, margin, stamp, x, y, yaw};
}

TagGate make_gate()
{
  constexpr double pi = 3.14159265358979323846;
  return TagGate(
    {{0, make_dock()}}, 50.0, 0, 3, 0.5, 0.1, 0.5, 0.25,
    20.0 * pi / 180.0);
}

TEST(TagPolicyData, StoresDockSpecification)
{
  const DockSpec dock{0, "demo_charge_dock", "charging_dock", "tag36h11:0"};
  EXPECT_EQ(dock.tag_id, 0);
  EXPECT_EQ(dock.dock_id, "demo_charge_dock");
  EXPECT_EQ(dock.dock_type, "charging_dock");
  EXPECT_EQ(dock.tag_frame, "tag36h11:0");
}

TEST(TagGate, RejectsBasicInvalidObservations)
{
  auto gate = make_gate();
  EXPECT_EQ(gate.evaluate({}, 1.0).reason, GateReason::kNoTag);
  EXPECT_EQ(gate.evaluate({make_detection(99)}, 1.0).reason, GateReason::kUnknownTag);
  EXPECT_EQ(
    gate.evaluate({make_detection(0, 1.0, 1.0, 0.0, 0.0, 49.9)}, 1.0).reason,
    GateReason::kLowMargin);
  EXPECT_EQ(
    gate.evaluate({make_detection(0, 1.0, 1.0, 0.0, 0.0, 60.0, 1)}, 1.0).reason,
    GateReason::kHamming);
  EXPECT_EQ(
    gate.evaluate({make_detection(), make_detection()}, 1.0).reason,
    GateReason::kMultiTag);
}

TEST(TagGate, RequiresThreeConsecutiveFrames)
{
  auto gate = make_gate();
  EXPECT_EQ(gate.evaluate({make_detection(0, 1.0)}, 1.0).reason, GateReason::kConfirming);
  EXPECT_EQ(gate.evaluate({make_detection(0, 1.1)}, 1.1).reason, GateReason::kConfirming);
  const auto result = gate.evaluate({make_detection(0, 1.2)}, 1.2);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.reason, GateReason::kAccepted);
  ASSERT_TRUE(result.dock.has_value());
  EXPECT_EQ(result.dock->dock_id, "dock_0");
}

TEST(TagGate, RateLimitsAndDetectsPoseJumps)
{
  auto gate = make_gate();
  gate.evaluate({make_detection(0, 1.0)}, 1.0);
  gate.evaluate({make_detection(0, 1.1)}, 1.1);
  EXPECT_EQ(gate.evaluate({make_detection(0, 1.2)}, 1.2).reason, GateReason::kAccepted);
  EXPECT_EQ(gate.evaluate({make_detection(0, 1.25)}, 1.25).reason, GateReason::kRateLimited);
  EXPECT_EQ(gate.evaluate({make_detection(0, 1.4, 1.3)}, 1.4).reason, GateReason::kPoseJump);
}

TEST(TagGate, HandlesLossAndRecovery)
{
  auto gate = make_gate();
  gate.evaluate({make_detection(0, 1.0)}, 1.0);
  EXPECT_FALSE(gate.loss_reason(1.5).has_value());
  const auto loss = gate.loss_reason(1.51);
  ASSERT_TRUE(loss.has_value());
  EXPECT_EQ(*loss, GateReason::kTagLost);
  EXPECT_FALSE(gate.loss_reason(2.0).has_value());
  EXPECT_EQ(gate.evaluate({make_detection(0, 2.0)}, 2.0).reason, GateReason::kConfirming);
}

TEST(TagGate, WrapsYawAcrossPi)
{
  constexpr double pi = 3.14159265358979323846;
  auto gate = make_gate();
  gate.evaluate({make_detection(0, 1.0, 1.0, 0.0, 179.0 * pi / 180.0)}, 1.0);
  gate.evaluate({make_detection(0, 1.1, 1.0, 0.0, 179.0 * pi / 180.0)}, 1.1);
  gate.evaluate({make_detection(0, 1.2, 1.0, 0.0, 179.0 * pi / 180.0)}, 1.2);
  const auto result = gate.evaluate(
    {make_detection(0, 1.4, 1.0, 0.0, -179.0 * pi / 180.0)}, 1.4);
  EXPECT_EQ(result.reason, GateReason::kAccepted);
}

}  // namespace
}  // namespace demo2_apriltag_docking
