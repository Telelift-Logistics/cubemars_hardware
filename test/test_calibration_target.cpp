// Unit tests for where midpoint calibration finishes: the stored lift
// position when usable, the bottom otherwise.

#include <limits>

#include <gtest/gtest.h>

#include "cubemars_hardware/can_utilities.hpp"

using cubemars_hardware::calibration_return_target;
using cubemars_hardware::command_hold_released;

TEST(CalibrationReturnTarget, UsesStoredPositionWithinRange)
{
  EXPECT_DOUBLE_EQ(calibration_return_target(0.25, 0.5), 0.25);
  EXPECT_DOUBLE_EQ(calibration_return_target(0.0, 0.5), 0.0);   // bottom edge
  EXPECT_DOUBLE_EQ(calibration_return_target(0.5, 0.5), 0.5);   // top edge
}

TEST(CalibrationReturnTarget, FallsBackToBottomWhenOutOfRange)
{
  EXPECT_DOUBLE_EQ(calibration_return_target(-0.01, 0.5), 0.0);
  EXPECT_DOUBLE_EQ(calibration_return_target(0.51, 0.5), 0.0);
}

TEST(CalibrationReturnTarget, FallsBackToBottomWhenNotFinite)
{
  EXPECT_DOUBLE_EQ(
    calibration_return_target(std::numeric_limits<double>::quiet_NaN(), 0.5), 0.0);
  EXPECT_DOUBLE_EQ(
    calibration_return_target(std::numeric_limits<double>::infinity(), 0.5), 0.0);
}

// --- Hold on the controller's command after a calibration or lowering

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kTol = 0.005;
}  // namespace

TEST(CommandHold, StaleCommandKeepsTheHold)
{
  // Lowered to 0 while the controller still holds the previous height 0.4.
  EXPECT_FALSE(command_hold_released(0.4, 0.4, 0.0, kTol));
}

TEST(CommandHold, NewCommandReleasesTheHold)
{
  EXPECT_TRUE(command_hold_released(0.3, 0.4, 0.0, kTol));
  // Motion control re-seeding its target to where the lift is matches the target.
  EXPECT_TRUE(command_hold_released(0.0, 0.4, 0.0, kTol));
}

TEST(CommandHold, CommandAtTheTargetReleasesTheHold)
{
  // The stale command already equals where the lift was left.
  EXPECT_TRUE(command_hold_released(0.25, 0.25, 0.25, kTol));
  EXPECT_TRUE(command_hold_released(0.252, 0.4, 0.25, kTol));
}

TEST(CommandHold, NoCommandKeepsTheHold)
{
  EXPECT_FALSE(command_hold_released(kNaN, 0.4, 0.0, kTol));
  EXPECT_FALSE(command_hold_released(kNaN, kNaN, 0.0, kTol));
}

TEST(CommandHold, FirstCommandAfterNoneReleasesTheHold)
{
  EXPECT_TRUE(command_hold_released(0.4, kNaN, 0.0, kTol));
}

TEST(CommandHold, UnknownTargetOnlyReleasesOnChange)
{
  EXPECT_FALSE(command_hold_released(0.4, 0.4, kNaN, kTol));
  EXPECT_TRUE(command_hold_released(0.1, 0.4, kNaN, kTol));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
