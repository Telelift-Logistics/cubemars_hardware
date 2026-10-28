// Unit tests for where midpoint calibration finishes: the stored lift
// position when usable, the bottom otherwise.

#include <limits>

#include <gtest/gtest.h>

#include "cubemars_hardware/can_utilities.hpp"

using cubemars_hardware::calibration_return_target;

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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
