// Unit tests for the lower-limit sensor decision helpers.
// These cover the pure logic behind the "limit-sensor" capability
// (debounced tracking and the protective downward stop). The full
// read()/write() integration path requires a live ros2_control executor and
// CAN bus and is exercised on hardware, not here.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "cubemars_hardware/can_utilities.hpp"

using cubemars_hardware::limit_debounce_update;
using cubemars_hardware::clamp_position_at_lower_limit;
using cubemars_hardware::clamp_downward_at_lower_limit;

// --- Debounced tracking (spec: "Debounced live lower-limit sensor state tracking")

TEST(LimitDebounce, AssertsOnlyAfterConsecutiveActiveFrames)
{
  int count = 0;
  EXPECT_FALSE(limit_debounce_update(count, 2, true));   // 1st active: not yet
  EXPECT_TRUE(limit_debounce_update(count, 2, true));    // 2nd active: asserted
  EXPECT_TRUE(limit_debounce_update(count, 2, true));    // stays asserted
}

TEST(LimitDebounce, TransientSpikeBelowThresholdIsIgnored)
{
  int count = 0;
  EXPECT_FALSE(limit_debounce_update(count, 3, true));   // spike, 1 of 3
  EXPECT_FALSE(limit_debounce_update(count, 3, true));   // spike, 2 of 3
  EXPECT_FALSE(limit_debounce_update(count, 3, false));  // clears before threshold
  EXPECT_EQ(count, 0);                                   // count reset
}

TEST(LimitDebounce, ClearsImmediatelyAndResetsCount)
{
  int count = 0;
  limit_debounce_update(count, 2, true);
  ASSERT_TRUE(limit_debounce_update(count, 2, true));    // asserted
  EXPECT_FALSE(limit_debounce_update(count, 2, false));  // single inactive clears
  EXPECT_EQ(count, 0);
  EXPECT_FALSE(limit_debounce_update(count, 2, true));   // must re-debounce from 0
}

TEST(LimitDebounce, FramesOneDisablesDebounce)
{
  int count = 0;
  EXPECT_TRUE(limit_debounce_update(count, 1, true));    // asserts on first active
}

// --- Protective downward stop, position (spec: "Protective downward stop...")

TEST(LimitPositionClamp, BlocksCommandBelowCurrentWhenAtLimit)
{
  // Commanded 0.10 below current 0.50 while at the limit -> held at current.
  EXPECT_DOUBLE_EQ(clamp_position_at_lower_limit(0.10, 0.50, true), 0.50);
}

TEST(LimitPositionClamp, AllowsAwayFromLimitCommand)
{
  // Commanded above current (moving up/away) passes through.
  EXPECT_DOUBLE_EQ(clamp_position_at_lower_limit(0.80, 0.50, true), 0.80);
}

TEST(LimitPositionClamp, NoClampWhenNotAtLimit)
{
  EXPECT_DOUBLE_EQ(clamp_position_at_lower_limit(0.10, 0.50, false), 0.10);
}

TEST(LimitPositionClamp, NanCurrentLeavesCommandUnchanged)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(clamp_position_at_lower_limit(0.10, nan, true), 0.10);
}

// --- Protective downward stop, velocity / effort sign-clamp

TEST(LimitDownwardClamp, BlocksNegativeWhenAtLimit)
{
  EXPECT_DOUBLE_EQ(clamp_downward_at_lower_limit(-2.5, true), 0.0);   // downward blocked
}

TEST(LimitDownwardClamp, AllowsPositiveWhenAtLimit)
{
  EXPECT_DOUBLE_EQ(clamp_downward_at_lower_limit(2.5, true), 2.5);    // away-from-limit passes
}

TEST(LimitDownwardClamp, NoClampWhenNotAtLimit)
{
  EXPECT_DOUBLE_EQ(clamp_downward_at_lower_limit(-2.5, false), -2.5); // fallback: unchanged
}

TEST(LimitDownwardClamp, ZeroPassesThrough)
{
  EXPECT_DOUBLE_EQ(clamp_downward_at_lower_limit(0.0, true), 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
