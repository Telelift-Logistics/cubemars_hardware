// Unit tests for the lift_controller action helpers: goal validation (the
// checks the lift_platform_hardware service performs) and the mapping of the
// internal calibration and power states onto the action messages.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "cubemars_hardware/lift_action_utils.hpp"

using cubemars_hardware::CalibrationPhase;
using cubemars_hardware::LiftJointView;
using cubemars_hardware::LiftPowerState;
using cubemars_hardware::check_lift_goal;
using cubemars_hardware::to_phase_msg;
using cubemars_hardware::to_power_state_msg;
using Goal = univan_control_msgs::action::LiftControl::Goal;
using Result = univan_control_msgs::action::LiftControl::Result;
using Feedback = univan_control_msgs::action::LiftControl::Feedback;
using JointMsg = univan_control_msgs::msg::LiftJointState;

namespace
{
// CAN 1: lift, calibrating, enabled. CAN 2: no calibration, enabled.
// CAN 3: calibrating, disabled.
const std::vector<LiftJointView> kJoints{
  {1, true, true},
  {2, false, true},
  {3, true, false},
};
}  // namespace

// --- Goal validation

TEST(LiftGoal, UnknownOperationIsInvalid)
{
  const auto check = check_lift_goal(7, {1}, kJoints);
  EXPECT_EQ(check.error_code, Result::ERROR_INVALID_GOAL);
  EXPECT_TRUE(check.targets.empty());
}

TEST(LiftGoal, DuplicateCanIdIsInvalid)
{
  EXPECT_EQ(check_lift_goal(Goal::OPERATION_ENABLE, {1, 1}, kJoints).error_code,
            Result::ERROR_INVALID_GOAL);
}

TEST(LiftGoal, UnknownCanIdIsRejected)
{
  EXPECT_EQ(check_lift_goal(Goal::OPERATION_ENABLE, {1, 9}, kJoints).error_code,
            Result::ERROR_UNKNOWN_CAN_ID);
}

TEST(LiftGoal, CalibrateRejectsJointWithoutCalibration)
{
  EXPECT_EQ(check_lift_goal(Goal::OPERATION_CALIBRATE, {2}, kJoints).error_code,
            Result::ERROR_NOT_ELIGIBLE);
}

TEST(LiftGoal, CalibrateRejectsDisabledJoint)
{
  EXPECT_EQ(check_lift_goal(Goal::OPERATION_CALIBRATE, {3}, kJoints).error_code,
            Result::ERROR_JOINT_DISABLED);
}

TEST(LiftGoal, CalibrateListedJoint)
{
  const auto check = check_lift_goal(Goal::OPERATION_CALIBRATE, {1}, kJoints);
  EXPECT_EQ(check.error_code, Result::ERROR_NONE);
  EXPECT_EQ(check.targets, (std::vector<std::size_t>{0}));
}

TEST(LiftGoal, EmptyCalibrateTargetsCalibratingJointsAndChecksTheyAreEnabled)
{
  // CAN 1 and 3 calibrate, and CAN 3 is disabled.
  EXPECT_EQ(check_lift_goal(Goal::OPERATION_CALIBRATE, {}, kJoints).error_code,
            Result::ERROR_JOINT_DISABLED);

  const std::vector<LiftJointView> enabled{{1, true, true}, {2, false, true}, {3, true, true}};
  const auto check = check_lift_goal(Goal::OPERATION_CALIBRATE, {}, enabled);
  EXPECT_EQ(check.error_code, Result::ERROR_NONE);
  EXPECT_EQ(check.targets, (std::vector<std::size_t>{0, 2}));
}

TEST(LiftGoal, EmptyCalibrateWithNoEligibleJointIsRejected)
{
  EXPECT_EQ(check_lift_goal(Goal::OPERATION_CALIBRATE, {}, {{2, false, true}}).error_code,
            Result::ERROR_NOT_ELIGIBLE);
}

TEST(LiftGoal, EmptyStateChangeTargetsEveryJoint)
{
  const auto check = check_lift_goal(Goal::OPERATION_DISABLE, {}, kJoints);
  EXPECT_EQ(check.error_code, Result::ERROR_NONE);
  EXPECT_EQ(check.targets, (std::vector<std::size_t>{0, 1, 2}));
}

TEST(LiftGoal, EnableAcceptsDisabledAndNonCalibratingJoints)
{
  const auto check = check_lift_goal(Goal::OPERATION_ENABLE, {3, 2}, kJoints);
  EXPECT_EQ(check.error_code, Result::ERROR_NONE);
  EXPECT_EQ(check.targets, (std::vector<std::size_t>{2, 1}));  // goal order
}

// --- State mapping

TEST(LiftStateMapping, EveryCalibrationPhaseMapsToItsMessagePhase)
{
  EXPECT_EQ(to_phase_msg(CalibrationPhase::DEFAULT), JointMsg::PHASE_IDLE);
  EXPECT_EQ(to_phase_msg(CalibrationPhase::FIND_ROOT), JointMsg::PHASE_FIND_ROOT);
  EXPECT_EQ(to_phase_msg(CalibrationPhase::SET_BOTTOM_ZERO), JointMsg::PHASE_SET_BOTTOM_ZERO);
  EXPECT_EQ(to_phase_msg(CalibrationPhase::FIND_MIDSECTION), JointMsg::PHASE_FIND_MIDSECTION);
  EXPECT_EQ(to_phase_msg(CalibrationPhase::SET_MID_ZERO), JointMsg::PHASE_SET_MID_ZERO);
  EXPECT_EQ(to_phase_msg(CalibrationPhase::RETURN_TO_HOME), JointMsg::PHASE_RETURN_TO_HOME);
  EXPECT_EQ(to_phase_msg(CalibrationPhase::DONE), JointMsg::PHASE_DONE);
  EXPECT_EQ(to_phase_msg(CalibrationPhase::FAILED), JointMsg::PHASE_FAILED);
  EXPECT_EQ(to_phase_msg(CalibrationPhase::SET_RETRY_ZERO), JointMsg::PHASE_SET_RETRY_ZERO);
}

TEST(LiftStateMapping, MessagePhasesShareTheInternalValues)
{
  // The result's failure message casts a message phase back to name it.
  for (std::uint8_t v = 0; v <= 8; ++v) {
    EXPECT_EQ(to_phase_msg(static_cast<CalibrationPhase>(v)), v);
  }
}

TEST(LiftStateMapping, PowerStates)
{
  EXPECT_EQ(to_power_state_msg(LiftPowerState::ONLINE), Feedback::POWER_ONLINE);
  EXPECT_EQ(to_power_state_msg(LiftPowerState::OFFLINE), Feedback::POWER_OFFLINE);
  EXPECT_EQ(to_power_state_msg(LiftPowerState::RECOVERING), Feedback::POWER_RECOVERING);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
