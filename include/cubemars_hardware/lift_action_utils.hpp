#ifndef CUBEMARS_HARDWARE__LIFT_ACTION_UTILS_HPP_
#define CUBEMARS_HARDWARE__LIFT_ACTION_UTILS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#include "cubemars_hardware/can_utilities.hpp"
#include "univan_control_msgs/action/lift_control.hpp"
#include "univan_control_msgs/msg/lift_joint_state.hpp"

namespace cubemars_hardware
{
    // ---------------------------------------------------------------------------
    // lift_controller action helpers (pure, unit-testable)
    // ---------------------------------------------------------------------------

    /// @brief What goal validation needs to know about one joint.
    struct LiftJointView
    {
        std::uint8_t can_id{0};
        bool calibration_enabled{false};
        bool enabled{false};  // motor state is ENABLE
    };

    /// @brief Outcome of check_lift_goal: an error code from LiftControl::Result
    /// and, when it is ERROR_NONE, the indices of the targeted joints.
    struct LiftGoalCheck
    {
        std::uint8_t error_code{0};
        std::string error_msg;
        std::vector<std::size_t> targets;
    };

    /// @brief Validate a LiftControl goal against the configured joints and
    /// resolve the joints it targets. Mirrors the checks of the
    /// lift_platform_hardware service; busy and power checks are left to the
    /// caller, as they depend on live state.
    inline LiftGoalCheck check_lift_goal(
        std::uint8_t operation, const std::vector<std::uint8_t> & can_ids,
        const std::vector<LiftJointView> & joints)
    {
        using Goal = univan_control_msgs::action::LiftControl::Goal;
        using Result = univan_control_msgs::action::LiftControl::Result;

        LiftGoalCheck check;
        auto fail = [&check](std::uint8_t code, std::string msg) {
            check.error_code = code;
            check.error_msg = std::move(msg);
            check.targets.clear();
            return check;
        };

        const bool calibrate = (operation == Goal::OPERATION_CALIBRATE);
        if (!calibrate && operation != Goal::OPERATION_ENABLE &&
            operation != Goal::OPERATION_DISABLE)
        {
            return fail(Result::ERROR_INVALID_GOAL,
                        "Unknown operation " + std::to_string(operation));
        }

        if (can_ids.empty()) {
            for (std::size_t i = 0; i < joints.size(); ++i) {
                if (!calibrate || joints[i].calibration_enabled) check.targets.push_back(i);
            }
            if (check.targets.empty()) {
                return fail(Result::ERROR_NOT_ELIGIBLE, "No joint is eligible for calibration");
            }
        } else {
            std::unordered_set<std::uint8_t> seen;
            for (const std::uint8_t id : can_ids) {
                const std::string can = "CAN " + std::to_string(id);
                if (!seen.insert(id).second) {
                    return fail(Result::ERROR_INVALID_GOAL, can + " is listed more than once");
                }
                std::size_t i = 0;
                while (i < joints.size() && joints[i].can_id != id) ++i;
                if (i == joints.size()) {
                    return fail(Result::ERROR_UNKNOWN_CAN_ID, "No joint configured for " + can);
                }
                if (calibrate && !joints[i].calibration_enabled) {
                    return fail(Result::ERROR_NOT_ELIGIBLE,
                                "Joint (" + can + ") is not eligible for calibration");
                }
                check.targets.push_back(i);
            }
        }

        if (calibrate) {
            for (const std::size_t i : check.targets) {
                if (!joints[i].enabled) {
                    return fail(Result::ERROR_JOINT_DISABLED,
                                "Joint (CAN " + std::to_string(joints[i].can_id) +
                                ") is disabled; enable it before calibrating");
                }
            }
        }
        return check;
    }

    /// @brief LiftJointState phase for an internal calibration phase.
    inline std::uint8_t to_phase_msg(CalibrationPhase p)
    {
        using Msg = univan_control_msgs::msg::LiftJointState;
        switch (p) {
            case CalibrationPhase::FIND_ROOT:       return Msg::PHASE_FIND_ROOT;
            case CalibrationPhase::SET_BOTTOM_ZERO: return Msg::PHASE_SET_BOTTOM_ZERO;
            case CalibrationPhase::FIND_MIDSECTION: return Msg::PHASE_FIND_MIDSECTION;
            case CalibrationPhase::SET_MID_ZERO:    return Msg::PHASE_SET_MID_ZERO;
            case CalibrationPhase::RETURN_TO_HOME:  return Msg::PHASE_RETURN_TO_HOME;
            case CalibrationPhase::DONE:            return Msg::PHASE_DONE;
            case CalibrationPhase::FAILED:          return Msg::PHASE_FAILED;
            case CalibrationPhase::SET_RETRY_ZERO:  return Msg::PHASE_SET_RETRY_ZERO;
            case CalibrationPhase::DEFAULT:         return Msg::PHASE_IDLE;
        }
        return Msg::PHASE_IDLE;
    }

    /// @brief LiftControl feedback power_state for an internal power state.
    inline std::uint8_t to_power_state_msg(LiftPowerState s)
    {
        using Feedback = univan_control_msgs::action::LiftControl::Feedback;
        switch (s) {
            case LiftPowerState::ONLINE:     return Feedback::POWER_ONLINE;
            case LiftPowerState::OFFLINE:    return Feedback::POWER_OFFLINE;
            case LiftPowerState::RECOVERING: return Feedback::POWER_RECOVERING;
        }
        return Feedback::POWER_OFFLINE;
    }

    /// @brief Readable name of a LiftControl operation, for logs and messages.
    inline const char * lift_operation_name(std::uint8_t operation)
    {
        using Goal = univan_control_msgs::action::LiftControl::Goal;
        switch (operation) {
            case Goal::OPERATION_CALIBRATE: return "CALIBRATE";
            case Goal::OPERATION_ENABLE:    return "ENABLE";
            case Goal::OPERATION_DISABLE:   return "DISABLE";
        }
        return "UNKNOWN";
    }
}

#endif  // CUBEMARS_HARDWARE__LIFT_ACTION_UTILS_HPP_
