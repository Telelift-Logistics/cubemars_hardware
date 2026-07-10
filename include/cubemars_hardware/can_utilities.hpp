#ifndef CUBEMARS_HARDWARE__CAN_COMMANDS_HPP_
#define CUBEMARS_HARDWARE__CAN_COMMANDS_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace cubemars_hardware
{
    // ---------------------------------------------------------------------------
    // CAN payload constants
    // ---------------------------------------------------------------------------
    static inline constexpr std::uint8_t ZEROCMD[4] = {0, 0, 0, 0};

    static inline constexpr std::uint8_t MOTORENABLECMD[8] =
        {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    static inline constexpr std::uint8_t MOTORDISABLECMD[8] =
        {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

    // NOTE: This is the MIT-mode "set zero position" payload. CubeMars servo-mode
    // firmware may instead accept a 1-byte payload {0|1|2} on control mode 5
    // (SET_ORIGIN_MODE). Verify against the firmware revision deployed on your
    // motors. If the servo-mode form is required, replace this constant and
    // adjust the length in the write_message() call sites.
    static inline constexpr std::uint8_t SETZEROPOSCMD[8] =
        {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    // CubeMars AK servo-mode "set origin" payloads. Control mode 5
    // (SET_ORIGIN_MODE) takes a single byte:
    //   0x00 -- temporary zero (lost on power cycle); preferred for
    //           calibrate-on-startup workflows since it avoids wearing the
    //           motor's NVM.
    //   0x01 -- permanent zero (written to NVM; finite write endurance).
    //   0x02 -- restore the factory zero, discarding any prior set_origin.
    static inline constexpr std::uint8_t SET_ORIGIN_TEMPORARY[1] = {0x00};
    static inline constexpr std::uint8_t SET_ORIGIN_PERMANENT[1] = {0x01};
    static inline constexpr std::uint8_t SET_ORIGIN_RESTORE[1]   = {0x02};

    // ---------------------------------------------------------------------------
    // Joint kinematic limits (set from URDF)
    // ---------------------------------------------------------------------------
    /// @brief Per-joint mechanical range. Units match `use_meters_` (m or rad).
    struct JointLimits
    {
        double range{0.0};
    };

    // ---------------------------------------------------------------------------
    // Calibration phases
    // ---------------------------------------------------------------------------
    /// @brief Five-step calibration flow. See system.cpp::write() for the
    /// state transition diagram.
    ///
    ///   FIND_ROOT       -- drive in mount direction toward hard stop; detect
    ///                      via GPIO limit sensor, torque spike, or timeout.
    ///   SET_BOTTOM_ZERO -- issue SETZEROPOSCMD at the hard stop.
    ///   FIND_MIDSECTION -- move encoder to +/- range/2 (raw frame, no offset).
    ///   SET_MID_ZERO    -- issue SETZEROPOSCMD at midpoint; compute enc_offs_
    ///                      so that the reported position at the bottom == 0.
    ///   RETURN_TO_HOME  -- drive back to the bottom (now operational zero).
    ///   DONE            -- mark joint calibrated; calibration finished.
    ///   FAILED          -- terminal failure state; joint remains uncalibrated.
    enum struct CalibrationPhase : std::uint8_t
    {
        FIND_ROOT = 1,
        SET_BOTTOM_ZERO = 2,
        FIND_MIDSECTION = 3,
        SET_MID_ZERO = 4,
        RETURN_TO_HOME = 5,
        DONE = 6,
        FAILED = 7,
        SET_RETRY_ZERO = 8,
        DEFAULT = 0  // not running
    };

    inline const char * to_string(CalibrationPhase p)
    {
        switch (p) {
            case CalibrationPhase::FIND_ROOT: return "FIND_ROOT";
            case CalibrationPhase::SET_BOTTOM_ZERO: return "SET_BOTTOM_ZERO";
            case CalibrationPhase::FIND_MIDSECTION: return "FIND_MIDSECTION";
            case CalibrationPhase::SET_MID_ZERO: return "SET_MID_ZERO";
            case CalibrationPhase::RETURN_TO_HOME: return "RETURN_TO_HOME";
            case CalibrationPhase::SET_RETRY_ZERO: return "SET_RETRY_ZERO";
            case CalibrationPhase::DONE: return "DONE";
            case CalibrationPhase::FAILED: return "FAILED";
            case CalibrationPhase::DEFAULT: return "DEFAULT";
        }
        return "UNKNOWN";
    }

    // ---------------------------------------------------------------------------
    // Hardware-wide lift power state
    // ---------------------------------------------------------------------------
    /// @brief Top-level state of the lift power. Distinct from per-joint
    /// calibration state. Transitions are driven from read() based on GPIO
    /// power-state input and telemetry timeout.
    enum struct LiftPowerState : std::uint8_t
    {
        ONLINE = 0,      // power on, telemetry flowing, normal operation
        OFFLINE = 1,     // power confirmed off via GPIO or telemetry timeout
        RECOVERING = 2,  // telemetry just resumed; awaiting (re)calibration
    };

    inline const char * to_string(LiftPowerState s)
    {
        switch (s) {
            case LiftPowerState::ONLINE:     return "ONLINE";
            case LiftPowerState::OFFLINE:    return "OFFLINE";
            case LiftPowerState::RECOVERING: return "RECOVERING";
        }
        return "UNKNOWN";
    }

    // ---------------------------------------------------------------------------
    // Per-joint static calibration configuration (parsed from URDF on init)
    // ---------------------------------------------------------------------------
    /// @brief Static, joint-specific calibration tuning loaded from URDF.
    /// All position-like values are in the joint's reported units (m or rad,
    /// per the global `use_meters_` flag).
    struct CalibrationConfig
    {
        /// Step issued each cycle while searching for the hard stop.
        /// Default chosen to be safe at typical control rates; tune per joint.
        double search_step{0.01};

        /// Minimum interval between successive position steps during FIND_ROOT.
        /// Decouples the search rate from the controller update rate so the
        /// motor's inner loop has time to track each setpoint.
        std::chrono::milliseconds step_period{20};

        /// Position tolerance for "arrived at target" checks (FIND_MIDSECTION,
        /// RETURN_TO_HOME).
        double position_tolerance{0.005};

        /// Torque-based stall detection threshold during FIND_ROOT only.
        /// Intentionally separate from the operational `trq_limit` because
        /// stall detection during calibration usually wants a more sensitive
        /// threshold. Set <= 0 to disable torque detection.
        double stall_torque{0.0};

        /// Per-phase timeout. If a single phase does not progress within this
        /// duration the joint is marked FAILED.
        std::chrono::seconds phase_timeout{10};

        /// Settling delay after issuing SETZEROPOSCMD before the new zero is
        /// trusted in subsequent reads. Empirically 5-50 ms on CubeMars.
        std::chrono::milliseconds zero_settle{20};

        /// Whether this joint participates in calibration at all. Joints that
        /// have `bypass_calibration` set are pre-marked calibrated and skip
        /// the flow entirely.
        bool enabled{true};

        /// Name of the GPIO interface providing the lower-limit-sensor reading
        /// for this joint inside the gpio_state_msg interface group. Empty
        /// means no per-joint sensor.
        std::string gpio_ifc_name{};

        /// Name of the GPIO interface group containing `gpio_sensor_name`.
        /// Empty means no per-joint sensor.
        std::string gpio_group_name{};
    };

    // ---------------------------------------------------------------------------
    // Per-joint dynamic calibration runtime state
    // ---------------------------------------------------------------------------
    /// @brief Mutable per-joint state tracked during an active calibration run.
    /// Reset on every calibration entry.
    struct CalibrationRuntime
    {
        CalibrationPhase phase{CalibrationPhase::DEFAULT};
        std::chrono::steady_clock::time_point phase_started{};
        std::chrono::steady_clock::time_point last_step{};
        std::chrono::steady_clock::time_point zero_issued{};
        bool zero_cmd_pending{false};   // waiting for zero_settle to elapse
        bool limit_sensor_seen{false};  // latched once per phase
        double commanded_setpoint{0.0}; // raw-frame setpoint being driven to
        std::chrono::steady_clock::time_point mode_wait_started{}; // epoch = not waiting for position mode
        std::uint16_t retry_count{0};   // out-of-range retries used this run
    };
}

#endif  // CUBEMARS_HARDWARE__CAN_COMMANDS_HPP_