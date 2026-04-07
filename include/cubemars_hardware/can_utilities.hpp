#ifndef CUBEMARS_HARDWARE__CAN_COMMANDS_HPP_
#define CUBEMARS_HARDWARE__CAN_COMMANDS_HPP_

#include <array>
#include <cstdint>

namespace cubemars_hardware
{
    static inline constexpr std::uint8_t ZEROCMD[4] = {0, 0, 0, 0};

    static inline constexpr std::uint8_t MOTORENABLECMD[8] = 
        {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

     static inline constexpr std::uint8_t MOTORDISABLECMD[8] = 
        {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

     static inline constexpr std::uint8_t SETZEROPOSCMD[8] = 
        {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    
    static inline constexpr uint8_t CALIBRATIONSTEP = 10;
    static inline constexpr uint8_t CALIBRATIONTHRESHOLD = 1;

    /// @brief JointLimits struct.
    struct JointLimits {
        double range;
        // double min_virt_pos;
        // double max_virt_pos;
    };

    /// @brief Possible phases of motor calibration.
    enum struct CalibrationPhase: uint8_t
    {
        FIND_ROOT = 1, // Try to find the minimum of the axis
        FIND_MIDSECTION = 2, // Go to the midpoint of the configured range
        SET_VIRTUAL_ZERO = 3, // Set virtual zero for subsequent operation
        INIT = 4, // Go to new initial position of axis
        DEFAULT // Done/ Do nothing
    };
}

#endif  // CUBEMARS_HARDWARE__CAN_COMMANDS_HPP_