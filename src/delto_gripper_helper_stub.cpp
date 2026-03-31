/**
 * Reverse-engineered implementation of delto_gripper_helper for arm64/aarch64.
 *
 * The vendor-provided libdelto_gripper_helper.so is x86_64 only.
 * Constants and logic extracted from the x86_64 binary via disassembly.
 *
 * ConvertDuty:    duty = clamp(torque * 100.54, -100, 100)
 * CurrentControl: current limiting with hysteresis (170/129 mA thresholds)
 * ConvertEffort:  current(mA) * scale → effort (extracted from binary)
 */

#include <cmath>
#include <algorithm>
#include <vector>
#include "delto_hardware/delto_gripper_helper.hpp"

namespace delto_gripper_helper {

// Constants extracted from .rodata section of x86_64 binary
static constexpr double DUTY_SCALE_A   = 12.06521739130435;  // 0x60e8
static constexpr double DUTY_SCALE_B   = 12.0;               // 0x60f0
static constexpr double DUTY_MAX       = 100.0;              // 0x60f8
static constexpr double DUTY_MIN       = -100.0;             // 0x6100
static constexpr int    CURRENT_LIMIT_HIGH = 170;            // 0xAA
static constexpr int    CURRENT_LIMIT_LOW  = 129;            // 0x81
static constexpr double EFFORT_SCALE   = 13.875;             // 0x6008
static constexpr double EFFORT_FACTOR  = 1.15;               // 0x6010

double GetLibraryVersion() {
    return 0.1;  // from 0x6030
}

std::vector<double> ConvertEffort(const std::vector<double>& current) {
    // Convert current (mA) to effort using extracted scale factors
    std::vector<double> effort(current.size());
    for (size_t i = 0; i < current.size(); ++i) {
        effort[i] = current[i] / EFFORT_SCALE * EFFORT_FACTOR;
    }
    return effort;
}

std::vector<double> CurrentControl(
    int joint_count,
    const std::vector<int>& actual_current,
    const std::vector<double>& target_torque,
    std::vector<int>& current_limit_flag,
    std::vector<double>& current_integral)
{
    std::vector<double> output(joint_count, 0.0);

    for (int i = 0; i < joint_count; ++i) {
        int abs_current = std::abs(actual_current[i]);

        // Hysteresis current limiting (from disassembly)
        if (abs_current > CURRENT_LIMIT_HIGH) {
            // Over-current: set limit flag
            current_limit_flag[i] = 1;
        } else if (abs_current <= CURRENT_LIMIT_LOW) {
            // Normal: clear limit flag and reset integral
            current_limit_flag[i] = 0;
            current_integral[i] = 0.0;
        }

        // Passthrough: deliver target_torque directly to preserve P gain effect.
        // Only the hysteresis current limiter (above) restricts output.
        // This matches the vendor binary behavior: full effort delivery
        // with protection only when actual motor current exceeds limits.
        // If oscillation occurs, add D gain in the controller yaml, not here.
        if (current_limit_flag[i] == 0) {
            output[i] = target_torque[i];
        } else {
            output[i] = 0.0;
        }
    }

    return output;
}

std::vector<double> ConvertDuty(int /*joint_count*/,
                                std::vector<double> target_torque) {
    // duty = clamp(torque * DUTY_SCALE_A / DUTY_SCALE_B * DUTY_MAX, -100, 100)
    // Effective: duty = clamp(torque * 100.5434..., -100, 100)
    std::vector<double> duty(target_torque.size());
    for (size_t i = 0; i < target_torque.size(); ++i) {
        double d = target_torque[i] * DUTY_SCALE_A / DUTY_SCALE_B * DUTY_MAX;
        duty[i] = std::clamp(d, DUTY_MIN, DUTY_MAX);
    }
    return duty;
}

}  // namespace delto_gripper_helper
