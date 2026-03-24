/**
 * Stub implementation of delto_gripper_helper for arm64/aarch64 builds.
 *
 * The vendor-provided libdelto_gripper_helper.so is x86_64 only.
 * This stub provides passthrough implementations sufficient for
 * JointTrajectoryController (position control) mode.
 *
 * For effort/current control, request the arm64 binary from:
 *   support@tesollo.com
 */

#include <vector>
#include "delto_hardware/delto_gripper_helper.hpp"

namespace delto_gripper_helper {

double GetLibraryVersion() {
    return 0.0;  // stub
}

std::vector<double> ConvertEffort(const std::vector<double>& current) {
    // Passthrough: treat current (mA) as effort directly
    return current;
}

std::vector<double> CurrentControl(
    int joint_count,
    const std::vector<int>& /*actual_current*/,
    const std::vector<double>& target_torque,
    std::vector<int>& current_limit_flag,
    std::vector<double>& /*current_integral*/)
{
    // Passthrough: return target_torque as-is (no PID filtering)
    current_limit_flag.assign(joint_count, 0);
    return target_torque;
}

std::vector<double> ConvertDuty(int /*joint_count*/,
                                std::vector<double> target_torque) {
    // Passthrough: treat torque as duty directly
    return target_torque;
}

}  // namespace delto_gripper_helper
