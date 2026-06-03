#pragma once

namespace test_sequences {

/**
 * @brief Run the direct motor bring-up sequence for all four drivetrain motors.
 * @details Each motor is driven forward, then reverse, with a fixed PWM duty cycle and blocking delays between phases.
 * @author GitHub Copilot
 * @date 2026-06-03
 */
void testMotors();

}  // namespace test_sequences