#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

#include <Arduino.h>

/// Motor pin descriptor (matches the struct used in the main sketch).
struct MotorPins {
    uint8_t in1;
    uint8_t in2;
    uint8_t en;
};

namespace MotorCalibration {

/// Run the startup calibration sequence.
/// Spins each motor CW then CCW at full PWM, measures encoder pps,
/// and stores the slowest reading as the global speed reference.
/// Prints results to Monitor.
/// @param motors     Array of 4 MotorPins (FL, FR, BL, BR order).
/// @param testDurationMs  How long to spin each motor per direction.
/// @return The maximum reference pps (slowest motor at full PWM), or 0 on error.
int32_t calibrate(const MotorPins motors[4], uint32_t testDurationMs = 1000);

/// Get the max reference pps determined during the last calibration.
int32_t getMaxReferencePps();

}  // namespace MotorCalibration

#endif
