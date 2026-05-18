#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include <Arduino.h>

#include "MotorCalibration.h"
#include "SpeedController.h"

namespace DriveSystem {

/// Initialise the drive system: encoders, calibration, control thread.
/// Call once in setup() after pinSetup() and Monitor.begin().
/// @param motors  Array of 4 MotorPins (FL, FR, BL, BR).
/// @return true if calibration succeeded and the thread started.
bool init(const MotorPins motors[4]);

/// Set the velocity command from the joystick.
/// @param vx    Strafe: −1000 left, +1000 right.
/// @param vy    Forward/back: −1000 back, +1000 forward.
/// @param omega Rotation: −1000 CCW, +1000 CW.
void setVelocity(int16_t vx, int16_t vy, int16_t omega);

/// Emergency stop — all motors off, PID reset.
void stop();

/// Change the regulator mode on all four controllers at once.
void setRegulatorMode(RegulatorMode mode);

/// Change PID gains on all four controllers at once.
void setGains(float kp, float ki, float kd);

/// Change ramp limits on all four controllers at once.
void setRampLimits(float accelPpsPerSec, float decelPpsPerSec);

/// Return true if the drive system has been initialised and the thread is running.
bool isRunning();

}  // namespace DriveSystem

#endif
