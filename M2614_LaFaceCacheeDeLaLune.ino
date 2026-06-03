#include <Arduino_RouterBridge.h>
#include <zephyr/kernel.h>

#include "LiDAR/LiDARSensor.h"
#include "PINS.h"
#include "debug/debug_print.h"
#include "debug/test_sequences.h"
#include "driver/FeedbackEncoder.h"
#include "driver/MecanumDriver.h"
#include "driver/RemoteController.h"
#include "driver/SpeedController.h"

MotorPinConfig frontLeftPins = {FL_IN1, FL_IN2, FL_EN};
MotorPinConfig frontRightPins = {FR_IN1, FR_IN2, FR_EN};
MotorPinConfig rearLeftPins = {BL_IN1, BL_IN2, BL_EN};
MotorPinConfig rearRightPins = {BR_IN1, BR_IN2, BR_EN};
MecanumPins mecanumPins = {frontLeftPins, frontRightPins, rearLeftPins, rearRightPins};
MecanumDriver mecanumDriver = MecanumDriver(mecanumPins);

RCReceiverPins rcPins = {RC_PIN_A, RC_PIN_B, RC_PIN_C, RC_PIN_D, RC_PIN_E, RC_PIN_F, RC_PIN_G, RC_PIN_H};
RemoteController rc = RemoteController(rcPins);

EncoderChannelPins frontLeftEncoderPins = {FL_ENC_A, FL_ENC_B};
EncoderChannelPins frontRightEncoderPins = {FR_ENC_A, FR_ENC_B};
EncoderChannelPins rearLeftEncoderPins = {BL_ENC_A, BL_ENC_B};
EncoderChannelPins rearRightEncoderPins = {BR_ENC_A, BR_ENC_B};
FeedbackEncoderPins encoderPins = {
    frontLeftEncoderPins,
    frontRightEncoderPins,
    rearLeftEncoderPins,
    rearRightEncoderPins,
};
FeedbackEncoder encoders = FeedbackEncoder(encoderPins);
LiDARSensor LiDAR;

namespace {
constexpr uint32_t kControlPeriodMs = 50;
constexpr float kDefaultDtSeconds = 0.05f;
constexpr float kMaxSpeedPulsesPerPeriod = 80.0f;
constexpr int16_t kJoystickDeadzone = 30;

uint32_t lastControlUpdateMs = 0;

int16_t applyDeadzone(int16_t value, int16_t deadzone) {
    return (value > -deadzone && value < deadzone) ? 0 : value;
}

float wheelCommandToSpeedSetpoint(int16_t wheelCommand) {
    return (static_cast<float>(wheelCommand) / 500.0f) * kMaxSpeedPulsesPerPeriod;
}

float signedMeasuredSpeed(int32_t rawPulseCount, float setpoint) {
    if (setpoint > 0.0f) {
        return static_cast<float>(rawPulseCount);
    }
    if (setpoint < 0.0f) {
        return static_cast<float>(-rawPulseCount);
    }
    return 0.0f;
}

// void resetSpeedControllers(SpeedController& frontLeft, SpeedController& frontRight, SpeedController& rearLeft, SpeedController& rearRight) {
//     frontLeft.reset();
//     frontRight.reset();
//     rearLeft.reset();
//     rearRight.reset();
// }
}  // namespace

// SpeedControllerConfig speedControllerConfig = {
//     .kp = 6.0f,
//     .ki = 0.5f,
//     .kd = 0.0f,
//     .integralMin = -400.0f,
//     .integralMax = 400.0f,
//     .outputMin = -500.0f,
//     .outputMax = 500.0f,
// };
//
// PIDSpeedController frontLeftSpeedController(speedControllerConfig);
// PIDSpeedController frontRightSpeedController(speedControllerConfig);
// PIDSpeedController rearLeftSpeedController(speedControllerConfig);
// PIDSpeedController rearRightSpeedController(speedControllerConfig);

void setup() {
    Monitor.begin();
    Monitor.println("===============================");
    Monitor.println("Starting up...");
    pinSetup();
    Serial1.begin(LDS_LDROBOT_LD19::SERIAL_BAUD_RATE);
    LiDAR.begin(Serial1);
    mecanumDriver.begin();
    rc.begin();
    encoders.begin();
    lastControlUpdateMs = millis();
    Monitor.println("Setup complete.");
}

void loop() {
    const uint32_t rcUpdateNowUs = micros();
    debug_print::noteRcUpdateCadence(rcUpdateNowUs);
    rc.update();
    LiDAR.update();

    const uint32_t nowMs = millis();
    const uint32_t elapsedMs = nowMs - lastControlUpdateMs;
    if (elapsedMs < kControlPeriodMs) return;

    lastControlUpdateMs = nowMs;

    // Print LiDAR query diagnostics at low control rate.
    const LiDARSensor::QueryResult lidarAt0Deg = LiDAR.queryAngleAt(0);
    const LiDARSensor::QueryResult lidarAt90Deg = LiDAR.queryAngleAt(90);
    const LiDARSensor::QueryResult lidarAt270Deg = LiDAR.queryAngleAt(270);
    // Monitor.println(
    //     String("[DEBUG] LiDAR scan=") + String(LiDAR.getLastCommittedScanId()) +
    //     " points=" + String(LiDAR.getLastValidPointCount()) +
    //     " dropped=" + String(LiDAR.getDroppedScanCount()) +
    //     " | 0°=" + String(lidarAt0Deg.distance_mm) + "@" + String(lidarAt0Deg.matched_angle_deg_x100 * 0.01f, 2) +
    //     " | 90°=" + String(lidarAt90Deg.distance_mm) + "@" + String(lidarAt90Deg.matched_angle_deg_x100 * 0.01f, 2) +
    //     " | 270°=" + String(lidarAt270Deg.distance_mm) + "@" + String(lidarAt270Deg.matched_angle_deg_x100 * 0.01f, 2));
    debug_print::printLidarDistances(lidarAt0Deg, lidarAt90Deg, lidarAt270Deg);

    const bool hasDriveSignal = rc.isSignalValid(RCChannel::A) && rc.isSignalValid(RCChannel::B) && rc.isSignalValid(RCChannel::D);
    if (!hasDriveSignal) {
        // debug_print::printRcDebugSummary(rc, nowMs, elapsedMs);
        mecanumDriver.stop();
        // resetSpeedControllers(frontLeftSpeedController, frontRightSpeedController, rearLeftSpeedController, rearRightSpeedController);
        return;
    }

    // debug_print::printDriveRcSnapshot(rc);
    // debug_print::printRcUpdateCadence();
    const int16_t longitudinalCommand = applyDeadzone(rc.getJoystick(RCChannel::A), kJoystickDeadzone);
    const int16_t lateralCommand = applyDeadzone(rc.getJoystick(RCChannel::B), kJoystickDeadzone);
    const int16_t rotationCommand = applyDeadzone(rc.getJoystick(RCChannel::D), kJoystickDeadzone);

    const MecanumDriver::WheelCommands targetWheelCommands = MecanumDriver::mix(lateralCommand, longitudinalCommand, rotationCommand);

    const float frontLeftSetpoint = wheelCommandToSpeedSetpoint(targetWheelCommands.frontLeft);
    const float frontRightSetpoint = wheelCommandToSpeedSetpoint(targetWheelCommands.frontRight);
    const float rearLeftSetpoint = wheelCommandToSpeedSetpoint(targetWheelCommands.rearLeft);
    const float rearRightSetpoint = wheelCommandToSpeedSetpoint(targetWheelCommands.rearRight);

    const EncoderSpeedSnapshot measuredSpeeds = encoders.getCurrentSpeed();

    const float dtSeconds = (elapsedMs > 0U) ? (static_cast<float>(elapsedMs) / 1000.0f) : kDefaultDtSeconds;

    // const float frontLeftCommand = frontLeftSpeedController.update(frontLeftSetpoint, signedMeasuredSpeed(measuredSpeeds.frontLeft, frontLeftSetpoint), dtSeconds);
    // const float frontRightCommand = frontRightSpeedController.update(frontRightSetpoint, signedMeasuredSpeed(measuredSpeeds.frontRight, frontRightSetpoint), dtSeconds);
    // const float rearLeftCommand = rearLeftSpeedController.update(rearLeftSetpoint, signedMeasuredSpeed(measuredSpeeds.rearLeft, rearLeftSetpoint), dtSeconds);
    // const float rearRightCommand = rearRightSpeedController.update(rearRightSetpoint, signedMeasuredSpeed(measuredSpeeds.rearRight, rearRightSetpoint), dtSeconds);

    // We temporarily scale up by the time we fix the PID Speed regulator
    constexpr uint8_t speedScalingFactor = 4;
    constexpr uint8_t speedOffset = 0;
    const float frontLeftCommand = frontLeftSetpoint * speedScalingFactor + speedOffset;
    const float frontRightCommand = frontRightSetpoint * speedScalingFactor + speedOffset;
    const float rearLeftCommand = rearLeftSetpoint * speedScalingFactor + speedOffset;
    const float rearRightCommand = rearRightSetpoint * speedScalingFactor + speedOffset;

    // Somehow, we need to scale the computed wheel commands. It shouldn't be the case so let's debug print some values over here
    // debug_print::printScaledWheelCommands(frontLeftCommand, frontRightCommand, rearLeftCommand, rearRightCommand);
    // debug_print::printTargetWheelCommands(targetWheelCommands);

    mecanumDriver.driveWheels(static_cast<int16_t>(frontLeftCommand), static_cast<int16_t>(frontRightCommand), static_cast<int16_t>(rearLeftCommand), static_cast<int16_t>(rearRightCommand));
}