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

enum ControlState {
    MANUAL_CONTROL,
    AUTOMATIC_CONTROL,
    CONNECTION_LOST
};

enum AutoControlState {
    FORWARD1,
    TURN1,
    FORWARD2,
    TURN2,
    FORWARD3,
    TURN3,
    FORWARD4,
    TURN4,
    FORWARD5,
    TURN5,
    FORWARD6,
    TURN6,
    POSITIONNING,
    COMPLETED
};

ControlState CURRENT_STATE = ControlState::MANUAL_CONTROL;
AutoControlState AUTO_STATE = FORWARD1;
namespace {
constexpr uint32_t kControlPeriodMs = 50;
constexpr uint32_t kAutoSwitchLogPeriodMs = 250;
constexpr uint32_t kRcSignalLossDebounceMs = 250;
constexpr float kDefaultDtSeconds = 0.05f;
constexpr float kMaxSpeedPulsesPerPeriod = 80.0f;
constexpr int16_t kJoystickDeadzone = 30;
constexpr uint16_t kButtonPressThresholdUs = 1850;

uint32_t lastControlUpdateMs = 0;
uint32_t driveSignalInvalidSinceMs = 0;

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
    // const uint32_t rcUpdateNowUs = micros();
    // debug_print::noteRcUpdateCadence(rcUpdateNowUs);
    rc.update();
    LiDAR.update();

    const uint32_t nowMs = millis();
    const uint32_t elapsedMs = nowMs - lastControlUpdateMs;

    constexpr LiDARSensor::QueryResult defaultQueryResult = {
        false,
        0U,
        0U,
        -1,
        0,
        0U,
        0U,
        0U};

    LiDARSensor::QueryResult lidarAt0Deg = defaultQueryResult;
    LiDARSensor::QueryResult lidarAt90Deg = defaultQueryResult;
    LiDARSensor::QueryResult lidarAt270Deg = defaultQueryResult;

    const bool hasDriveSignal = rc.isSignalValid(RCChannel::A) && rc.isSignalValid(RCChannel::B) && rc.isSignalValid(RCChannel::D);
    if (hasDriveSignal) {
        driveSignalInvalidSinceMs = 0;
        if (CURRENT_STATE == ControlState::CONNECTION_LOST) {
            CURRENT_STATE = ControlState::MANUAL_CONTROL;
            Monitor.println("=============== RC SIGNAL RESTORED ===============");
        }
    } else {
        if (driveSignalInvalidSinceMs == 0U) {
            driveSignalInvalidSinceMs = nowMs;
        }

        const bool lossDebounceElapsed = (nowMs - driveSignalInvalidSinceMs) >= kRcSignalLossDebounceMs;
        if (lossDebounceElapsed && CURRENT_STATE != ControlState::CONNECTION_LOST) {
            CURRENT_STATE = ControlState::CONNECTION_LOST;
            Monitor.println("=============== RC SIGNAL LOST ===============");
        }
    }

    int16_t longitudinalCommand = 0;
    int16_t lateralCommand = 0;
    int16_t rotationCommand = 0;

    switch (CURRENT_STATE) {
        case ControlState::MANUAL_CONTROL: {
            // In manual control, we directly map RC joystick commands to mecanum drive.
            longitudinalCommand = applyDeadzone(rc.getJoystick(RCChannel::A), kJoystickDeadzone);
            lateralCommand = applyDeadzone(rc.getJoystick(RCChannel::B), kJoystickDeadzone);
            rotationCommand = applyDeadzone(rc.getJoystick(RCChannel::D), kJoystickDeadzone);

            // If we press both joystick buttons, we switch to automatic control mode.
            static bool wasAutoSwitchPressed = false;
            static uint32_t lastAutoSwitchLogMs = 0;
            const bool hasAutoSwitchSignal = rc.isSignalValid(RCChannel::E) && rc.isSignalValid(RCChannel::F);
            const bool isAutoSwitchPressed = hasAutoSwitchSignal && rc.getButton(RCChannel::E, kButtonPressThresholdUs) && rc.getButton(RCChannel::F, kButtonPressThresholdUs);
            const uint16_t pulseE = rc.getPulseWidthUs(RCChannel::E);
            const uint16_t pulseF = rc.getPulseWidthUs(RCChannel::F);

            if ((nowMs - lastAutoSwitchLogMs) >= kAutoSwitchLogPeriodMs) {
                lastAutoSwitchLogMs = nowMs;
                Monitor.print("AUTO_SWITCH E_valid=");
                Monitor.print(rc.isSignalValid(RCChannel::E));
                Monitor.print(" F_valid=");
                Monitor.print(rc.isSignalValid(RCChannel::F));
                Monitor.print(" E_us=");
                Monitor.print(pulseE);
                Monitor.print(" F_us=");
                Monitor.print(pulseF);
                Monitor.print(" thresh=");
                Monitor.print(kButtonPressThresholdUs);
                Monitor.print(" pressed=");
                Monitor.print(isAutoSwitchPressed);
                Monitor.print(" edge=");
                Monitor.println(isAutoSwitchPressed && !wasAutoSwitchPressed);
            }

            if (isAutoSwitchPressed && !wasAutoSwitchPressed) {
                CURRENT_STATE = ControlState::AUTOMATIC_CONTROL;
                AUTO_STATE = FORWARD1;
                Monitor.println("=============== SWITCHING TO AUTOMATIC CONTROL MODE ===============");
            }
            wasAutoSwitchPressed = isAutoSwitchPressed;
            break;
        }
        case ControlState::AUTOMATIC_CONTROL:
            lidarAt0Deg = LiDAR.queryAngleAt(0);
            lidarAt90Deg = LiDAR.queryAngleAt(90);
            lidarAt270Deg = LiDAR.queryAngleAt(270);

            switch (AUTO_STATE) {
                case FORWARD1:
                    longitudinalCommand = 150;
                    if (lidarAt90Deg.distance_mm > lidarAt270Deg.distance_mm * 2) longitudinalCommand = 75;
                    if (lidarAt0Deg.distance_mm <= lidarAt270Deg.distance_mm) {
                        AUTO_STATE = TURN1;
                        Monitor.print("[DEBUG] Transitioning to TURN1: lidarAt0Deg.distance_mm=");
                        Monitor.print(lidarAt0Deg.distance_mm);
                        Monitor.print(" lidarAt270Deg.distance_mm=");
                        Monitor.println(lidarAt270Deg.distance_mm);
                    }
                    break;
                case TURN1:
                    rotationCommand = 200;
                    if (lidarAt0Deg.distance_mm > lidarAt270Deg.distance_mm * 4) {
                        AUTO_STATE = COMPLETED;
                        Monitor.print("[DEBUG] Transitioning to COMPLETED: lidarAt0Deg.distance_mm=");
                        Monitor.print(lidarAt0Deg.distance_mm);
                        Monitor.print(" lidarAt270Deg.distance_mm=");
                        Monitor.println(lidarAt270Deg.distance_mm);
                    }
                    break;

                case COMPLETED:
                    mecanumDriver.stop();
                    break;
            }

            break;
        case ControlState::CONNECTION_LOST:
            // We should never reach this case due to the hasDriveSignal check above, but we include it for completeness.
            mecanumDriver.stop();
            // resetSpeedControllers(frontLeftSpeedController, frontRightSpeedController, rearLeftSpeedController, rearRightSpeedController);
            return;
    }

    const MecanumDriver::WheelCommands targetWheelCommands = MecanumDriver::mix(lateralCommand, longitudinalCommand, rotationCommand);

    // const float frontLeftSetpoint = wheelCommandToSpeedSetpoint(targetWheelCommands.frontLeft);
    // const float frontRightSetpoint = wheelCommandToSpeedSetpoint(targetWheelCommands.frontRight);
    // const float rearLeftSetpoint = wheelCommandToSpeedSetpoint(targetWheelCommands.rearLeft);
    // const float rearRightSetpoint = wheelCommandToSpeedSetpoint(targetWheelCommands.rearRight);

    // const EncoderSpeedSnapshot measuredSpeeds = encoders.getCurrentSpeed();

    // const float dtSeconds = (elapsedMs > 0U) ? (static_cast<float>(elapsedMs) / 1000.0f) : kDefaultDtSeconds;

    // const float frontLeftCommand = frontLeftSpeedController.update(frontLeftSetpoint, signedMeasuredSpeed(measuredSpeeds.frontLeft, frontLeftSetpoint), dtSeconds);
    // const float frontRightCommand = frontRightSpeedController.update(frontRightSetpoint, signedMeasuredSpeed(measuredSpeeds.frontRight, frontRightSetpoint), dtSeconds);
    // const float rearLeftCommand = rearLeftSpeedController.update(rearLeftSetpoint, signedMeasuredSpeed(measuredSpeeds.rearLeft, rearLeftSetpoint), dtSeconds);
    // const float rearRightCommand = rearRightSpeedController.update(rearRightSetpoint, signedMeasuredSpeed(measuredSpeeds.rearRight, rearRightSetpoint), dtSeconds);

    float frontLeftCommand = static_cast<float>(targetWheelCommands.frontLeft);
    float frontRightCommand = static_cast<float>(targetWheelCommands.frontRight);
    float rearLeftCommand = static_cast<float>(targetWheelCommands.rearLeft);
    float rearRightCommand = static_cast<float>(targetWheelCommands.rearRight);

    constexpr float speedScalingFactor = 0.65f;
    constexpr float speedOffset = 0.0f;
    frontLeftCommand = frontLeftCommand * speedScalingFactor + speedOffset;
    frontRightCommand = frontRightCommand * speedScalingFactor + speedOffset;
    rearLeftCommand = rearLeftCommand * speedScalingFactor + speedOffset;
    rearRightCommand = rearRightCommand * speedScalingFactor + speedOffset;

    mecanumDriver.driveWheels(static_cast<int16_t>(frontLeftCommand), static_cast<int16_t>(frontRightCommand), static_cast<int16_t>(rearLeftCommand), static_cast<int16_t>(rearRightCommand));

    // ===== DEBUG PRINT ZONE =====
    if (elapsedMs < kControlPeriodMs) return;
    lastControlUpdateMs = nowMs;
    // debug_print::printDetailedLidarDistances(LiDAR, lidarAt0Deg, lidarAt90Deg, lidarAt270Deg);
    // debug_print::printLidarDistances(lidarAt0Deg, lidarAt90Deg, lidarAt270Deg);
}