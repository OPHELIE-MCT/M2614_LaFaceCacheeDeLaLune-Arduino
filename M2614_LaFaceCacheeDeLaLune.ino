#include <Arduino_RouterBridge.h>
#include <zephyr/kernel.h>

#include "PINS.h"
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

namespace {
constexpr uint32_t kControlPeriodMs = 50;
constexpr uint32_t kRcDebugPeriodMs = 250;
constexpr float kDefaultDtSeconds = 0.05f;
constexpr float kMaxSpeedPulsesPerPeriod = 80.0f;
constexpr int16_t kJoystickDeadzone = 30;

uint32_t lastControlUpdateMs = 0;
uint32_t lastRcDebugPrintMs = 0;
uint32_t lastRcUpdateUs = 0;
uint32_t rcUpdateIntervalMinUs = 0xFFFFFFFFUL;
uint32_t rcUpdateIntervalMaxUs = 0;
uint32_t rcUpdateIntervalSumUs = 0;
uint32_t rcUpdateIntervalCount = 0;

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

const char* rcChannelName(RCChannel channel) {
    switch (channel) {
        case RCChannel::A:
            return "A";
        case RCChannel::B:
            return "B";
        case RCChannel::C:
            return "C";
        case RCChannel::D:
            return "D";
        case RCChannel::E:
            return "E";
        case RCChannel::F:
            return "F";
        case RCChannel::G:
            return "G";
        case RCChannel::H:
            return "H";
        case RCChannel::Count:
        default:
            return "?";
    }
}

void printRcDebugChannel(RCChannel channel) {
    const RCDebugSnapshot debug = rc.getDebugSnapshot(channel);

    Monitor.println(
        String("  ") + rcChannelName(channel) +
        " pin=" + String(debug.pin) +
        " lvl=" + String(debug.sampledHigh ? 1 : 0) +
        " valid=" + String(debug.signalValid ? 1 : 0) +
        " rawUs=" + String(debug.rawPulseWidthUs) +
        " pulseUs=" + String(debug.clampedPulseWidthUs) +
        " riseAgeUs=" + String(debug.ageSinceLastRiseUs) +
        " pulseAgeUs=" + String(debug.ageSinceLastPulseUs) +
        " edges=" + String(debug.risingEdgeCount) + "/" + String(debug.fallingEdgeCount));
}

void printRcDebugSummary(uint32_t nowMs, uint32_t elapsedMs) {
    if ((nowMs - lastRcDebugPrintMs) < kRcDebugPeriodMs) {
        return;
    }

    lastRcDebugPrintMs = nowMs;
    Monitor.println(String("[RC DEBUG] invalid drive signal dtMs=") + String(elapsedMs));
    printRcDebugChannel(RCChannel::A);
    printRcDebugChannel(RCChannel::B);
    printRcDebugChannel(RCChannel::C);
    printRcDebugChannel(RCChannel::D);
    printRcDebugChannel(RCChannel::E);
    printRcDebugChannel(RCChannel::F);
    printRcDebugChannel(RCChannel::G);
    printRcDebugChannel(RCChannel::H);
}

void noteRcUpdateCadence(uint32_t nowUs) {
    if (lastRcUpdateUs != 0U) {
        const uint32_t dtUs = nowUs - lastRcUpdateUs;
        if (dtUs < rcUpdateIntervalMinUs) {
            rcUpdateIntervalMinUs = dtUs;
        }
        if (dtUs > rcUpdateIntervalMaxUs) {
            rcUpdateIntervalMaxUs = dtUs;
        }
        rcUpdateIntervalSumUs += dtUs;
        ++rcUpdateIntervalCount;
    }
    lastRcUpdateUs = nowUs;
}

void printRcUpdateCadence() {
    const uint32_t minUs = (rcUpdateIntervalCount > 0U) ? rcUpdateIntervalMinUs : 0U;
    const uint32_t maxUs = (rcUpdateIntervalCount > 0U) ? rcUpdateIntervalMaxUs : 0U;
    const uint32_t avgUs = (rcUpdateIntervalCount > 0U) ? (rcUpdateIntervalSumUs / rcUpdateIntervalCount) : 0U;

    Monitor.println(
        String("[DEBUG] RC cadence avg=") + String(avgUs) + "us" +
        " min=" + String(minUs) + "us" +
        " max=" + String(maxUs) + "us" +
        " samples=" + String(rcUpdateIntervalCount));

    rcUpdateIntervalMinUs = 0xFFFFFFFFUL;
    rcUpdateIntervalMaxUs = 0;
    rcUpdateIntervalSumUs = 0;
    rcUpdateIntervalCount = 0;
}
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

void testMotors() {
    Monitor.println("Testing front left motor...");
    analogWrite(FL_EN, 128);
    digitalWrite(FL_IN1, HIGH);
    digitalWrite(FL_IN2, LOW);
    delay(2000);
    digitalWrite(FL_IN1, LOW);
    digitalWrite(FL_IN2, HIGH);
    delay(2000);
    digitalWrite(FL_IN1, LOW);
    digitalWrite(FL_IN2, LOW);
    analogWrite(FL_EN, 0);

    Monitor.println("Testing front right motor...");
    analogWrite(FR_EN, 128);
    digitalWrite(FR_IN1, HIGH);
    digitalWrite(FR_IN2, LOW);
    delay(2000);
    digitalWrite(FR_IN1, LOW);
    digitalWrite(FR_IN2, HIGH);
    delay(2000);
    digitalWrite(FR_IN1, LOW);
    digitalWrite(FR_IN2, LOW);
    analogWrite(FR_EN, 0);

    Monitor.println("Testing rear left motor...");
    analogWrite(BL_EN, 128);
    digitalWrite(BL_IN1, HIGH);
    digitalWrite(BL_IN2, LOW);
    delay(2000);
    digitalWrite(BL_IN1, LOW);
    digitalWrite(BL_IN2, HIGH);
    delay(2000);
    digitalWrite(BL_IN1, LOW);
    digitalWrite(BL_IN2, LOW);
    analogWrite(BL_EN, 0);

    Monitor.println("Testing rear right motor...");
    analogWrite(BR_EN, 128);
    digitalWrite(BR_IN1, HIGH);
    digitalWrite(BR_IN2, LOW);
    delay(2000);
    digitalWrite(BR_IN1, LOW);
    digitalWrite(BR_IN2, HIGH);
    delay(2000);
    digitalWrite(BR_IN1, LOW);
    digitalWrite(BR_IN2, LOW);
    analogWrite(BR_EN, 0);

    Monitor.println("Motor test sequence complete.");
}

void testEnMapping() {
    digitalWrite(FL_IN1, HIGH);
    digitalWrite(FL_IN2, LOW);
    digitalWrite(FR_IN1, HIGH);
    digitalWrite(FR_IN2, LOW);
    digitalWrite(BL_IN1, HIGH);
    digitalWrite(BL_IN2, LOW);
    digitalWrite(BR_IN1, HIGH);
    digitalWrite(BR_IN2, LOW);

    constexpr uint8_t speed = 35;
    Monitor.println("Front right");
    analogWrite(FR_EN, speed);
    delay(2000);
    analogWrite(FR_EN, 0);
    Monitor.println("Front left");
    analogWrite(FL_EN, speed);
    delay(2000);
    analogWrite(FL_EN, 0);
    Monitor.println("Back right");
    analogWrite(BR_EN, speed);
    delay(2000);
    analogWrite(BR_EN, 0);
    Monitor.println("Back left");
    analogWrite(BL_EN, speed);
    delay(2000);
    analogWrite(BL_EN, 0);

    Monitor.println("EN mapping test complete.");
}

void testInDirections() {
    analogWrite(FL_EN, 35);
    analogWrite(FR_EN, 35);
    analogWrite(BL_EN, 35);
    analogWrite(BR_EN, 35);

    Monitor.println("Testing forward direction...");
    digitalWrite(FL_IN1, HIGH);
    digitalWrite(FL_IN2, LOW);
    digitalWrite(FR_IN1, HIGH);
    digitalWrite(FR_IN2, LOW);
    digitalWrite(BL_IN1, HIGH);
    digitalWrite(BL_IN2, LOW);
    digitalWrite(BR_IN1, HIGH);
    digitalWrite(BR_IN2, LOW);
    delay(5000);

    Monitor.println("Testing reverse direction...");
    digitalWrite(FL_IN1, LOW);
    digitalWrite(FL_IN2, HIGH);
    digitalWrite(FR_IN1, LOW);
    digitalWrite(FR_IN2, HIGH);
    digitalWrite(BL_IN1, LOW);
    digitalWrite(BL_IN2, HIGH);
    digitalWrite(BR_IN1, LOW);
    digitalWrite(BR_IN2, HIGH);
    delay(5000);

    Monitor.println("Testing stop...");
    digitalWrite(FL_IN1, LOW);
    digitalWrite(FL_IN2, LOW);
    digitalWrite(FR_IN1, LOW);
    digitalWrite(FR_IN2, LOW);
    digitalWrite(BL_IN1, LOW);
    digitalWrite(BL_IN2, LOW);
    digitalWrite(BR_IN1, LOW);
    digitalWrite(BR_IN2, LOW);
    delay(1500);

    Monitor.println("Direction test sequence complete.");
}

void setup() {
    Monitor.begin();
    Monitor.println("===============================");
    Monitor.println("Starting up...");
    pinSetup();
    mecanumDriver.begin();
    rc.begin();
    encoders.begin();
    lastControlUpdateMs = millis();
    Monitor.println("Setup complete.");
}

void loop() {
    // return;  // IGNORE - loop body is currently empty to disable robot control while testing other components
    const uint32_t rcUpdateNowUs = micros();
    noteRcUpdateCadence(rcUpdateNowUs);
    rc.update();

    const uint32_t nowMs = millis();
    const uint32_t elapsedMs = nowMs - lastControlUpdateMs;
    if (elapsedMs < kControlPeriodMs) return;

    lastControlUpdateMs = nowMs;

    const bool hasDriveSignal = rc.isSignalValid(RCChannel::A) && rc.isSignalValid(RCChannel::B) && rc.isSignalValid(RCChannel::D);
    if (!hasDriveSignal) {
        // printRcDebugSummary(nowMs, elapsedMs);
        mecanumDriver.stop();
        // resetSpeedControllers(frontLeftSpeedController, frontRightSpeedController, rearLeftSpeedController, rearRightSpeedController);
        return;
    }

    Monitor.println(
        String("[DEBUG] RC A=") + String(rc.getJoystick(RCChannel::A)) + " (" + String(rc.getPulseWidthUs(RCChannel::A)) + "us)" +
        " B=" + String(rc.getJoystick(RCChannel::B)) + " (" + String(rc.getPulseWidthUs(RCChannel::B)) + "us)" +
        " D=" + String(rc.getJoystick(RCChannel::D)) + " (" + String(rc.getPulseWidthUs(RCChannel::D)) + "us)");
    printRcUpdateCadence();
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

    const float frontLeftCommand = frontLeftSetpoint;
    const float frontRightCommand = frontRightSetpoint;
    const float rearLeftCommand = rearLeftSetpoint;
    const float rearRightCommand = rearRightSetpoint;

    mecanumDriver.driveWheels(static_cast<int16_t>(frontLeftCommand), static_cast<int16_t>(frontRightCommand), static_cast<int16_t>(rearLeftCommand), static_cast<int16_t>(rearRightCommand));
}