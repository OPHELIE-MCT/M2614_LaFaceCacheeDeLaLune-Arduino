#include <Arduino_RouterBridge.h>

#include "LiDAR/LiDARSensor.h"
#include "PINS.h"
#include "debug/debug_print.h"
#include "driver/MecanumDriver.h"
#include "driver/RemoteController.h"
#include "driver/ToFDistanceSensor.h"

MotorPinConfig frontLeftPins = {FL_IN1, FL_IN2, FL_EN};
MotorPinConfig frontRightPins = {FR_IN1, FR_IN2, FR_EN};
MotorPinConfig rearLeftPins = {BL_IN1, BL_IN2, BL_EN};
MotorPinConfig rearRightPins = {BR_IN1, BR_IN2, BR_EN};
MecanumPins mecanumPins = {frontLeftPins, frontRightPins, rearLeftPins, rearRightPins};
MecanumDriver mecanumDriver = MecanumDriver(mecanumPins);

RCReceiverPins rcPins = {RC_PIN_A, RC_PIN_B, RC_PIN_C, RC_PIN_D, RC_PIN_E, RC_PIN_F, RC_PIN_G, RC_PIN_H};
RemoteController rc = RemoteController(rcPins);

LiDARSensor LiDAR;
ToFDistanceSensor tofDistanceSensor;

enum ControlState {
    MANUAL_CONTROL,
    AUTOMATIC_CONTROL,
    CONNECTION_LOST
};

enum AutoControlState {
    SLOW_FORWARD,
    STOP,
    TURN,
    REPOSITION,
    SLOW_FORWARD_2,
    EXIT_TURN,
    EXIT_FORWARD
};

ControlState CURRENT_STATE = ControlState::MANUAL_CONTROL;
AutoControlState AUTO_STATE = AutoControlState::SLOW_FORWARD;

uint8_t stopCounter = 0;
constexpr uint32_t kDebugPrintDelayMs = 200;
constexpr uint32_t kRcSignalLossDebounceMs = 250;
constexpr int16_t kJoystickDeadzone = 30;
constexpr uint16_t kButtonPressThresholdUs = 1850;
constexpr float kToFStopThresholdMm = ToFDistanceSensor::kDefaultThresholdMm;

uint32_t lastControlUpdateMs = 0;
uint32_t driveSignalInvalidSinceMs = 0;

int16_t applyDeadzone(int16_t value, int16_t deadzone) {
    return (value > -deadzone && value < deadzone) ? 0 : value;
}

void setup() {
    Monitor.begin();
    Monitor.println("===============================");
    Monitor.println("Starting up...");
    pinSetup();
    const bool tofStarted = tofDistanceSensor.begin(Wire1, kToFStopThresholdMm);
    Serial1.begin(LDS_LDROBOT_LD19::SERIAL_BAUD_RATE);
    LiDAR.begin(Serial1);
    mecanumDriver.begin();
    rc.begin();
    lastControlUpdateMs = millis();
    Monitor.println(tofStarted ? "ToF sensor ready on Wire1." : "ToF sensor init failed on Wire1.");
    Monitor.println("Setup complete.");
}

void loop() {
    rc.update();
    LiDAR.update();
    const ToFDistanceMeasurement tofMeasurement = tofDistanceSensor.update();

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

    lidarAt0Deg = LiDAR.queryAngleAt(0);
    lidarAt90Deg = LiDAR.queryAngleAt(90);
    lidarAt270Deg = LiDAR.queryAngleAt(270);

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

            if (isAutoSwitchPressed && !wasAutoSwitchPressed) {
                CURRENT_STATE = ControlState::AUTOMATIC_CONTROL;
                AUTO_STATE = AutoControlState::SLOW_FORWARD;
                Monitor.println("=============== SWITCHING TO AUTOMATIC CONTROL MODE ===============");
            }
            wasAutoSwitchPressed = isAutoSwitchPressed;
            break;
        }
        case ControlState::AUTOMATIC_CONTROL:

            switch (AUTO_STATE) {
                case AutoControlState::SLOW_FORWARD:
                    longitudinalCommand = 100;
                    lateralCommand = 25;
                    if (lidarAt0Deg.distance_mm < 25) {
                        AUTO_STATE = AutoControlState::TURN;
                        stopCounter = 0;
                        Monitor.println("[DEBUG] Transitioning to TURN: Obstacle detected at 0° within 25mm");
                    }
                    break;
                case AutoControlState::TURN:
                    longitudinalCommand = 0;
                    lateralCommand = 0;
                    rotationCommand = -150;
                    if (lidarAt0Deg.distance_mm > 430) {
                        AUTO_STATE = AutoControlState::REPOSITION;
                        Monitor.println("[DEBUG] Transitioning to REPOSITION: Path ahead at 0° is clear beyond 430mm");
                    }
                    break;
                case AutoControlState::REPOSITION:
                    longitudinalCommand = 0;
                    lateralCommand = 225;
                    rotationCommand = 0;
                    // Do that for 3 seconds, then stop
                    static uint32_t repositionStartMs = 0;
                    if (repositionStartMs == 0U) {
                        repositionStartMs = nowMs;
                    }
                    if ((nowMs - repositionStartMs) >= 3000) {
                        AUTO_STATE = AutoControlState::SLOW_FORWARD_2;
                        repositionStartMs = 0U;
                        Monitor.println("[DEBUG] Transitioning to SLOW_FORWARD_2: Completed repositioning maneuver");
                    }
                    break;
                case AutoControlState::SLOW_FORWARD_2:
                    longitudinalCommand = 100;
                    lateralCommand = 2;
                    rotationCommand = 0;
                    if (lidarAt0Deg.distance_mm < 25) {
                        AUTO_STATE = AutoControlState::EXIT_TURN;
                        stopCounter = 0;
                        Monitor.println("[DEBUG] Transitioning to EXIT_TURN: Obstacle detected at 0° within 25mm");
                    }
                    break;
                case AutoControlState::EXIT_TURN:
                    longitudinalCommand = 0;
                    lateralCommand = 0;
                    rotationCommand = -150;
                    if (lidarAt0Deg.distance_mm > 2000) {
                        AUTO_STATE = AutoControlState::EXIT_FORWARD;
                        Monitor.println("[DEBUG] Transitioning to EXIT_FORWARD: Path ahead at 0° is clear beyond 2000mm");
                    }
                    break;
                case AutoControlState::EXIT_FORWARD:
                    longitudinalCommand = 250;
                    lateralCommand = 0;
                    rotationCommand = 0;
                    if (lidarAt0Deg.distance_mm < 350) {
                        AUTO_STATE = AutoControlState::STOP;
                        Monitor.println("[DEBUG] Transitioning to STOP: Obstacle detected at 0° within 250mm");
                    }
                    break;
                case AutoControlState::STOP:
                    mecanumDriver.stop();
                    CURRENT_STATE = ControlState::MANUAL_CONTROL;
                    Monitor.println("=============== SWITCHING TO MANUAL CONTROL MODE ===============");
                    break;
            }
            break;
        case ControlState::CONNECTION_LOST:
            // We should never reach this case due to the hasDriveSignal check above, but we include it for completeness.
            mecanumDriver.stop();
            return;
    }

    const MecanumDriver::WheelCommands targetWheelCommands = MecanumDriver::mix(lateralCommand, longitudinalCommand, rotationCommand);

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
    if (elapsedMs < kDebugPrintDelayMs) return;
    lastControlUpdateMs = nowMs;
    // debug_print::printDetailedLidarDistances(LiDAR, lidarAt0Deg, lidarAt90Deg, lidarAt270Deg);
    debug_print::printLidarDistances(lidarAt0Deg, lidarAt90Deg, lidarAt270Deg);
    debug_print::printTofMeasurement(tofMeasurement, tofDistanceSensor.getLastStatusCode(), tofDistanceSensor.getThresholdMm());
}