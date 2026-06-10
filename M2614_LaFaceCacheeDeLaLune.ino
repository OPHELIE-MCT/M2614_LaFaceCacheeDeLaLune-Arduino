#include <Arduino_RouterBridge.h>

#include "LiDAR/LiDARSensor.h"
#include "PINS.h"
#include "debug/debug_print.h"
#include "driver/FeedbackEncoder.h"
#include "driver/MecanumDriver.h"
#include "driver/RemoteController.h"
#include "driver/SpeedController.h"
#include "driver/ToFDistanceSensor.h"

MotorPinConfig frontLeftPins = {FL_IN1, FL_IN2, FL_EN};
MotorPinConfig frontRightPins = {FR_IN1, FR_IN2, FR_EN};
MotorPinConfig rearLeftPins = {BL_IN1, BL_IN2, BL_EN};
MotorPinConfig rearRightPins = {BR_IN1, BR_IN2, BR_EN};
MecanumPins mecanumPins = {frontLeftPins, frontRightPins, rearLeftPins, rearRightPins};
MecanumDriver mecanumDriver = MecanumDriver(mecanumPins);

FeedbackEncoderPins encoderPins = {
    {FL_ENC_A, FL_ENC_B},
    {FR_ENC_A, FR_ENC_B},
    {BL_ENC_A, BL_ENC_B},
    {BR_ENC_A, BR_ENC_B},
};
FeedbackEncoder feedbackEncoder = FeedbackEncoder(encoderPins);

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
    TOF_PAUSE_STOP,
    TURN,
    REPOSITION,
    SLOW_FORWARD_2,
    EXIT_TURN,
    EXIT_FORWARD
};

ControlState CURRENT_STATE = ControlState::MANUAL_CONTROL;
AutoControlState AUTO_STATE = AutoControlState::SLOW_FORWARD;

struct TofPauseTracker {
    uint8_t completedStops;
    uint8_t maxStops;
    bool pauseArmed;
    bool stopPending;
    uint32_t pauseStartMs;
    uint32_t stopPendingSinceMs;
    AutoControlState resumeState;
    const char* segmentLabel;
};

constexpr uint32_t kDebugPrintDelayMs = 50;
constexpr uint32_t kRcSignalLossDebounceMs = 250;
constexpr uint32_t kTofPauseDurationMs = 2000;
constexpr uint32_t kTofRisingEdgeStopDelayMs = 120;
constexpr int16_t kJoystickDeadzone = 30;
constexpr uint16_t kButtonPressThresholdUs = 1850;
constexpr int16_t kTofSlowLongitudinalCommand = 85;
constexpr int16_t kAutoForwardLongitudinalCommand = 95;
constexpr float kToFStopThresholdMm = ToFDistanceSensor::kDefaultThresholdMm;

uint32_t lastControlUpdateMs = 0;
uint32_t driveSignalInvalidSinceMs = 0;
TofPauseTracker tofPauseTracker = {0U, 0U, false, false, 0U, 0U, AutoControlState::SLOW_FORWARD, nullptr};

// ===== PID SPEED CONTROLLER =====
// Gains and clamps are shared across all four wheels. Tune kp/ki/kd on the robot;
// the placeholder values below will compile but may need adjustment.
// outputMin is 0 because the encoder has no direction sensing (channel B only).
constexpr SpeedControllerConfig kWheelPidConfig = {
    /* kp          */ 2.0f,
    /* ki          */ 0.0f,
    /* kd          */ 0.0f,
    /* integralMin */ -200.0f,
    /* integralMax */ 200.0f,
    /* outputMin   */ -120.0f,
    /* outputMax   */ 120.0f,
};

// Open-loop forward bias used by the PID controllers.
// The PID output is a correction added on top of this baseline instead of the
// final wheel command itself.
constexpr int16_t kWheelPidBaseCommand = 50;

PIDSpeedController pidFL(kWheelPidConfig);
PIDSpeedController pidFR(kWheelPidConfig);
PIDSpeedController pidRL(kWheelPidConfig);
PIDSpeedController pidRR(kWheelPidConfig);

// Per-segment wheel speed targets (pulses / 50 ms).
// SLOW_FORWARD: FL/RR slightly higher to replicate the old lateralCommand=25 drift.
// Adjust these values based on observed encoder readings at the desired speed.
constexpr WheelSpeedTargets kSlowForwardTargets = {20.0f, 12.0f, 12.0f, 20.0f};

// Last PID wheel command, updated every 50 ms and held between updates.
MecanumDriver::WheelCommands pidWheelCommands = {0, 0, 0, 0};

// True when the active AUTO_STATE uses per-wheel PID; skips mix() in the drive path.
bool inPidForwardMode = false;

// Tracks the previous AUTO_STATE to detect state entries (used to reset PIDs).
AutoControlState prevAutoState = AutoControlState::SLOW_FORWARD;

int16_t applyDeadzone(int16_t value, int16_t deadzone) {
    return (value > -deadzone && value < deadzone) ? 0 : value;
}

const char* autoStateName(AutoControlState state) {
    switch (state) {
        case AutoControlState::SLOW_FORWARD:
            return "SLOW_FORWARD";
        case AutoControlState::STOP:
            return "STOP";
        case AutoControlState::TOF_PAUSE_STOP:
            return "TOF_PAUSE_STOP";
        case AutoControlState::TURN:
            return "TURN";
        case AutoControlState::REPOSITION:
            return "REPOSITION";
        case AutoControlState::SLOW_FORWARD_2:
            return "SLOW_FORWARD_2";
        case AutoControlState::EXIT_TURN:
            return "EXIT_TURN";
        case AutoControlState::EXIT_FORWARD:
            return "EXIT_FORWARD";
        default:
            return "UNKNOWN";
    }
}

void configureTofPauseSegment(uint8_t maxStops, const char* segmentLabel, AutoControlState resumeState) {
    tofPauseTracker.completedStops = 0U;
    tofPauseTracker.maxStops = maxStops;
    tofPauseTracker.pauseArmed = false;
    tofPauseTracker.stopPending = false;
    tofPauseTracker.pauseStartMs = 0U;
    tofPauseTracker.stopPendingSinceMs = 0U;
    tofPauseTracker.resumeState = resumeState;
    tofPauseTracker.segmentLabel = segmentLabel;
}

void clearTofPauseTracking() {
    tofPauseTracker.pauseArmed = false;
    tofPauseTracker.stopPending = false;
    tofPauseTracker.pauseStartMs = 0U;
    tofPauseTracker.stopPendingSinceMs = 0U;
    tofPauseTracker.maxStops = 0U;
    tofPauseTracker.segmentLabel = nullptr;
}

void resetTofPauseTrackingStats() {
    tofPauseTracker.completedStops = 0U;
    tofPauseTracker.maxStops = 0U;
    tofPauseTracker.pauseArmed = false;
    tofPauseTracker.stopPending = false;
    tofPauseTracker.pauseStartMs = 0U;
    tofPauseTracker.stopPendingSinceMs = 0U;
    tofPauseTracker.resumeState = AutoControlState::SLOW_FORWARD;
    tofPauseTracker.segmentLabel = nullptr;
}

bool hasRemainingTofPauseBudget() {
    return tofPauseTracker.completedStops < tofPauseTracker.maxStops;
}

void enterTofPauseStop(AutoControlState resumeState, uint32_t nowMs) {
    tofPauseTracker.resumeState = resumeState;
    tofPauseTracker.pauseStartMs = nowMs;
    tofPauseTracker.pauseArmed = false;
    tofPauseTracker.stopPending = false;
    tofPauseTracker.stopPendingSinceMs = 0U;
    if (tofPauseTracker.completedStops < UINT8_MAX) {
        ++tofPauseTracker.completedStops;
    }
    AUTO_STATE = AutoControlState::TOF_PAUSE_STOP;
    Monitor.println(
        String("[DEBUG] Entering TOF_PAUSE_STOP in ") +
        String(tofPauseTracker.segmentLabel != nullptr ? tofPauseTracker.segmentLabel : "UNKNOWN") +
        " | stop " + String(tofPauseTracker.completedStops) + "/" + String(tofPauseTracker.maxStops));
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
    feedbackEncoder.begin();
    rc.begin();
    lastControlUpdateMs = millis();
    Monitor.println(tofStarted ? "ToF sensor ready on Wire1." : "ToF sensor init failed on Wire1.");
    tofDistanceSensor.update();
    Monitor.println("Setup complete.");
}

void loop() {
    inPidForwardMode = false;
    rc.update();
    LiDAR.update();
    // tofDistanceSensor.update();
    const ToFDistanceMeasurement tofMeasurement = tofDistanceSensor.getLastMeasurement();

    const uint32_t nowMs = millis();
    const uint32_t elapsedMs = nowMs - lastControlUpdateMs;

    constexpr LiDARSensor::QueryResult defaultQueryResult = {false, 0U, 0U, -1, -1, 0U, 0U, 0U};
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
            resetTofPauseTrackingStats();
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
                configureTofPauseSegment(3U, "SLOW_FORWARD", AutoControlState::SLOW_FORWARD);
                Monitor.println("=============== SWITCHING TO AUTOMATIC CONTROL MODE ===============");
            }
            wasAutoSwitchPressed = isAutoSwitchPressed;
            break;
        }
        case ControlState::AUTOMATIC_CONTROL:

            switch (AUTO_STATE) {
                case AutoControlState::SLOW_FORWARD:
                    if (prevAutoState != AUTO_STATE) {
                        pidFL.reset();
                        pidFR.reset();
                        pidRL.reset();
                        pidRR.reset();
                        pidWheelCommands = {0, 0, 0, 0};
                    }
                    inPidForwardMode = true;

                    if (hasRemainingTofPauseBudget()) {
                        if (tofMeasurement.isValid && !tofMeasurement.isThresholdReached) {
                            tofPauseTracker.pauseArmed = true;
                            tofPauseTracker.stopPending = false;
                            tofPauseTracker.stopPendingSinceMs = 0U;
                        } else if (tofMeasurement.isValid && tofMeasurement.isThresholdReached && tofPauseTracker.pauseArmed) {
                            if (!tofPauseTracker.stopPending) {
                                tofPauseTracker.stopPending = true;
                                tofPauseTracker.stopPendingSinceMs = nowMs;
                            }

                            if ((nowMs - tofPauseTracker.stopPendingSinceMs) >= kTofRisingEdgeStopDelayMs) {
                                pidWheelCommands = {0, 0, 0, 0};
                                enterTofPauseStop(AutoControlState::SLOW_FORWARD, nowMs);
                            }
                        }
                    }

                    if (lidarAt0Deg.distance_mm < 25) {
                        clearTofPauseTracking();
                        AUTO_STATE = AutoControlState::TURN;
                        Monitor.println("[DEBUG] Transitioning to TURN: Obstacle detected at 0° at " + String(lidarAt0Deg.distance_mm) + "/25 mm");
                    }
                    break;
                case AutoControlState::TURN:
                    longitudinalCommand = 0;
                    lateralCommand = 0;
                    rotationCommand = -150;
                    if (lidarAt0Deg.distance_mm > 430) {
                        AUTO_STATE = AutoControlState::REPOSITION;
                        Monitor.println("[DEBUG] Transitioning to REPOSITION: Path ahead at 0° is " + String(lidarAt0Deg.distance_mm) + "mm");
                    }
                    break;
                case AutoControlState::REPOSITION:
                    longitudinalCommand = 50;
                    lateralCommand = 225;
                    rotationCommand = 0;
                    // Do that for 3 seconds, then stop
                    static uint32_t repositionStartMs = 0;
                    if (repositionStartMs == 0U) {
                        repositionStartMs = nowMs;
                    }
                    if ((nowMs - repositionStartMs) >= 3000) {
                        AUTO_STATE = AutoControlState::SLOW_FORWARD_2;
                        configureTofPauseSegment(1U, "SLOW_FORWARD_2", AutoControlState::SLOW_FORWARD_2);
                        repositionStartMs = 0U;
                        Monitor.println("[DEBUG] Transitioning to SLOW_FORWARD_2: Completed repositioning maneuver");
                    }
                    break;
                case AutoControlState::SLOW_FORWARD_2:
                    if (prevAutoState != AUTO_STATE) {
                        pidFL.reset();
                        pidFR.reset();
                        pidRL.reset();
                        pidRR.reset();
                        pidWheelCommands = {0, 0, 0, 0};
                    }
                    inPidForwardMode = true;

                    if (hasRemainingTofPauseBudget()) {
                        if (tofMeasurement.isValid && !tofMeasurement.isThresholdReached) {
                            tofPauseTracker.pauseArmed = true;
                            tofPauseTracker.stopPending = false;
                            tofPauseTracker.stopPendingSinceMs = 0U;
                        } else if (tofMeasurement.isValid && tofMeasurement.isThresholdReached && tofPauseTracker.pauseArmed) {
                            if (!tofPauseTracker.stopPending) {
                                tofPauseTracker.stopPending = true;
                                tofPauseTracker.stopPendingSinceMs = nowMs;
                            }

                            if ((nowMs - tofPauseTracker.stopPendingSinceMs) >= kTofRisingEdgeStopDelayMs) {
                                pidWheelCommands = {0, 0, 0, 0};
                                enterTofPauseStop(AutoControlState::SLOW_FORWARD_2, nowMs);
                            }
                        }
                    }

                    if (lidarAt0Deg.distance_mm < 25) {
                        clearTofPauseTracking();
                        AUTO_STATE = AutoControlState::EXIT_TURN;
                        Monitor.println("[DEBUG] Transitioning to EXIT_TURN: Obstacle detected at 0° at " + String(lidarAt0Deg.distance_mm) + "/25 mm");
                    }
                    break;
                case AutoControlState::TOF_PAUSE_STOP:
                    longitudinalCommand = 0;
                    lateralCommand = 0;
                    rotationCommand = 0;
                    if ((nowMs - tofPauseTracker.pauseStartMs) >= kTofPauseDurationMs) {
                        AUTO_STATE = tofPauseTracker.resumeState;
                        tofPauseTracker.pauseStartMs = 0U;
                        Monitor.println(
                            String("[DEBUG] Leaving TOF_PAUSE_STOP, resuming ") +
                            String(autoStateName(AUTO_STATE)) +
                            " | completed stops=" + String(tofPauseTracker.completedStops) + "/" + String(tofPauseTracker.maxStops));
                    }
                    break;
                case AutoControlState::EXIT_TURN:
                    longitudinalCommand = 0;
                    lateralCommand = 0;
                    rotationCommand = -150;
                    if (lidarAt0Deg.distance_mm > 2000) {
                        AUTO_STATE = AutoControlState::EXIT_FORWARD;
                        Monitor.println("[DEBUG] Transitioning to EXIT_FORWARD: Path ahead at 0° is clear beyond " + String(lidarAt0Deg.distance_mm) + "mm");
                    }
                    break;
                case AutoControlState::EXIT_FORWARD:
                    longitudinalCommand = 250;
                    lateralCommand = 0;
                    rotationCommand = 0;
                    if (lidarAt0Deg.distance_mm < 350) {
                        clearTofPauseTracking();
                        AUTO_STATE = AutoControlState::STOP;
                        Monitor.println("[DEBUG] Transitioning to STOP: Obstacle detected at 0° at " + String(lidarAt0Deg.distance_mm) + "/350 mm");
                    }
                    break;
                case AutoControlState::STOP:
                    clearTofPauseTracking();
                    mecanumDriver.stop();
                    CURRENT_STATE = ControlState::MANUAL_CONTROL;
                    resetTofPauseTrackingStats();
                    Monitor.println("=============== SWITCHING TO MANUAL CONTROL MODE ===============");
                    break;
            }
            prevAutoState = AUTO_STATE;
            break;
        case ControlState::CONNECTION_LOST:
            // We should never reach this case due to the hasDriveSignal check above, but we include it for completeness.
            mecanumDriver.stop();
            return;
    }

    if (inPidForwardMode) {
        // PID mode: drive wheels directly from last PID output — no mix(), no scaling.
        mecanumDriver.driveWheels(
            pidWheelCommands.frontLeft,
            pidWheelCommands.frontRight,
            pidWheelCommands.rearLeft,
            pidWheelCommands.rearRight);
    } else {
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
    }

    // ===== DEBUG PRINT ZONE =====
    if (elapsedMs < kDebugPrintDelayMs) return;
    lastControlUpdateMs = nowMs;
    tofDistanceSensor.update();
    EncoderSpeedSnapshot feedbackEncoderSnapshot = feedbackEncoder.getCurrentSpeed();
    // debug_print::printEncoderPulsesPer50Ms(feedbackEncoderSnapshot);

    // ===== PID UPDATE (every 50 ms, synchronized with encoder sample window) =====
    if (inPidForwardMode) {
        const WheelSpeedTargets& target = kSlowForwardTargets;
        constexpr float kDtSeconds = static_cast<float>(FeedbackEncoder::kSamplePeriodMs) / 1000.0f;
        const int16_t frontLeftCorrection = static_cast<int16_t>(pidFL.update(target.frontLeft, static_cast<float>(feedbackEncoderSnapshot.frontLeft), kDtSeconds));
        const int16_t frontRightCorrection = static_cast<int16_t>(pidFR.update(target.frontRight, static_cast<float>(feedbackEncoderSnapshot.frontRight), kDtSeconds));
        const int16_t rearLeftCorrection = static_cast<int16_t>(pidRL.update(target.rearLeft, static_cast<float>(feedbackEncoderSnapshot.rearLeft), kDtSeconds));
        const int16_t rearRightCorrection = static_cast<int16_t>(pidRR.update(target.rearRight, static_cast<float>(feedbackEncoderSnapshot.rearRight), kDtSeconds));

        pidWheelCommands.frontLeft = constrain(static_cast<int16_t>(kWheelPidBaseCommand + frontLeftCorrection), 0, 500);
        pidWheelCommands.frontRight = constrain(static_cast<int16_t>(kWheelPidBaseCommand + frontRightCorrection), 0, 500);
        pidWheelCommands.rearLeft = constrain(static_cast<int16_t>(kWheelPidBaseCommand + rearLeftCorrection), 0, 500);
        pidWheelCommands.rearRight = constrain(static_cast<int16_t>(kWheelPidBaseCommand + rearRightCorrection), 0, 500);

        // Monitor.println(
        //     String("[DEBUG] PID commands: FL=") + String(pidWheelCommands.frontLeft) +
        //     " FR=" + String(pidWheelCommands.frontRight) +
        //     " RL=" + String(pidWheelCommands.rearLeft) +
        //     " RR=" + String(pidWheelCommands.rearRight) +
        //     String(" | corr FL=") + String(frontLeftCorrection) +
        //     " FR=" + String(frontRightCorrection) +
        //     " RL=" + String(rearLeftCorrection) +
        //     " RR=" + String(rearRightCorrection));
    }
    // debug_print::printDetailedLidarDistances(LiDAR, lidarAt0Deg, lidarAt90Deg, lidarAt270Deg);
    // debug_print::printTofMeasurement(tofMeasurement, tofDistanceSensor.getLastStatusCode(), tofDistanceSensor.getThresholdMm());
    // Monitor.println(
    //     String("[DEBUG] Auto state=") + String(autoStateName(AUTO_STATE)) +
    //     " | ToF stops=" + String(tofPauseTracker.completedStops) + "/" + String(tofPauseTracker.maxStops) +
    //     " | pauseArmed=" + String(tofPauseTracker.pauseArmed ? 1 : 0));
}