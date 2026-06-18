#include <Adafruit_AS7341.h>
#include <Arduino_RouterBridge.h>

#include "LiDAR/LiDARSensor.h"
#include "PINS.h"
#include "calibration/ColorCalibration.h"
#include "debug/debug_print.h"
#include "driver/FeedbackEncoder.h"
#include "driver/MecanumDriver.h"
#include "driver/RemoteController.h"
#include "driver/SpeedController.h"
#include "driver/ToFDistanceSensor.h"

// Runtime flag controlling the automatic control fallback when RC signal is lost.
// Defaults to true. Can be toggled at runtime through the RouterBridge RPC methods
// robot.autonomous.enabled (getter) and robot.autonomous.set_enabled (setter).
// Set this to false to disable the autonomous behavior (e.g. on another circuit).
bool autonomousModeEnabled = true;

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

// ===== SENSOR QUERY RESULT STRUCT =====
struct SensorSnapshot {
    ToFDistanceMeasurement tofMeasurement;
    LiDARSensor::QueryResult lidarAt0Deg;
    LiDARSensor::QueryResult lidarAt90Deg;
    LiDARSensor::QueryResult lidarAt270Deg;
};

/**
 * @brief Struct to hold longitudinal, lateral, and rotational drive commands.
 * @param longitudinal Forward/backward command in [-500, 500].
 * @param lateral Left/right command in [-500, 500].
 * @param rotation Rotational command in [-500, 500].
 */
struct DriveCommands {
    int16_t longitudinal;
    int16_t lateral;
    int16_t rotation;
};

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
constexpr uint32_t kThirdTofRisingEdgeStopDelayMs = 180;
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
    /* ki          */ 0.20f,
    /* kd          */ 0.0f,
    /* integralMin */ -90.0f,
    /* integralMax */ 90.0f,
    /* outputMin   */ -120.0f,
    /* outputMax   */ 120.0f,
};

// Open-loop forward bias used by the PID controllers.
// The PID output is a correction added on top of this baseline instead of the
// final wheel command itself.
constexpr int16_t kWheelPidBaseCommand = 50;

constexpr float kStallTargetThreshold = 6.0f;
constexpr int32_t kStallMeasuredThreshold = 2;
constexpr uint8_t kStallDetectCycles = 6;
constexpr uint32_t kStallBoostDurationMs = 250;
constexpr int16_t kStallBoostCommand = 170;

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

uint8_t pidStallCounters[4] = {0, 0, 0, 0};
uint32_t pidStallBoostUntilMs[4] = {0U, 0U, 0U, 0U};

// Tracks the previous AUTO_STATE to detect state entries (used to reset PIDs).
AutoControlState prevAutoState = AutoControlState::SLOW_FORWARD;

// Sorter latch: set on RC F falling edge while in MANUAL_CONTROL, reset outside manual.
bool isBallSorterEnabled = false;
bool previousSorterButtonPressed = false;
bool previousAutoSwitchPressed = false;

void resetPidStallRecovery() {
    for (uint8_t i = 0; i < 4; ++i) {
        pidStallCounters[i] = 0;
        pidStallBoostUntilMs[i] = 0U;
    }
}

void applyPidStallRecovery(uint8_t wheelIndex, float target, int32_t measured, int16_t& command, uint32_t nowMs) {
    if (target < kStallTargetThreshold) {
        pidStallCounters[wheelIndex] = 0;
        pidStallBoostUntilMs[wheelIndex] = 0U;
        return;
    }

    if (measured <= kStallMeasuredThreshold) {
        if (pidStallCounters[wheelIndex] < UINT8_MAX) {
            ++pidStallCounters[wheelIndex];
        }
        if (pidStallCounters[wheelIndex] >= kStallDetectCycles) {
            pidStallBoostUntilMs[wheelIndex] = nowMs + kStallBoostDurationMs;
        }
    } else {
        pidStallCounters[wheelIndex] = 0;
    }

    const uint32_t boostUntilMs = pidStallBoostUntilMs[wheelIndex];
    if (boostUntilMs != 0U) {
        if (static_cast<int32_t>(nowMs - boostUntilMs) < 0) {
            if (command < kStallBoostCommand) {
                command = kStallBoostCommand;
            }
        } else {
            pidStallBoostUntilMs[wheelIndex] = 0U;
        }
    }
}

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

/**
 * @brief RouterBridge getter for the autonomous fallback flag.
 * @return 1 when the autonomous fallback is enabled, 0 otherwise.
 * @author GOLETTA David
 * @date 2026-06-18
 */
bool getAutonomousModeEnabled() {
    return autonomousModeEnabled;
}

/**
 * @brief RouterBridge setter for the autonomous fallback flag.
 * @param enabled Non-zero to enable the autonomous fallback, 0 to disable it.
 * @return True when the new value was applied.
 * @author GOLETTA David
 * @date 2026-06-18
 */
bool setAutonomousModeEnabled(int enabled) {
    const bool newValue = (enabled != 0);
    if (newValue != autonomousModeEnabled) {
        autonomousModeEnabled = newValue;
        Monitor.println(String("Autonomous fallback ") + (newValue ? "enabled" : "disabled") + " via RouterBridge.");
    }
    return true;
}

void setup() {
    Bridge.begin();
    Monitor.begin();
    Monitor.println("===============================");
    Monitor.println("Starting up...");
    pinSetup();
    ColorCalibration::registerBridgeMethods();
    Bridge.provide("robot.autonomous.enabled", getAutonomousModeEnabled);
    Bridge.provide("robot.autonomous.set_enabled", setAutonomousModeEnabled);
    const bool calibrationModeActive = ColorCalibration::begin();
    Monitor.println(
        ColorCalibration::isSensorReady()
            ? "AS7341 ready on Wire1. Entering calibration mode."
            : "AS7341 init failed on Wire1. Continuing normal robot setup.");
    if (calibrationModeActive) {
        Monitor.println("Setup complete.");
        return;
    }

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

// ===== SENSOR UPDATE =====
SensorSnapshot updateSensors() {
    constexpr LiDARSensor::QueryResult defaultQueryResult = {false, 0U, 0U, -1, -1, 0U, 0U, 0U};
    SensorSnapshot snapshot;
    snapshot.tofMeasurement = tofDistanceSensor.getLastMeasurement();
    snapshot.lidarAt0Deg = LiDAR.queryAngleAt(0);
    snapshot.lidarAt90Deg = LiDAR.queryAngleAt(90);
    snapshot.lidarAt270Deg = LiDAR.queryAngleAt(270);
    return snapshot;
}

// ===== RC SIGNAL / CONNECTION MANAGEMENT =====
void updateConnectionState(uint32_t nowMs) {
    static bool hasSignal = true;
    const bool hasDriveSignal = rc.isSignalValid(RCChannel::A) && rc.isSignalValid(RCChannel::B) && rc.isSignalValid(RCChannel::D);
    if (hasDriveSignal) {
        driveSignalInvalidSinceMs = 0;
        if (!hasSignal) {
            CURRENT_STATE = ControlState::MANUAL_CONTROL;
            resetTofPauseTrackingStats();
            previousAutoSwitchPressed = false;
            Monitor.println("=============== RC SIGNAL RESTORED ===============");
            hasSignal = true;
        }
    } else {
        if (driveSignalInvalidSinceMs == 0U) {
            driveSignalInvalidSinceMs = nowMs;
        }
        const bool lossDebounceElapsed = (nowMs - driveSignalInvalidSinceMs) >= kRcSignalLossDebounceMs;
        if (lossDebounceElapsed && hasSignal) {
            CURRENT_STATE = autonomousModeEnabled ? ControlState::AUTOMATIC_CONTROL : ControlState::CONNECTION_LOST;
            AUTO_STATE = AutoControlState::SLOW_FORWARD;
            configureTofPauseSegment(3U, "SLOW_FORWARD", AutoControlState::SLOW_FORWARD);
            Monitor.println("=============== RC SIGNAL LOST ===============");
            Monitor.println(autonomousModeEnabled ? "Autonomous fallback enabled." : "Autonomous fallback disabled.");
            hasSignal = false;
        }
    }
}

// ===== MANUAL CONTROL =====
DriveCommands handleManualControl() {
    DriveCommands cmd;
    cmd.longitudinal = applyDeadzone(rc.getJoystick(RCChannel::A), kJoystickDeadzone);
    cmd.lateral = applyDeadzone(rc.getJoystick(RCChannel::B), kJoystickDeadzone);
    cmd.rotation = applyDeadzone(rc.getJoystick(RCChannel::D), kJoystickDeadzone);

    static uint32_t lastAutoSwitchLogMs = 0;
    const bool hasAutoSwitchSignal = rc.isSignalValid(RCChannel::E) && rc.isSignalValid(RCChannel::F);
    const bool isAutoSwitchPressed = hasAutoSwitchSignal && rc.getButton(RCChannel::E) && rc.getButton(RCChannel::F);
    const bool sorterButtonPressed = rc.getButton(RCChannel::F);
    const bool sorterButtonFallingEdge = previousSorterButtonPressed && !sorterButtonPressed;
    const uint16_t pulseE = rc.getPulseWidthUs(RCChannel::E);
    const uint16_t pulseF = rc.getPulseWidthUs(RCChannel::F);

    if (isAutoSwitchPressed && !previousAutoSwitchPressed) {
        CURRENT_STATE = ControlState::AUTOMATIC_CONTROL;
        AUTO_STATE = AutoControlState::SLOW_FORWARD;
        configureTofPauseSegment(3U, "SLOW_FORWARD", AutoControlState::SLOW_FORWARD);
        Monitor.println("=============== SWITCHING TO AUTOMATIC CONTROL MODE ===============");
    }
    previousAutoSwitchPressed = isAutoSwitchPressed;

    // Latch behavior in MANUAL_CONTROL:
    // - Toggle on sorter button falling edge (release).
    // - Reset whenever the auto switch combo is active.
    if (isAutoSwitchPressed) {
        isBallSorterEnabled = false;
    } else if (sorterButtonFallingEdge) {
        isBallSorterEnabled = !isBallSorterEnabled;
    }

    digitalWrite(ENABLE_SORTER, isBallSorterEnabled);
    previousSorterButtonPressed = sorterButtonPressed;
    return cmd;
}

// ===== TOF PAUSE LOGIC (shared by SLOW_FORWARD states) =====
void handleTofPause(AutoControlState resumeState, const ToFDistanceMeasurement& tofMeasurement, uint32_t nowMs) {
    if (!hasRemainingTofPauseBudget()) return;

    if (tofMeasurement.isValid && !tofMeasurement.isThresholdReached) {
        tofPauseTracker.pauseArmed = true;
        tofPauseTracker.stopPending = false;
        tofPauseTracker.stopPendingSinceMs = 0U;
    } else if (tofMeasurement.isValid && tofMeasurement.isThresholdReached && tofPauseTracker.pauseArmed) {
        if (!tofPauseTracker.stopPending) {
            tofPauseTracker.stopPending = true;
            tofPauseTracker.stopPendingSinceMs = nowMs;
        }
        const uint32_t stopPendingDurationMs = nowMs - tofPauseTracker.stopPendingSinceMs;
        if (tofPauseTracker.completedStops == 2U) {
            if (stopPendingDurationMs >= kThirdTofRisingEdgeStopDelayMs) {
                pidWheelCommands = {0, 0, 0, 0};
                enterTofPauseStop(resumeState, nowMs);
            }
        } else if (stopPendingDurationMs >= kTofRisingEdgeStopDelayMs) {
            pidWheelCommands = {0, 0, 0, 0};
            enterTofPauseStop(resumeState, nowMs);
        }
    }
}

// ===== AUTO STATE HANDLERS =====
DriveCommands handleAutoSlowForward(const SensorSnapshot& sensors, uint32_t nowMs) {
    if (prevAutoState != AUTO_STATE) {
        pidFL.reset();
        pidFR.reset();
        pidRL.reset();
        pidRR.reset();
        resetPidStallRecovery();
        pidWheelCommands = {0, 0, 0, 0};
    }
    inPidForwardMode = true;

    handleTofPause(AutoControlState::SLOW_FORWARD, sensors.tofMeasurement, nowMs);

    if (sensors.lidarAt0Deg.found && sensors.lidarAt0Deg.distance_mm < 25) {
        clearTofPauseTracking();
        AUTO_STATE = AutoControlState::TURN;
        Monitor.println("[DEBUG] Transitioning to TURN: Obstacle detected at 0° at " + String(sensors.lidarAt0Deg.distance_mm) + "/25 mm");
    }
    return {0, 0, 0};
}

DriveCommands handleAutoTurn(const SensorSnapshot& sensors) {
    if (sensors.lidarAt0Deg.found && sensors.lidarAt0Deg.distance_mm > 430) {
        AUTO_STATE = AutoControlState::REPOSITION;
        Monitor.println("[DEBUG] Transitioning to REPOSITION: Path ahead at 0° is " + String(sensors.lidarAt0Deg.distance_mm) + "mm");
    }
    return {0, 0, -150};
}

DriveCommands handleAutoReposition(const SensorSnapshot& sensors, uint32_t nowMs) {
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
    return {50, 325, -30};
}

DriveCommands handleAutoSlowForward2(const SensorSnapshot& sensors, uint32_t nowMs) {
    if (prevAutoState != AUTO_STATE) {
        pidFL.reset();
        pidFR.reset();
        pidRL.reset();
        pidRR.reset();
        resetPidStallRecovery();
        pidWheelCommands = {0, 0, 0, 0};
    }
    inPidForwardMode = true;

    handleTofPause(AutoControlState::SLOW_FORWARD_2, sensors.tofMeasurement, nowMs);

    if (sensors.lidarAt0Deg.found && sensors.lidarAt0Deg.distance_mm < 25) {
        clearTofPauseTracking();
        AUTO_STATE = AutoControlState::EXIT_TURN;
        Monitor.println("[DEBUG] Transitioning to EXIT_TURN: Obstacle detected at 0° at " + String(sensors.lidarAt0Deg.distance_mm) + "/25 mm");
    }
    return {0, 0, 0};
}

DriveCommands handleAutoTofPauseStop(uint32_t nowMs) {
    if ((nowMs - tofPauseTracker.pauseStartMs) >= kTofPauseDurationMs) {
        AUTO_STATE = tofPauseTracker.resumeState;
        tofPauseTracker.pauseStartMs = 0U;
        Monitor.println(
            String("[DEBUG] Leaving TOF_PAUSE_STOP, resuming ") +
            String(autoStateName(AUTO_STATE)) +
            " | completed stops=" + String(tofPauseTracker.completedStops) + "/" + String(tofPauseTracker.maxStops));
    }
    return {0, 0, 0};
}

DriveCommands handleAutoExitTurn(const SensorSnapshot& sensors) {
    if (sensors.lidarAt0Deg.found && sensors.lidarAt0Deg.distance_mm > 2000) {
        AUTO_STATE = AutoControlState::EXIT_FORWARD;
        Monitor.println("[DEBUG] Transitioning to EXIT_FORWARD: Path ahead at 0° is clear beyond " + String(sensors.lidarAt0Deg.distance_mm) + "mm");
    }
    return {0, 0, -150};
}

DriveCommands handleAutoExitForward(const SensorSnapshot& sensors) {
    if (sensors.lidarAt0Deg.found && sensors.lidarAt0Deg.distance_mm < 450) {
        clearTofPauseTracking();
        AUTO_STATE = AutoControlState::STOP;
        Monitor.println("[DEBUG] Transitioning to STOP: Obstacle detected at 0° at " + String(sensors.lidarAt0Deg.distance_mm) + "/350 mm");
    }
    return {250, 0, 0};
}

DriveCommands handleAutoStop() {
    clearTofPauseTracking();
    mecanumDriver.stop();
    CURRENT_STATE = ControlState::MANUAL_CONTROL;
    AUTO_STATE = AutoControlState::SLOW_FORWARD;
    resetTofPauseTrackingStats();
    previousAutoSwitchPressed = false;
    Monitor.println("=============== SWITCHING TO MANUAL CONTROL MODE ===============");
    return {0, 0, 0};
}

// ===== AUTOMATIC CONTROL DISPATCHER =====
DriveCommands handleAutomaticControl(const SensorSnapshot& sensors, uint32_t nowMs) {
    DriveCommands cmd = {0, 0, 0};
    switch (AUTO_STATE) {
        case AutoControlState::SLOW_FORWARD:
            cmd = handleAutoSlowForward(sensors, nowMs);
            break;
        case AutoControlState::TURN:
            cmd = handleAutoTurn(sensors);
            break;
        case AutoControlState::REPOSITION:
            cmd = handleAutoReposition(sensors, nowMs);
            break;
        case AutoControlState::SLOW_FORWARD_2:
            cmd = handleAutoSlowForward2(sensors, nowMs);
            break;
        case AutoControlState::TOF_PAUSE_STOP:
            cmd = handleAutoTofPauseStop(nowMs);
            break;
        case AutoControlState::EXIT_TURN:
            cmd = handleAutoExitTurn(sensors);
            break;
        case AutoControlState::EXIT_FORWARD:
            cmd = handleAutoExitForward(sensors);
            break;
        case AutoControlState::STOP:
            cmd = handleAutoStop();
            break;
    }
    prevAutoState = AUTO_STATE;
    return cmd;
}

// ===== DRIVE OUTPUT =====
void applyDriveCommands(const DriveCommands& cmd) {
    if (inPidForwardMode) {
        // PID mode: drive wheels directly from last PID output — no mix(), no scaling.
        mecanumDriver.driveWheels(
            pidWheelCommands.frontLeft,
            pidWheelCommands.frontRight,
            pidWheelCommands.rearLeft,
            pidWheelCommands.rearRight);
    } else {
        const MecanumDriver::WheelCommands targetWheelCommands = MecanumDriver::mix(cmd.lateral, cmd.longitudinal, cmd.rotation);

        constexpr float speedScalingFactor = 0.65f;
        constexpr float speedOffset = 0.0f;
        const int16_t fl = static_cast<int16_t>(static_cast<float>(targetWheelCommands.frontLeft) * speedScalingFactor + speedOffset);
        const int16_t fr = static_cast<int16_t>(static_cast<float>(targetWheelCommands.frontRight) * speedScalingFactor + speedOffset);
        const int16_t rl = static_cast<int16_t>(static_cast<float>(targetWheelCommands.rearLeft) * speedScalingFactor + speedOffset);
        const int16_t rr = static_cast<int16_t>(static_cast<float>(targetWheelCommands.rearRight) * speedScalingFactor + speedOffset);

        mecanumDriver.driveWheels(fl, fr, rl, rr);
    }
}

// ===== PID UPDATE =====
void updatePid() {
    tofDistanceSensor.update();
    EncoderSpeedSnapshot feedbackEncoderSnapshot = feedbackEncoder.getCurrentSpeed();
    // debug_print::printEncoderPulsesPer50Ms(feedbackEncoderSnapshot);

    if (inPidForwardMode) {
        const WheelSpeedTargets& target = kSlowForwardTargets;
        const uint32_t nowMs = millis();
        constexpr float kDtSeconds = static_cast<float>(FeedbackEncoder::kSamplePeriodMs) / 1000.0f;
        const int16_t frontLeftCorrection = static_cast<int16_t>(pidFL.update(target.frontLeft, static_cast<float>(feedbackEncoderSnapshot.frontLeft), kDtSeconds));
        const int16_t frontRightCorrection = static_cast<int16_t>(pidFR.update(target.frontRight, static_cast<float>(feedbackEncoderSnapshot.frontRight), kDtSeconds));
        const int16_t rearLeftCorrection = static_cast<int16_t>(pidRL.update(target.rearLeft, static_cast<float>(feedbackEncoderSnapshot.rearLeft), kDtSeconds));
        const int16_t rearRightCorrection = static_cast<int16_t>(pidRR.update(target.rearRight, static_cast<float>(feedbackEncoderSnapshot.rearRight), kDtSeconds));

        int16_t frontLeftCommand = constrain(static_cast<int16_t>(kWheelPidBaseCommand + frontLeftCorrection), 0, 500);
        int16_t frontRightCommand = constrain(static_cast<int16_t>(kWheelPidBaseCommand + frontRightCorrection), 0, 500);
        int16_t rearLeftCommand = constrain(static_cast<int16_t>(kWheelPidBaseCommand + rearLeftCorrection), 0, 500);
        int16_t rearRightCommand = constrain(static_cast<int16_t>(kWheelPidBaseCommand + rearRightCorrection), 0, 500);

        applyPidStallRecovery(0, target.frontLeft, feedbackEncoderSnapshot.frontLeft, frontLeftCommand, nowMs);
        applyPidStallRecovery(1, target.frontRight, feedbackEncoderSnapshot.frontRight, frontRightCommand, nowMs);
        applyPidStallRecovery(2, target.rearLeft, feedbackEncoderSnapshot.rearLeft, rearLeftCommand, nowMs);
        applyPidStallRecovery(3, target.rearRight, feedbackEncoderSnapshot.rearRight, rearRightCommand, nowMs);

        pidWheelCommands.frontLeft = frontLeftCommand;
        pidWheelCommands.frontRight = frontRightCommand;
        pidWheelCommands.rearLeft = rearLeftCommand;
        pidWheelCommands.rearRight = rearRightCommand;
    } else {
        resetPidStallRecovery();
    }
}

void loop() {
    if (ColorCalibration::isActive()) {
        ColorCalibration::update(millis());
        return;
    }

    inPidForwardMode = false;
    rc.update();
    LiDAR.update();
    // tofDistanceSensor.update();

    const uint32_t nowMs = millis();
    const uint32_t elapsedMs = nowMs - lastControlUpdateMs;

    const SensorSnapshot sensors = updateSensors();

    updateConnectionState(nowMs);

    DriveCommands cmd = {0, 0, 0};

    switch (CURRENT_STATE) {
        case ControlState::MANUAL_CONTROL:
            cmd = handleManualControl();
            break;
        case ControlState::AUTOMATIC_CONTROL:
            isBallSorterEnabled = false;
            previousSorterButtonPressed = rc.getButton(RCChannel::F);
            previousAutoSwitchPressed = false;
            digitalWrite(ENABLE_SORTER, LOW);
            cmd = handleAutomaticControl(sensors, nowMs);
            break;
        case ControlState::CONNECTION_LOST:
            isBallSorterEnabled = false;
            previousSorterButtonPressed = rc.getButton(RCChannel::F);
            previousAutoSwitchPressed = false;
            digitalWrite(ENABLE_SORTER, LOW);
            mecanumDriver.stop();
            return;
    }

    applyDriveCommands(cmd);

    // ===== DEBUG PRINT ZONE =====
    if (elapsedMs < kDebugPrintDelayMs) return;
    lastControlUpdateMs = nowMs;

    // ===== PID UPDATE (every 50 ms, synchronized with encoder sample window) =====
    updatePid();

    // debug_print::printDetailedLidarDistances(LiDAR, sensors.lidarAt0Deg, sensors.lidarAt90Deg, sensors.lidarAt270Deg);
    // debug_print::printTofMeasurement(sensors.tofMeasurement, tofDistanceSensor.getLastStatusCode(), tofDistanceSensor.getThresholdMm());
    // Monitor.println(
    //     String("[DEBUG] Auto state=") + String(autoStateName(AUTO_STATE)) +
    //     " | ToF stops=" + String(tofPauseTracker.completedStops) + "/" + String(tofPauseTracker.maxStops) +
    //     " | pauseArmed=" + String(tofPauseTracker.pauseArmed ? 1 : 0));
}