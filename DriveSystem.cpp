#include "DriveSystem.h"

#include <Arduino_RouterBridge.h>
#include <zephyr/kernel.h>

#include "MecanumMixer.h"
#include "QuadratureEncoder.h"

// ═══════════════════════════════════════════════════════════════════════════
//  Configuration
// ═══════════════════════════════════════════════════════════════════════════

/// Control loop period in milliseconds (50 Hz).
static constexpr uint32_t CONTROL_PERIOD_MS = 20;

/// Safety timeout: stop motors if setVelocity() not called within this window.
static constexpr uint32_t COMMAND_TIMEOUT_MS = 250;

/// Safety: minimum time at zero before allowing direction reversal.
static constexpr uint32_t DIRECTION_CHANGE_DELAY_MS = 500;

/// Zephyr thread configuration.
static constexpr size_t DRIVE_THREAD_STACK_SIZE = 2048;
static constexpr int DRIVE_THREAD_PRIORITY = 5;

// ═══════════════════════════════════════════════════════════════════════════
//  Internal state
// ═══════════════════════════════════════════════════════════════════════════

static const MotorPins* _motors = nullptr;
static SpeedController controllers[4];
static int32_t maxRefPps = 0;

// Current per-wheel target in pps (set by setVelocity, consumed by thread).
static volatile int32_t wheelTargetPps[4] = {0, 0, 0, 0};
static volatile unsigned long lastCommandTime = 0;
static volatile bool running = false;
static volatile bool stopped = true;

// Applied PWM tracking for direction-change safety.
static int16_t appliedPwm[4] = {0, 0, 0, 0};
static unsigned long zeroSince[4] = {0, 0, 0, 0};

// Zephyr thread
static struct k_thread driveThreadData;
static k_thread_stack_t* driveThreadStack = nullptr;

// ═══════════════════════════════════════════════════════════════════════════
//  Low-level motor control (same as MotorCalibration)
// ═══════════════════════════════════════════════════════════════════════════

static void setMotorRaw(const MotorPins& m, int16_t pwm) {
    if (pwm > 255) pwm = 255;
    if (pwm < -255) pwm = -255;

    if (pwm > 0) {
        digitalWrite(m.in1, HIGH);
        digitalWrite(m.in2, LOW);
        analogWrite(m.en, pwm);
    } else if (pwm < 0) {
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, HIGH);
        analogWrite(m.en, -pwm);
    } else {
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, LOW);
        analogWrite(m.en, 0);
    }
}

static void stopAllMotorsRaw() {
    for (uint8_t i = 0; i < 4; i++) {
        setMotorRaw(_motors[i], 0);
        appliedPwm[i] = 0;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Direction-change safety (ported from Mecanum-Controller prototype)
// ═══════════════════════════════════════════════════════════════════════════

static void applyMotorWithSafety(uint8_t idx, int16_t pwm) {
    unsigned long now = millis();

    bool directionChange =
        ((appliedPwm[idx] > 0) && (pwm < 0)) ||
        ((appliedPwm[idx] < 0) && (pwm > 0));

    if (directionChange) {
        // Force to zero first
        setMotorRaw(_motors[idx], 0);
        appliedPwm[idx] = 0;
        zeroSince[idx] = now;
        return;
    }

    if (pwm == 0) {
        setMotorRaw(_motors[idx], 0);
        if (appliedPwm[idx] != 0) {
            zeroSince[idx] = now;
        }
        appliedPwm[idx] = 0;
        return;
    }

    if (appliedPwm[idx] == 0) {
        // Only allow turning back on after the safety delay
        if (now - zeroSince[idx] >= DIRECTION_CHANGE_DELAY_MS) {
            setMotorRaw(_motors[idx], pwm);
            appliedPwm[idx] = pwm;
        } else {
            setMotorRaw(_motors[idx], 0);
        }
        return;
    }

    // Same direction — apply directly
    setMotorRaw(_motors[idx], pwm);
    appliedPwm[idx] = pwm;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Control loop thread
// ═══════════════════════════════════════════════════════════════════════════

static void controlThread(void*, void*, void*) {
    const float dt = (float)CONTROL_PERIOD_MS / 1000.0f;

    while (true) {
        k_msleep(CONTROL_PERIOD_MS);

        if (!running) continue;

        unsigned long now = millis();

        // Safety timeout
        if (now - lastCommandTime > COMMAND_TIMEOUT_MS) {
            // No fresh command — stop
            for (uint8_t i = 0; i < 4; i++) {
                controllers[i].setTarget(0);
            }
        }

        // Read encoders and compute PPS
        for (uint8_t i = 0; i < 4; i++) {
            int32_t delta = QuadratureEncoder::readAndResetDelta(i);
            // Convert delta pulses over CONTROL_PERIOD_MS to pps
            int32_t measuredPps = (int32_t)((int64_t)delta * 1000 / (int64_t)CONTROL_PERIOD_MS);

            // Load the latest target
            noInterrupts();
            int32_t target = wheelTargetPps[i];
            interrupts();
            controllers[i].setTarget(target);

            // Run PID
            int16_t pwm = controllers[i].update(measuredPps, dt);

            // Apply with direction-change safety
            applyMotorWithSafety(i, pwm);
        }

        stopped = false;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Public API
// ═══════════════════════════════════════════════════════════════════════════

namespace DriveSystem {

bool init(const MotorPins motors[4]) {
    _motors = motors;

    // Initialise quadrature encoders
    QuadratureEncoder::begin();

    // Run motor calibration
    maxRefPps = MotorCalibration::calibrate(motors, 2000);
    if (maxRefPps <= 0) {
        Monitor.println("[DRV] Calibration failed — drive system disabled.");
        return false;
    }

    // Initialise zero-since timestamps
    unsigned long now = millis();
    for (uint8_t i = 0; i < 4; i++) {
        zeroSince[i] = now;
        appliedPwm[i] = 0;
        controllers[i].reset();
    }

    // Allocate and start Zephyr thread
    driveThreadStack = k_thread_stack_alloc(DRIVE_THREAD_STACK_SIZE, 0);
    if (driveThreadStack == nullptr) {
        Monitor.println("[DRV] Failed to allocate thread stack.");
        return false;
    }

    lastCommandTime = millis();
    running = true;

    k_thread_create(&driveThreadData, driveThreadStack, DRIVE_THREAD_STACK_SIZE,
                    controlThread, nullptr, nullptr, nullptr,
                    DRIVE_THREAD_PRIORITY, 0, K_NO_WAIT);

    Monitor.println("[DRV] Drive system initialised.");
    return true;
}

void setVelocity(int16_t vx, int16_t vy, int16_t omega) {
    WheelSpeeds w = MecanumMixer::compute(vx, vy, omega);

    // Scale from ±1000 wheel units to ±maxRefPps
    noInterrupts();
    wheelTargetPps[0] = (int32_t)((int64_t)w.fl * maxRefPps / 1000);
    wheelTargetPps[1] = (int32_t)((int64_t)w.fr * maxRefPps / 1000);
    wheelTargetPps[2] = (int32_t)((int64_t)w.bl * maxRefPps / 1000);
    wheelTargetPps[3] = (int32_t)((int64_t)w.br * maxRefPps / 1000);
    lastCommandTime = millis();
    interrupts();
}

void stop() {
    noInterrupts();
    for (uint8_t i = 0; i < 4; i++) {
        wheelTargetPps[i] = 0;
    }
    interrupts();

    // Immediately cut power
    stopAllMotorsRaw();

    for (uint8_t i = 0; i < 4; i++) {
        controllers[i].reset();
    }
    stopped = true;
}

void setRegulatorMode(RegulatorMode mode) {
    for (uint8_t i = 0; i < 4; i++) {
        controllers[i].setMode(mode);
    }
}

void setGains(float kp, float ki, float kd) {
    for (uint8_t i = 0; i < 4; i++) {
        controllers[i].setGains(kp, ki, kd);
    }
}

void setRampLimits(float accelPpsPerSec, float decelPpsPerSec) {
    for (uint8_t i = 0; i < 4; i++) {
        controllers[i].setRampLimits(accelPpsPerSec, decelPpsPerSec);
    }
}

bool isRunning() {
    return running;
}

}  // namespace DriveSystem
