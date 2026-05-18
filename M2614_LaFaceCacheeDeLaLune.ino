#include <Arduino_RouterBridge.h>
#include <zephyr/kernel.h>

#include "DriveSystem.h"
#include "PINS.h"
#include "RC_Receiver.h"

// ═══════════════════════════════════════════════════════════════════════════
//  Configuration
// ═══════════════════════════════════════════════════════════════════════════

/// Channel indices (match physical wiring: pin 9 = A, pin 2 = H).
enum Channel : uint8_t {
    CH_A = 0,  // Left stick Y  → Vy (forward/backward)
    CH_B,      // Left stick X  → Vx (strafe)
    CH_C,      // Right stick Y → (unused)
    CH_D,      // Right stick X → ω  (rotation)
    CH_E,      // Left button
    CH_F,      // Right button
    CH_G,      // Left pot
    CH_H       // Right pot
};

/// GPIO pins for the 8 RC channels — ordered A→H.
static const uint8_t RC_PINS[] = {
    RC_PIN_A, RC_PIN_B, RC_PIN_C, RC_PIN_D,
    RC_PIN_E, RC_PIN_F, RC_PIN_G, RC_PIN_H};

/// Dead zone around joystick center in mapped units (±1000 range).
static constexpr int16_t JOYSTICK_DEADZONE = 100;

/// Interval between debug prints on Monitor (ms).
static constexpr unsigned long DEBUG_PRINT_INTERVAL_MS = 200;

// ═══════════════════════════════════════════════════════════════════════════
//  Motor pin descriptors (order: FL, FR, BL, BR)
// ═══════════════════════════════════════════════════════════════════════════

static const MotorPins MOTORS[4] = {
    {FL_IN1, FL_IN2, FL_EN},  // FL
    {FR_IN1, FR_IN2, FR_EN},  // FR
    {BL_IN1, BL_IN2, BL_EN},  // BL
    {BR_IN1, BR_IN2, BR_EN},  // BR
};

// ═══════════════════════════════════════════════════════════════════════════
//  RC Receiver
// ═══════════════════════════════════════════════════════════════════════════

RC_Receiver rcReceiver(RC_PINS, 8);

static int16_t applyDeadZone(int16_t value) {
    return (value > -JOYSTICK_DEADZONE && value < JOYSTICK_DEADZONE) ? 0 : value;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    pinSetup();
    Monitor.begin();

    Monitor.println("M2614 — Initialising...");

    // Apply default calibration endpoints (1000-2000 µs) on all RC channels
    for (uint8_t i = 0; i < rcReceiver.getChannelCount(); i++) {  // TODO: Only calibrate joystick channels, not buttons/pots
        rcReceiver.getChannel(i)->calibrate(1000, 2000);          // TODO: add calibration procedure by asking user to move sticks to extremes
    }

    // Attach edge interrupts to all RC channel pins so pulse widths are
    // measured on transitions instead of being polled from loop().
    rcReceiver.beginInterruptDriven();

    // Initialise drive system (runs motor calibration, starts control thread)
    if (!DriveSystem::init(MOTORS)) {
        Monitor.println("FATAL: Drive system init failed. Halting.");
        while (true) {
            k_msleep(1000);
        }
    }

    // ── Regulator mode selection ──────────────────────────────────────
    // Uncomment ONE of the following lines to select the regulator:
    // DriveSystem::setRegulatorMode(REG_P);
    // DriveSystem::setRegulatorMode(REG_PI);
    DriveSystem::setRegulatorMode(REG_PID);  // default

    // ── PID gains (tune experimentally) ──────────────────────────────
    // DriveSystem::setGains(0.5f, 0.1f, 0.01f);

    // ── Ramp limits (pps per second) ─────────────────────────────────
    // DriveSystem::setRampLimits(5000.0f, 8000.0f);

    Monitor.println("M2614 — Ready.");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Main loop — RC polling + velocity command dispatch
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    static unsigned long lastPrintTime = 0;
    unsigned long now = millis();

    // Poll RC channels as fast as possible
    rcReceiver.update();

    // Read joystick axes
    int16_t vy = applyDeadZone(rcReceiver.getChannel(CH_A)->getMappedPulseWidth());
    int16_t vx = applyDeadZone(rcReceiver.getChannel(CH_B)->getMappedPulseWidth());
    int16_t omega = applyDeadZone(rcReceiver.getChannel(CH_D)->getMappedPulseWidth());

    // Send velocity command to drive system
    if (vy == 0 && vx == 0 && omega == 0) {
        // All axes neutral — could be intentional stop or signal loss.
        // DriveSystem safety timeout handles true signal loss.
        DriveSystem::setVelocity(0, 0, 0);
    } else {
        DriveSystem::setVelocity(vx, vy, omega);
    }

    // Debug output
    if (now - lastPrintTime >= DEBUG_PRINT_INTERVAL_MS) {
        lastPrintTime = now;

        Monitor.print("[RC] Vx:");
        Monitor.print(vx);
        Monitor.print(" Vy:");
        Monitor.print(vy);
        Monitor.print(" W:");
        Monitor.print(omega);
        Monitor.print(" | E:");
        Monitor.print(rcReceiver.getChannel(CH_E)->getPulseWidth());
        Monitor.print(" F:");
        Monitor.print(rcReceiver.getChannel(CH_F)->getPulseWidth());
        Monitor.print(" G:");
        Monitor.print(rcReceiver.getChannel(CH_G)->getMappedPulseWidth());
        Monitor.print(" H:");
        Monitor.println(rcReceiver.getChannel(CH_H)->getMappedPulseWidth());
    }
}
