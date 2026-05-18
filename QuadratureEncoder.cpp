#include "QuadratureEncoder.h"

#include <Arduino_RouterBridge.h>

#include "PINS.h"

// Pulse counters incremented on B-channel edges.
static volatile int32_t encoderDeltas[ENCODER_COUNT] = {0, 0, 0, 0};

static void incrementFrontLeftEncoder() {
    encoderDeltas[ENC_FL]++;
}

static void incrementFrontRightEncoder() {
    encoderDeltas[ENC_FR]++;
}

static void incrementBackLeftEncoder() {
    encoderDeltas[ENC_BL]++;
}

static void incrementBackRightEncoder() {
    encoderDeltas[ENC_BR]++;
}

static void logInterruptConfig(const char* label, uint8_t pin) {
    int interruptId = digitalPinToInterrupt(pin);

    Monitor.print("[ENC] ");
    Monitor.print(label);
    Monitor.print(" pin=");
    Monitor.print(pin);
    Monitor.print(" irq=");
    Monitor.println(interruptId);

    if (interruptId < 0) {
        Monitor.print("[ENC] WARNING: ");
        Monitor.print(label);
        Monitor.println(" is not interrupt-capable on this core.");
    }
}

namespace QuadratureEncoder {

void begin() {
    // logInterruptConfig("FL_ENC_B", FL_ENC_B);
    // logInterruptConfig("FR_ENC_B", FR_ENC_B);
    // logInterruptConfig("BL_ENC_B", BL_ENC_B);
    // logInterruptConfig("BR_ENC_B", BR_ENC_B);

    attachInterrupt(digitalPinToInterrupt(FL_ENC_B), incrementFrontLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FR_ENC_B), incrementFrontRightEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BL_ENC_B), incrementBackLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BR_ENC_B), incrementBackRightEncoder, CHANGE);
}

int32_t readAndResetDelta(uint8_t index) {
    if (index >= ENCODER_COUNT) return 0;
    noInterrupts();
    int32_t val = encoderDeltas[index];
    encoderDeltas[index] = 0;
    interrupts();
    return val;
}

int32_t readDelta(uint8_t index) {
    if (index >= ENCODER_COUNT) return 0;
    noInterrupts();
    int32_t val = encoderDeltas[index];
    interrupts();
    return val;
}

void resetDelta(uint8_t index) {
    if (index >= ENCODER_COUNT) return;
    noInterrupts();
    encoderDeltas[index] = 0;
    interrupts();
}

}  // namespace QuadratureEncoder
