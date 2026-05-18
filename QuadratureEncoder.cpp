#include "QuadratureEncoder.h"

#include "PINS.h"

// Pulse counters incremented on B-channel edges.
static volatile int32_t encoderDeltas[ENCODER_COUNT] = {0, 0, 0, 0};

static void isrFL() {
    encoderDeltas[ENC_FL]++;
}

static void isrFR() {
    encoderDeltas[ENC_FR]++;
}

static void isrBL() {
    encoderDeltas[ENC_BL]++;
}

static void isrBR() {
    encoderDeltas[ENC_BR]++;
}

namespace QuadratureEncoder {

void begin() {
    // attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isrFL, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isrFR, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(BL_ENC_A), isrBL, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(BR_ENC_A), isrBR, CHANGE);

    attachInterrupt(digitalPinToInterrupt(FL_ENC_B), isrFL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FR_ENC_B), isrFR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BL_ENC_B), isrBL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BR_ENC_B), isrBR, CHANGE);
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
