#include "QuadratureEncoder.h"

#include "PINS.h"

// Signed pulse counters — incremented/decremented by ISRs.
static volatile int32_t encoderDeltas[ENCODER_COUNT] = {0, 0, 0, 0};

// Pin B lookup table for direction detection inside ISRs.
static const uint8_t encoderPinB[ENCODER_COUNT] = {FL_ENC_B, FR_ENC_B, BL_ENC_B, BR_ENC_B};

// ISR template: read channel A edge direction via channel B state.
// On CHANGE of A: if A==B → one direction, if A!=B → other direction.
static void isrFL() {
    bool a = digitalRead(FL_ENC_A);
    bool b = digitalRead(FL_ENC_B);
    encoderDeltas[ENC_FL] += (a == b) ? 1 : -1;
}

static void isrFR() {
    bool a = digitalRead(FR_ENC_A);
    bool b = digitalRead(FR_ENC_B);
    encoderDeltas[ENC_FR] += (a == b) ? 1 : -1;
}

static void isrBL() {
    bool a = digitalRead(BL_ENC_A);
    bool b = digitalRead(BL_ENC_B);
    encoderDeltas[ENC_BL] += (a == b) ? 1 : -1;
}

static void isrBR() {
    bool a = digitalRead(BR_ENC_A);
    bool b = digitalRead(BR_ENC_B);
    encoderDeltas[ENC_BR] += (a == b) ? 1 : -1;
}

namespace QuadratureEncoder {

void begin() {
    attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isrFL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isrFR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BL_ENC_A), isrBL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BR_ENC_A), isrBR, CHANGE);
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
