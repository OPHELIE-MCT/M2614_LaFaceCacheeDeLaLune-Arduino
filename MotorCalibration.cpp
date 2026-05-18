#include "MotorCalibration.h"

#include <Arduino_RouterBridge.h>

#include "QuadratureEncoder.h"

static int32_t maxReferencePps = 0;

static void setMotorRaw(const MotorPins& m, int16_t pwmSigned) {
    if (pwmSigned > 255) pwmSigned = 255;
    if (pwmSigned < -255) pwmSigned = -255;

    if (pwmSigned > 0) {
        digitalWrite(m.in1, HIGH);
        digitalWrite(m.in2, LOW);
        analogWrite(m.en, pwmSigned);
    } else if (pwmSigned < 0) {
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, HIGH);
        analogWrite(m.en, -pwmSigned);
    } else {
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, LOW);
        analogWrite(m.en, 0);
    }
}

static void stopMotor(const MotorPins& m) {
    setMotorRaw(m, 0);
}

namespace MotorCalibration {

int32_t calibrate(const MotorPins motors[4], uint32_t testDurationMs) {
    static const char* names[4] = {"FL", "FR", "BL", "BR"};
    int32_t ppsResults[4][2];  // [motor][0=CW, 1=CCW]
    int32_t slowest = INT32_MAX;

    Monitor.println("[CAL] Starting motor calibration...");

    for (uint8_t m = 0; m < 4; m++) {
        // --- CW test ---
        QuadratureEncoder::resetDelta(m);
        setMotorRaw(motors[m], 255);
        delay(testDurationMs);
        stopMotor(motors[m]);

        int32_t deltaCW = QuadratureEncoder::readAndResetDelta(m);
        // Convert pulse count over testDurationMs to pulses per second
        int32_t ppsCW = (int32_t)((int64_t)abs(deltaCW) * 1000 / (int64_t)testDurationMs);
        ppsResults[m][0] = ppsCW;

        delay(300);  // brief pause between directions

        // --- CCW test ---
        QuadratureEncoder::resetDelta(m);
        setMotorRaw(motors[m], -255);
        delay(testDurationMs);
        stopMotor(motors[m]);

        int32_t deltaCCW = QuadratureEncoder::readAndResetDelta(m);
        int32_t ppsCCW = (int32_t)((int64_t)abs(deltaCCW) * 1000 / (int64_t)testDurationMs);
        ppsResults[m][1] = ppsCCW;

        delay(300);  // pause before next motor

        // Track the slowest
        int32_t minOfBoth = min(ppsCW, ppsCCW);
        if (minOfBoth < slowest) {
            slowest = minOfBoth;
        }

        Monitor.print("[CAL] ");
        Monitor.print(names[m]);
        Monitor.print(": CW=");
        Monitor.print(ppsCW);
        Monitor.print(" pps, CCW=");
        Monitor.print(ppsCCW);
        Monitor.println(" pps");
    }

    if (slowest <= 0) {
        Monitor.println("[CAL] ERROR: at least one motor produced 0 pps!");
        maxReferencePps = 0;
        return 0;
    }

    maxReferencePps = slowest;

    Monitor.print("[CAL] Reference max speed: ");
    Monitor.print(maxReferencePps);
    Monitor.println(" pps");

    return maxReferencePps;
}

int32_t getMaxReferencePps() {
    return maxReferencePps;
}

}  // namespace MotorCalibration
