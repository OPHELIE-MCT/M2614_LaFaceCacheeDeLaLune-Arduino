#include "test_sequences.h"

#include <Arduino.h>
#include <Arduino_RouterBridge.h>

#include "../PINS.h"

namespace {
void runMotorStep(const char* label, uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin) {
    Monitor.println(label);
    analogWrite(pwmPin, 128);

    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    delay(2000);

    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    delay(2000);

    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);
}
}  // namespace

namespace test_sequences {

void testMotors() {
    runMotorStep("Testing front left motor...", FL_EN, FL_IN1, FL_IN2);
    runMotorStep("Testing front right motor...", FR_EN, FR_IN1, FR_IN2);
    runMotorStep("Testing rear left motor...", BL_EN, BL_IN1, BL_IN2);
    runMotorStep("Testing rear right motor...", BR_EN, BR_IN1, BR_IN2);

    Monitor.println("Motor test sequence complete.");
}

}  // namespace test_sequences