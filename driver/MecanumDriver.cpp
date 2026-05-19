#include "MecanumDriver.h"

namespace {
int32_t abs32(int32_t value) {
    return (value < 0) ? -value : value;
}
}  // namespace

MecanumDriver::MecanumDriver(const MecanumPins& pins) : pins_(pins) {}

void MecanumDriver::begin() {
    pinMode(pins_.frontLeft.in1, OUTPUT);
    pinMode(pins_.frontLeft.in2, OUTPUT);
    pinMode(pins_.frontLeft.en, OUTPUT);

    pinMode(pins_.frontRight.in1, OUTPUT);
    pinMode(pins_.frontRight.in2, OUTPUT);
    pinMode(pins_.frontRight.en, OUTPUT);

    pinMode(pins_.rearLeft.in1, OUTPUT);
    pinMode(pins_.rearLeft.in2, OUTPUT);
    pinMode(pins_.rearLeft.en, OUTPUT);

    pinMode(pins_.rearRight.in1, OUTPUT);
    pinMode(pins_.rearRight.in2, OUTPUT);
    pinMode(pins_.rearRight.en, OUTPUT);

    stop();
}

void MecanumDriver::driveMotors(const DirectionVector& direction, int16_t rotation) {
    const int16_t xClamped = clampInput(direction.x);
    const int16_t yClamped = clampInput(direction.y);
    const int16_t rotationClamped = clampInput(rotation);

    const WheelCommands wheelCommands = mix(xClamped, yClamped, rotationClamped);

    setMotor(pins_.frontLeft, inputToPwm(wheelCommands.frontLeft));
    setMotor(pins_.frontRight, inputToPwm(wheelCommands.frontRight));
    setMotor(pins_.rearLeft, inputToPwm(wheelCommands.rearLeft));
    setMotor(pins_.rearRight, inputToPwm(wheelCommands.rearRight));
}

void MecanumDriver::stop() {
    setMotor(pins_.frontLeft, 0);
    setMotor(pins_.frontRight, 0);
    setMotor(pins_.rearLeft, 0);
    setMotor(pins_.rearRight, 0);
}

void MecanumDriver::driveWheels(int16_t frontLeft, int16_t frontRight, int16_t rearLeft,
                                int16_t rearRight) {
    setMotor(pins_.frontLeft, inputToPwm(clampInput(frontLeft)));
    setMotor(pins_.frontRight, inputToPwm(clampInput(frontRight)));
    setMotor(pins_.rearLeft, inputToPwm(clampInput(rearLeft)));
    setMotor(pins_.rearRight, inputToPwm(clampInput(rearRight)));
}

int16_t MecanumDriver::clampInput(int16_t value) {
    if (value > kInputMax) {
        return kInputMax;
    }
    if (value < kInputMin) {
        return kInputMin;
    }
    return value;
}

int16_t MecanumDriver::inputToPwm(int16_t value) {
    const int32_t clamped = clampInput(value);
    return static_cast<int16_t>((clamped * kPwmMax) / kInputMax);
}

MecanumDriver::WheelCommands MecanumDriver::mix(int16_t x, int16_t y, int16_t rotation) {
    const int32_t frontLeftRaw = static_cast<int32_t>(y) + x + rotation;
    const int32_t frontRightRaw = static_cast<int32_t>(y) - x - rotation;
    const int32_t rearLeftRaw = static_cast<int32_t>(y) - x + rotation;
    const int32_t rearRightRaw = static_cast<int32_t>(y) + x - rotation;

    int32_t maxMagnitude = abs32(frontLeftRaw);
    if (abs32(frontRightRaw) > maxMagnitude) maxMagnitude = abs32(frontRightRaw);
    if (abs32(rearLeftRaw) > maxMagnitude) maxMagnitude = abs32(rearLeftRaw);
    if (abs32(rearRightRaw) > maxMagnitude) maxMagnitude = abs32(rearRightRaw);

    if (maxMagnitude < kInputMax) {
        maxMagnitude = kInputMax;
    }

    WheelCommands cmd;
    cmd.frontLeft = static_cast<int16_t>((frontLeftRaw * kInputMax) / maxMagnitude);
    cmd.frontRight = static_cast<int16_t>((frontRightRaw * kInputMax) / maxMagnitude);
    cmd.rearLeft = static_cast<int16_t>((rearLeftRaw * kInputMax) / maxMagnitude);
    cmd.rearRight = static_cast<int16_t>((rearRightRaw * kInputMax) / maxMagnitude);
    return cmd;
}

void MecanumDriver::setMotor(const MotorPinConfig& motor, int16_t signedPwm) const {
    if (signedPwm > 0) {
        digitalWrite(motor.in1, HIGH);
        digitalWrite(motor.in2, LOW);
        analogWrite(motor.en, static_cast<uint8_t>(signedPwm));
    } else if (signedPwm < 0) {
        digitalWrite(motor.in1, LOW);
        digitalWrite(motor.in2, HIGH);
        analogWrite(motor.en, static_cast<uint8_t>(-signedPwm));
    } else {
        digitalWrite(motor.in1, LOW);
        digitalWrite(motor.in2, LOW);
        analogWrite(motor.en, 0);
    }
}
