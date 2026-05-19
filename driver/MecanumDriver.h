#pragma once

#include <Arduino.h>

/**
 * @struct MotorPinConfig
 * @brief GPIO mapping for one motor driver channel.
 * @author GOLETTA David
 * @date 2026-05-18
 */
struct MotorPinConfig {
    uint8_t in1;
    uint8_t in2;
    uint8_t en;
};

/**
 * @struct MecanumPins
 * @brief Complete GPIO mapping for all four mecanum wheels.
 *
 * Wheel naming follows: front-left, front-right, rear-left, rear-right.
 * @author GOLETTA David
 * @date 2026-05-18
 */
struct MecanumPins {
    MotorPinConfig frontLeft;
    MotorPinConfig frontRight;
    MotorPinConfig rearLeft;
    MotorPinConfig rearRight;
};

/**
 * @struct DirectionVector
 * @brief Planar direction command used by driveMotors.
 *
 * Components are expected in the range [-500, 500].
 * @author GOLETTA David
 * @date 2026-05-18
 */
struct DirectionVector {
    int16_t x;
    int16_t y;
};

/**
 * @class MecanumDriver
 * @brief Object-oriented motor driver for a 4-wheel mecanum base.
 *
 * This class takes a full pin configuration at construction and exposes
 * a `driveMotors(direction, rotation)` method where each component is expected
 * in [-500, 500].
 * @author GOLETTA David
 * @date 2026-05-18
 */
class MecanumDriver {
   public:
    /**
     * @struct WheelCommands
     * @brief Signed wheel commands in input scale units.
     * @author GOLETTA David
     * @date 2026-05-19
     */
    struct WheelCommands {
        int16_t frontLeft;
        int16_t frontRight;
        int16_t rearLeft;
        int16_t rearRight;
    };

    /**
     * @brief Build a mecanum driver with the full wheel pin configuration.
     * @param pins Struct containing all motor control pins.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    explicit MecanumDriver(const MecanumPins& pins);

    /**
     * @brief Configure GPIO modes and force all motors to stop.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    void begin();

    /**
     * @brief Drive the robot from a planar direction vector.
     *
     * The input vector components are clamped to [-500, 500].
     * The mecanum mix is normalized so no wheel command exceeds the supported
     * input magnitude before being mapped to PWM.
     *
     * @param direction Direction vector where x is lateral and y is longitudinal.
     * @param rotation Rotational force: -500 = CCW, +500 = CW.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    void driveMotors(const DirectionVector& direction, int16_t rotation);

    /**
     * @brief Stop all wheels immediately.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    void stop();

    /**
     * @brief Drive each wheel directly with signed input-scale commands.
     * @param frontLeft Command for the front-left wheel in [-500, 500].
     * @param frontRight Command for the front-right wheel in [-500, 500].
     * @param rearLeft Command for the rear-left wheel in [-500, 500].
     * @param rearRight Command for the rear-right wheel in [-500, 500].
     * @author GOLETTA David
     * @date 2026-05-19
     */
    void driveWheels(int16_t frontLeft, int16_t frontRight, int16_t rearLeft, int16_t rearRight);

    /**
     * @brief Compute four wheel commands from translation and rotation commands.
     * @param x Lateral command in [-500, 500].
     * @param y Longitudinal command in [-500, 500].
     * @param rotation Rotational command in [-500, 500].
     * @return WheelCommands normalized to stay within [-500, 500].
     * @author GOLETTA David
     * @date 2026-05-19
     */
    static WheelCommands mix(int16_t x, int16_t y, int16_t rotation);

   private:
    static constexpr int16_t kInputMin = -500;
    static constexpr int16_t kInputMax = 500;
    static constexpr int16_t kPwmMax = 255;

    MecanumPins pins_;

    /**
     * @brief Clamp vector component to accepted input range.
     * @param value Raw vector component.
     * @return Clamped value in [-500, 500].
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static int16_t clampInput(int16_t value);

    /**
     * @brief Convert a signed input-scale command to signed PWM.
     * @param value Signed command in [-500, 500].
     * @return Signed PWM in [-255, 255].
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static int16_t inputToPwm(int16_t value);

    /**
     * @brief Apply a signed PWM command to one motor channel.
     * @param motor Pin mapping for one wheel.
     * @param signedPwm Signed PWM in [-255, 255].
     * @author GOLETTA David
     * @date 2026-05-18
     */
    void setMotor(const MotorPinConfig& motor, int16_t signedPwm) const;
};
