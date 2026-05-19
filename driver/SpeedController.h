#pragma once

#include <Arduino.h>

/**
 * @struct SpeedControllerConfig
 * @brief Shared gain and clamp configuration for speed regulators.
 * @author GOLETTA David
 * @date 2026-05-19
 */
struct SpeedControllerConfig {
    float kp;
    float ki;
    float kd;
    float integralMin;
    float integralMax;
    float outputMin;
    float outputMax;
};

/**
 * @class SpeedController
 * @brief Abstract speed controller interface used by all regulator types.
 * @author GOLETTA David
 * @date 2026-05-19
 */
class SpeedController {
   public:
    /**
     * @brief Virtual destructor for polymorphic use.
     * @author GOLETTA David
     * @date 2026-05-19
     */
    virtual ~SpeedController() = default;

    /**
     * @brief Reset internal controller state.
     * @author GOLETTA David
     * @date 2026-05-19
     */
    virtual void reset() = 0;

    /**
     * @brief Compute the next control command from target and measured speed.
     * @param setpoint Desired speed value.
     * @param measured Current measured speed value.
     * @param dtSeconds Time since previous update in seconds.
     * @return Controller output clamped to configured bounds.
     * @author GOLETTA David
     * @date 2026-05-19
     */
    virtual float update(float setpoint, float measured, float dtSeconds) = 0;
};

/**
 * @class PSpeedController
 * @brief Proportional-only speed regulator.
 * @author GOLETTA David
 * @date 2026-05-19
 */
class PSpeedController : public SpeedController {
   public:
    /**
     * @brief Build a P speed regulator from a shared configuration.
     * @param config Controller gains and clamp values.
     * @author GOLETTA David
     * @date 2026-05-19
     */
    explicit PSpeedController(const SpeedControllerConfig& config);

    /**
     * @brief Reset internal state used by the controller.
     * @author GOLETTA David
     * @date 2026-05-19
     */
    void reset() override;

    /**
     * @brief Compute proportional output from speed error.
     * @param setpoint Desired speed value.
     * @param measured Current measured speed value.
     * @param dtSeconds Time since previous update in seconds.
     * @return Proportional output clamped to configured bounds.
     * @author GOLETTA David
     * @date 2026-05-19
     */
    float update(float setpoint, float measured, float dtSeconds) override;

   private:
    SpeedControllerConfig config_;
};

/**
 * @class PISpeedController
 * @brief Proportional-integral speed regulator with integral clamping.
 * @author GOLETTA David
 * @date 2026-05-19
 */
class PISpeedController : public SpeedController {
   public:
    /**
     * @brief Build a PI speed regulator from a shared configuration.
     * @param config Controller gains and clamp values.
     * @author GOLETTA David
     * @date 2026-05-19
     */
    explicit PISpeedController(const SpeedControllerConfig& config);

    /**
     * @brief Reset integral accumulation state.
     * @author GOLETTA David
     * @date 2026-05-19
     */
    void reset() override;

    /**
     * @brief Compute PI output from speed error.
     * @param setpoint Desired speed value.
     * @param measured Current measured speed value.
     * @param dtSeconds Time since previous update in seconds.
     * @return PI output clamped to configured bounds.
     * @author GOLETTA David
     * @date 2026-05-19
     */
    float update(float setpoint, float measured, float dtSeconds) override;

   private:
    SpeedControllerConfig config_;
    float integralTerm_;
};

/**
 * @class PIDSpeedController
 * @brief Proportional-integral-derivative speed regulator.
 * @author GOLETTA David
 * @date 2026-05-19
 */
class PIDSpeedController : public SpeedController {
   public:
    /**
     * @brief Build a PID speed regulator from a shared configuration.
     * @param config Controller gains and clamp values.
     * @author GOLETTA David
     * @date 2026-05-19
     */
    explicit PIDSpeedController(const SpeedControllerConfig& config);

    /**
     * @brief Reset integral and derivative history state.
     * @author GOLETTA David
     * @date 2026-05-19
     */
    void reset() override;

    /**
     * @brief Compute PID output from speed error.
     * @param setpoint Desired speed value.
     * @param measured Current measured speed value.
     * @param dtSeconds Time since previous update in seconds.
     * @return PID output clamped to configured bounds.
     * @author GOLETTA David
     * @date 2026-05-19
     */
    float update(float setpoint, float measured, float dtSeconds) override;

   private:
    SpeedControllerConfig config_;
    float integralTerm_;
    float previousError_;
    bool hasPreviousError_;
};
