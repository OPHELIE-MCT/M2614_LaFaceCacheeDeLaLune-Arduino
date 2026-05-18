#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <Arduino.h>

/// Regulator algorithm selection — switch by changing ACTIVE_REGULATOR_MODE
/// or calling setMode() at runtime.
enum RegulatorMode : uint8_t { REG_P = 0,
                               REG_PI = 1,
                               REG_PID = 2 };

/// Per-motor speed controller with selectable P / PI / PID regulation
/// and a linear acceleration ramp.
class SpeedController {
   public:
    SpeedController();

    /// Set the regulator algorithm.
    void setMode(RegulatorMode mode);

    /// Set PID gains (unused terms are ignored depending on mode).
    void setGains(float kp, float ki, float kd);

    /// Set the maximum acceleration/deceleration in pps per second.
    void setRampLimits(float accelPpsPerSec, float decelPpsPerSec);

    /// Set the desired target speed in pps.
    void setTarget(int32_t targetPps);

    /// Run one control cycle.
    /// @param measuredPps  Current measured speed in pps.
    /// @param dt           Time since last call in seconds.
    /// @return PWM output in the range [−255, +255].
    int16_t update(int32_t measuredPps, float dt);

    /// Reset all internal state (integrator, ramp, previous error).
    void reset();

    /// Get the current ramped setpoint (for telemetry / debugging).
    int32_t getRampedSetpoint() const { return _rampedSetpoint; }

   private:
    RegulatorMode _mode = REG_PID;

    // Gains
    float _kp = 0.5f;
    float _ki = 0.1f;
    float _kd = 0.01f;

    // Target and ramp
    int32_t _targetPps = 0;
    float _rampedSetpoint = 0.0f;
    float _maxAccelPpsPerSec = 5000.0f;
    float _maxDecelPpsPerSec = 8000.0f;

    // PID state
    float _integral = 0.0f;
    float _prevError = 0.0f;
    bool _firstUpdate = true;

    // Anti-windup integrator limits (in PWM-seconds)
    static constexpr float INTEGRAL_LIMIT = 255.0f;

    static int16_t clampPwm(float v);
};

#endif
