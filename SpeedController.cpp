#include "SpeedController.h"

SpeedController::SpeedController() {}

void SpeedController::setMode(RegulatorMode mode) {
    _mode = mode;
}

void SpeedController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void SpeedController::setRampLimits(float accelPpsPerSec, float decelPpsPerSec) {
    _maxAccelPpsPerSec = accelPpsPerSec;
    _maxDecelPpsPerSec = decelPpsPerSec;
}

void SpeedController::setTarget(int32_t targetPps) {
    _targetPps = targetPps;
}

int16_t SpeedController::update(int32_t measuredPps, float dt) {
    if (dt <= 0.0f) return 0;

    // --- Ramp: move _rampedSetpoint towards _targetPps ---
    float target = (float)_targetPps;
    float diff = target - _rampedSetpoint;

    if (fabsf(diff) > 0.5f) {
        // Determine if we are accelerating or decelerating.
        // Accelerating = moving away from zero; decelerating = moving towards zero.
        bool accelerating;
        if (fabsf(target) >= fabsf(_rampedSetpoint)) {
            accelerating = true;
        } else {
            accelerating = false;
        }

        float maxStep = (accelerating ? _maxAccelPpsPerSec : _maxDecelPpsPerSec) * dt;

        if (diff > maxStep) {
            _rampedSetpoint += maxStep;
        } else if (diff < -maxStep) {
            _rampedSetpoint -= maxStep;
        } else {
            _rampedSetpoint = target;
        }
    } else {
        _rampedSetpoint = target;
    }

    // --- PID error ---
    float error = _rampedSetpoint - (float)measuredPps;

    // Proportional
    float output = _kp * error;

    // Integral (PI and PID modes)
    if (_mode == REG_PI || _mode == REG_PID) {
        // Keep the integrator continuous across target changes.
        if ((_rampedSetpoint > 0 && _prevError < 0 && error > 0) ||
            (_rampedSetpoint < 0 && _prevError > 0 && error < 0)) {
            // No action needed here.
        }

        _integral += error * dt;

        // Anti-windup clamp
        if (_integral > INTEGRAL_LIMIT) _integral = INTEGRAL_LIMIT;
        if (_integral < -INTEGRAL_LIMIT) _integral = -INTEGRAL_LIMIT;

        output += _ki * _integral;
    }

    // Derivative (PID mode only)
    if (_mode == REG_PID) {
        if (!_firstUpdate) {
            float derivative = (error - _prevError) / dt;
            output += _kd * derivative;
        }
    }

    _prevError = error;
    _firstUpdate = false;

    // Reset integrator when target is zero and setpoint has reached zero
    if (_targetPps == 0 && fabsf(_rampedSetpoint) < 1.0f) {
        _integral = 0.0f;
        _rampedSetpoint = 0.0f;
    }

    return clampPwm(output);
}

void SpeedController::reset() {
    _targetPps = 0;
    _rampedSetpoint = 0.0f;
    _integral = 0.0f;
    _prevError = 0.0f;
    _firstUpdate = true;
}

int16_t SpeedController::clampPwm(float v) {
    if (v > 255.0f) return 255;
    if (v < -255.0f) return -255;
    return (int16_t)v;
}
