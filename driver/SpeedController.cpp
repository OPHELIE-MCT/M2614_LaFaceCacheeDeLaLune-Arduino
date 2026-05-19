#include "SpeedController.h"

namespace {
/**
 * @brief Clamp a value between lower and upper bounds.
 * @param value Input value.
 * @param minimum Lower bound.
 * @param maximum Upper bound.
 * @return Clamped value in [minimum, maximum].
 * @author GOLETTA David
 * @date 2026-05-19
 */
float clampFloat(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}
}  // namespace

PSpeedController::PSpeedController(const SpeedControllerConfig& config) : config_(config) {}

void PSpeedController::reset() {}

float PSpeedController::update(float setpoint, float measured, float dtSeconds) {
    (void)dtSeconds;
    const float error = setpoint - measured;
    const float output = config_.kp * error;
    return clampFloat(output, config_.outputMin, config_.outputMax);
}

PISpeedController::PISpeedController(const SpeedControllerConfig& config) : config_(config), integralTerm_(0.0f) {}

void PISpeedController::reset() {
    integralTerm_ = 0.0f;
}

float PISpeedController::update(float setpoint, float measured, float dtSeconds) {
    const float error = setpoint - measured;

    if (dtSeconds > 0.0f) {
        integralTerm_ += error * dtSeconds;
        integralTerm_ = clampFloat(integralTerm_, config_.integralMin, config_.integralMax);
    }

    const float output = (config_.kp * error) + (config_.ki * integralTerm_);
    return clampFloat(output, config_.outputMin, config_.outputMax);
}

PIDSpeedController::PIDSpeedController(const SpeedControllerConfig& config)
    : config_(config), integralTerm_(0.0f), previousError_(0.0f), hasPreviousError_(false) {}

void PIDSpeedController::reset() {
    integralTerm_ = 0.0f;
    previousError_ = 0.0f;
    hasPreviousError_ = false;
}

float PIDSpeedController::update(float setpoint, float measured, float dtSeconds) {
    const float error = setpoint - measured;

    if (dtSeconds > 0.0f) {
        integralTerm_ += error * dtSeconds;
        integralTerm_ = clampFloat(integralTerm_, config_.integralMin, config_.integralMax);
    }

    float derivativeTerm = 0.0f;
    if (hasPreviousError_ && dtSeconds > 0.0f) {
        derivativeTerm = (error - previousError_) / dtSeconds;
    }

    previousError_ = error;
    hasPreviousError_ = true;

    const float output = (config_.kp * error) + (config_.ki * integralTerm_) + (config_.kd * derivativeTerm);
    return clampFloat(output, config_.outputMin, config_.outputMax);
}
