#include "ToFDistanceSensor.h"

namespace {
float sanitizeThreshold(float thresholdMm) {
    return (thresholdMm < 0.0f) ? 0.0f : thresholdMm;
}
}  // namespace

ToFDistanceSensor::ToFDistanceSensor()
    : sensor_(),
      lastMeasurement_{0.0f, false, false},
      thresholdMm_(kDefaultThresholdMm),
      lastStatusCode_(kInvalidStatusCode),
      started_(false) {}

bool ToFDistanceSensor::begin(TwoWire& wire, float thresholdMm) {
    thresholdMm_ = sanitizeThreshold(thresholdMm);
    sensor_ = DFRobot_VL6180X(VL6180X_IIC_ADDRESS, &wire);
    started_ = sensor_.begin();
    lastMeasurement_ = {0.0f, false, false};
    lastStatusCode_ = started_ ? VL6180X_NO_ERR : kInvalidStatusCode;
    return started_;
}

void ToFDistanceSensor::update() {
    if (!started_) {
        lastMeasurement_ = {0.0f, false, false};
        lastStatusCode_ = kInvalidStatusCode;
        return;
    }

    const uint8_t distanceMm = sensor_.rangePollMeasurement();
    lastStatusCode_ = sensor_.getRangeResult();
    lastMeasurement_.distanceMm = static_cast<float>(distanceMm);
    lastMeasurement_.isValid = (lastStatusCode_ == VL6180X_NO_ERR);
    lastMeasurement_.isThresholdReached = lastMeasurement_.isValid && (lastMeasurement_.distanceMm <= thresholdMm_);
}

ToFDistanceMeasurement ToFDistanceSensor::getLastMeasurement() const {
    return lastMeasurement_;
}

float ToFDistanceSensor::getThresholdMm() const {
    return thresholdMm_;
}

uint8_t ToFDistanceSensor::getLastStatusCode() const {
    return lastStatusCode_;
}

bool ToFDistanceSensor::isStarted() const {
    return started_;
}