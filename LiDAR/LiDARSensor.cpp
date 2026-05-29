#include "LiDARSensor.h"

LiDARSensor::LiDARSensor() : validAngleCount_(0) {
    reset();
}

void LiDARSensor::begin(Stream& serial) {
    parser_.begin(serial);
    reset();
}

void LiDARSensor::reset() {
    parser_.reset();
    validAngleCount_ = 0;

    for (uint16_t angleIndex = 0; angleIndex < kAngleCount; ++angleIndex) {
        cachedDistancesMm_[angleIndex] = kNoDistance;
        hasDistance_[angleIndex] = false;
    }
}

void LiDARSensor::update() {
    LDS_LDROBOT_LD19::ScanPoint point;
    while (parser_.readPoint(point)) {
        cachePoint(point);
    }
}

uint16_t LiDARSensor::readAngleAt(uint8_t angleInDegrees) const {
    if (validAngleCount_ == 0U) {
        return kNoDistance;
    }

    const uint16_t requestedAngle = angleInDegrees % kAngleCount;
    for (uint16_t offset = 0; offset <= (kAngleCount / 2U); ++offset) {
        const uint16_t upperIndex = wrapAngleIndex(static_cast<int16_t>(requestedAngle + offset));
        if (hasDistance_[upperIndex]) {
            return cachedDistancesMm_[upperIndex];
        }

        const uint16_t lowerIndex = wrapAngleIndex(static_cast<int16_t>(requestedAngle) - static_cast<int16_t>(offset));
        if (lowerIndex != upperIndex && hasDistance_[lowerIndex]) {
            return cachedDistancesMm_[lowerIndex];
        }
    }

    return kNoDistance;
}

bool LiDARSensor::isStarted() const {
    return parser_.isStarted();
}

void LiDARSensor::cachePoint(const LDS_LDROBOT_LD19::ScanPoint& point) {
    if (point.distance_mm == 0U) {
        return;
    }

    const uint16_t angleIndex = toNearestDegreeIndex(point.angle_deg_x100);
    if (!hasDistance_[angleIndex]) {
        ++validAngleCount_;
        hasDistance_[angleIndex] = true;
    }

    cachedDistancesMm_[angleIndex] = point.distance_mm;
}

uint16_t LiDARSensor::toNearestDegreeIndex(uint16_t angleDegX100) {
    const uint16_t normalizedAngle = angleDegX100 % 36000U;
    uint16_t angleIndex = (normalizedAngle + 50U) / 100U;
    if (angleIndex >= kAngleCount) {
        angleIndex -= kAngleCount;
    }
    return angleIndex;
}

uint16_t LiDARSensor::wrapAngleIndex(int16_t angleIndex) {
    while (angleIndex < 0) {
        angleIndex += static_cast<int16_t>(kAngleCount);
    }

    while (angleIndex >= static_cast<int16_t>(kAngleCount)) {
        angleIndex -= static_cast<int16_t>(kAngleCount);
    }

    return static_cast<uint16_t>(angleIndex);
}