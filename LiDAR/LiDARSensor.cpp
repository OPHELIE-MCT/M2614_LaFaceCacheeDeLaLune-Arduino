#include "LiDARSensor.h"

LiDARSensor::LiDARSensor()
    : read_buffer_index_(0),
      write_buffer_index_(1),
      has_committed_scan_(false),
      committed_scan_id_(0),
      dropped_scan_count_(0) {
    reset();
}

void LiDARSensor::begin(Stream& serial) {
    parser_.begin(serial);
    reset();
}

void LiDARSensor::reset() {
    parser_.reset();
    read_buffer_index_ = 0;
    write_buffer_index_ = 1;
    has_committed_scan_ = false;
    committed_scan_id_ = 0;
    dropped_scan_count_ = 0;

    clearScanBuffer(0);
    clearScanBuffer(1);
}

void LiDARSensor::update() {
    LDS_LDROBOT_LD19::ScanPoint point;
    while (parser_.readPoint(point)) {
        if (point.scan_completed) {
            finalizeWriteBuffer();
        }

        accumulatePointToWriteBuffer(point);
    }
}

uint16_t LiDARSensor::readAngleAt(uint16_t angleInDegrees) const {
    return queryAngleAt(angleInDegrees).distance_mm;
}

LiDARSensor::QueryResult LiDARSensor::queryAngleAt(uint16_t angleInDegrees) const {
    QueryResult result = {
        false,
        angleInDegrees,
        0U,
        0,
        kNoDistance,
        0U,
        committed_scan_id_,
        getLastValidPointCount(),
    };

    if (!has_committed_scan_) {
        return result;
    }

    const ScanBuffer& buffer = readBuffer();
    const uint16_t requestedAngle = angleInDegrees % kAngleCount;
    for (uint16_t offset = 0; offset <= kMaxQueryAngleDeltaDeg; ++offset) {
        const uint16_t upperIndex = wrapAngleIndex(static_cast<int16_t>(requestedAngle + offset));
        if (buffer.valid[upperIndex]) {
            result.found = true;
            result.matched_angle_deg_x100 = buffer.angle_deg_x100[upperIndex];
            result.angular_error_deg_x100 = shortestAngularErrorDegX100(angleInDegrees, result.matched_angle_deg_x100);
            result.distance_mm = buffer.distance_mm[upperIndex];
            result.quality = buffer.quality[upperIndex];
            return result;
        }

        const uint16_t lowerIndex = wrapAngleIndex(static_cast<int16_t>(requestedAngle) - static_cast<int16_t>(offset));
        if (lowerIndex != upperIndex && buffer.valid[lowerIndex]) {
            result.found = true;
            result.matched_angle_deg_x100 = buffer.angle_deg_x100[lowerIndex];
            result.angular_error_deg_x100 = shortestAngularErrorDegX100(angleInDegrees, result.matched_angle_deg_x100);
            result.distance_mm = buffer.distance_mm[lowerIndex];
            result.quality = buffer.quality[lowerIndex];
            return result;
        }
    }

    return result;
}

bool LiDARSensor::isStarted() const {
    return parser_.isStarted();
}

uint16_t LiDARSensor::getLastValidPointCount() const {
    return has_committed_scan_ ? readBuffer().unique_angle_count : 0U;
}

uint32_t LiDARSensor::getLastCommittedScanId() const {
    return committed_scan_id_;
}

uint32_t LiDARSensor::getDroppedScanCount() const {
    return dropped_scan_count_;
}

bool LiDARSensor::hasValidScan() const {
    return has_committed_scan_;
}

void LiDARSensor::accumulatePointToWriteBuffer(const LDS_LDROBOT_LD19::ScanPoint& point) {
    if (point.distance_mm == 0U) {
        return;
    }

    ScanBuffer& buffer = writeBuffer();
    const uint16_t angleIndex = toNearestDegreeIndex(point.angle_deg_x100);
    if (!buffer.valid[angleIndex]) {
        buffer.valid[angleIndex] = true;
        ++buffer.unique_angle_count;
    }

    ++buffer.accepted_point_count;
    buffer.distance_mm[angleIndex] = point.distance_mm;
    buffer.angle_deg_x100[angleIndex] = point.angle_deg_x100;
    buffer.quality[angleIndex] = point.quality;
}

void LiDARSensor::finalizeWriteBuffer() {
    if (shouldCommitWriteBuffer()) {
        const uint8_t previousReadIndex = read_buffer_index_;
        read_buffer_index_ = write_buffer_index_;
        write_buffer_index_ = previousReadIndex;
        has_committed_scan_ = true;
        ++committed_scan_id_;
    } else if (writeBuffer().accepted_point_count > 0U) {
        ++dropped_scan_count_;
    }

    clearScanBuffer(write_buffer_index_);
}

bool LiDARSensor::shouldCommitWriteBuffer() const {
    const ScanBuffer& buffer = scan_buffers_[write_buffer_index_];
    return buffer.unique_angle_count >= kMinUniqueAnglesForCommit &&
           buffer.accepted_point_count >= kMinAcceptedPointsForCommit;
}

void LiDARSensor::clearScanBuffer(uint8_t bufferIndex) {
    ScanBuffer& buffer = scan_buffers_[bufferIndex];
    buffer.unique_angle_count = 0U;
    buffer.accepted_point_count = 0U;

    for (uint16_t angleIndex = 0; angleIndex < kAngleCount; ++angleIndex) {
        buffer.distance_mm[angleIndex] = kNoDistance;
        buffer.angle_deg_x100[angleIndex] = 0U;
        buffer.quality[angleIndex] = 0U;
        buffer.valid[angleIndex] = false;
    }
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

int16_t LiDARSensor::shortestAngularErrorDegX100(uint16_t requestedAngleDeg, uint16_t matchedAngleDegX100) {
    int32_t errorDegX100 = static_cast<int32_t>(matchedAngleDegX100) - static_cast<int32_t>(requestedAngleDeg % 360U) * 100;

    while (errorDegX100 <= -18000) {
        errorDegX100 += 36000;
    }

    while (errorDegX100 > 18000) {
        errorDegX100 -= 36000;
    }

    return static_cast<int16_t>(errorDegX100);
}

const LiDARSensor::ScanBuffer& LiDARSensor::readBuffer() const {
    return scan_buffers_[read_buffer_index_];
}

LiDARSensor::ScanBuffer& LiDARSensor::writeBuffer() {
    return scan_buffers_[write_buffer_index_];
}