#include <Arduino_RouterBridge.h>

#include "LDS_LDROBOT_LD19.h"
#include "PINS.h"

namespace {

constexpr size_t kBufferedScanCapacity = 3;
constexpr size_t kMaxPointsPerScan = 512;
constexpr size_t kPointJsonEstimate = 24;
constexpr uint16_t kAngleWrapHysteresisDegX100 = 500;
constexpr uint16_t kMinimumPointsPerScan = 400;

struct StoredPoint {
    uint16_t angle_deg_x100;
    uint16_t distance_mm;
    uint8_t quality;
};

struct StoredScan {
    uint32_t scanId;
    float frequencyHz;
    uint16_t pointCount;
    StoredPoint points[kMaxPointsPerScan];
};

LDS_LDROBOT_LD19 lidar;
StoredScan scanBuffer[kBufferedScanCapacity];
StoredPoint currentScanPoints[kMaxPointsPerScan];
size_t scanBufferStart = 0;
size_t scanBufferCount = 0;
uint16_t currentPointCount = 0;
uint32_t nextScanId = 1;
uint16_t lastAngleDegX100 = 0;
bool hasLastAngle = false;

void resetCurrentScan() {
    currentPointCount = 0;
}

void appendCurrentPoint(const LDS_LDROBOT_LD19::ScanPoint& point) {
    if (point.distance_mm == 0 || currentPointCount >= kMaxPointsPerScan) {
        return;
    }

    currentScanPoints[currentPointCount++] = {
        point.angle_deg_x100,
        point.distance_mm,
        point.quality,
    };
}

void storeCompletedScan() {
    if (currentPointCount == 0) {
        return;
    }

    const size_t writeIndex = (scanBufferCount < kBufferedScanCapacity)
                                  ? (scanBufferStart + scanBufferCount) % kBufferedScanCapacity
                                  : scanBufferStart;

    if (scanBufferCount < kBufferedScanCapacity) {
        scanBufferCount++;
    } else {
        scanBufferStart = (scanBufferStart + 1) % kBufferedScanCapacity;
    }

    StoredScan& scan = scanBuffer[writeIndex];
    scan.scanId = nextScanId++;
    scan.frequencyHz = lidar.getCurrentScanFreqHz();
    scan.pointCount = currentPointCount;

    for (uint16_t index = 0; index < currentPointCount; ++index) {
        scan.points[index] = currentScanPoints[index];
    }

    resetCurrentScan();
}

size_t estimateDrainSize() {
    size_t estimatedSize = 2;

    for (size_t scanIndex = 0; scanIndex < scanBufferCount; ++scanIndex) {
        const size_t bufferIndex = (scanBufferStart + scanIndex) % kBufferedScanCapacity;
        estimatedSize += 64 + (scanBuffer[bufferIndex].pointCount * kPointJsonEstimate);
    }

    return estimatedSize;
}

String drainScans() {
    String drained;
    drained.reserve(estimateDrainSize());
    drained += '[';

    for (size_t scanIndex = 0; scanIndex < scanBufferCount; ++scanIndex) {
        const size_t bufferIndex = (scanBufferStart + scanIndex) % kBufferedScanCapacity;
        const StoredScan& scan = scanBuffer[bufferIndex];

        drained += "{\"scan_id\":";
        drained += String(scan.scanId);
        drained += ",\"frequency_hz\":";
        drained += String(scan.frequencyHz, 2);
        drained += ",\"points\":[";

        for (uint16_t pointIndex = 0; pointIndex < scan.pointCount; ++pointIndex) {
            const StoredPoint& point = scan.points[pointIndex];
            drained += '[';
            drained += String(point.angle_deg_x100);
            drained += ',';
            drained += String(point.distance_mm);
            drained += ',';
            drained += String(point.quality);
            drained += ']';

            if (pointIndex + 1 < scan.pointCount) {
                drained += ',';
            }
        }

        drained += "]}";
        if (scanIndex + 1 < scanBufferCount) {
            drained += ',';
        }
    }

    drained += ']';
    scanBufferStart = 0;
    scanBufferCount = 0;

    return drained;
}

void handlePoint(const LDS_LDROBOT_LD19::ScanPoint& point) {
    const bool angleWrapped = hasLastAngle && currentPointCount >= kMinimumPointsPerScan && point.angle_deg_x100 + kAngleWrapHysteresisDegX100 < lastAngleDegX100;
    const bool completedSweep = point.scan_completed || angleWrapped;

    // Flush the accumulated scan when the driver or angle wrap indicates a new sweep.
    if (completedSweep) {
        storeCompletedScan();
    }

    appendCurrentPoint(point);
    lastAngleDegX100 = point.angle_deg_x100;
    hasLastAngle = true;
}

}  // namespace

void setup() {
    pinMode(LED3_R, OUTPUT);
    pinMode(LED3_G, OUTPUT);
    pinMode(LED3_B, OUTPUT);
    pinMode(LED4_R, OUTPUT);
    pinMode(LED4_G, OUTPUT);
    pinMode(LED4_B, OUTPUT);
    digitalWrite(LED3_R, LOW);
    digitalWrite(LED3_G, LOW);
    digitalWrite(LED3_B, LOW);
    digitalWrite(LED4_R, LOW);
    digitalWrite(LED4_G, LOW);
    digitalWrite(LED4_B, LOW);
    if (!Bridge.begin()) {
        while (true) {
            digitalWrite(LED3_R, HIGH);
            digitalWrite(LED3_G, LOW);
            digitalWrite(LED3_B, LOW);
            delay(1000);
            digitalWrite(LED3_R, LOW);
            digitalWrite(LED3_G, LOW);
            digitalWrite(LED3_B, LOW);
        }
    }

    if (!Monitor.begin()) {
        while (true) {
            digitalWrite(LED4_R, HIGH);
            digitalWrite(LED4_G, LOW);
            digitalWrite(LED4_B, LOW);
            delay(1000);
            digitalWrite(LED4_R, LOW);
            digitalWrite(LED4_G, LOW);
            digitalWrite(LED4_B, LOW);
        }
    }

    delay(5000);

    if (!Bridge.provide_safe("lidar/drain_scans", drainScans)) {
        Monitor.println("Failed to register lidar/drain_scans");
    }

    Serial1.begin(LDS_LDROBOT_LD19::SERIAL_BAUD_RATE);
    lidar.begin(Serial1);

    Monitor.println("LD19 LiDAR bridge ready");
}

void loop() {
    LDS_LDROBOT_LD19::ScanPoint point;
    while (lidar.readPoint(point)) {
        handlePoint(point);
    }

    delay(1);
}
