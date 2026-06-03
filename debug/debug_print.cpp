#include "debug_print.h"

#include <Arduino_RouterBridge.h>

namespace {
constexpr uint32_t kRcDebugPeriodMs = 250;

uint32_t lastRcDebugPrintMs = 0;
uint32_t lastRcUpdateUs = 0;
uint32_t rcUpdateIntervalMinUs = 0xFFFFFFFFUL;
uint32_t rcUpdateIntervalMaxUs = 0;
uint32_t rcUpdateIntervalSumUs = 0;
uint32_t rcUpdateIntervalCount = 0;

const char* rcChannelName(RCChannel channel) {
    switch (channel) {
        case RCChannel::A:
            return "A";
        case RCChannel::B:
            return "B";
        case RCChannel::C:
            return "C";
        case RCChannel::D:
            return "D";
        case RCChannel::E:
            return "E";
        case RCChannel::F:
            return "F";
        case RCChannel::G:
            return "G";
        case RCChannel::H:
            return "H";
        case RCChannel::Count:
        default:
            return "?";
    }
}

void printRcDebugChannel(const RemoteController& rc, RCChannel channel) {
    const RCDebugSnapshot debug = rc.getDebugSnapshot(channel);

    Monitor.println(
        String("  ") + rcChannelName(channel) +
        " pin=" + String(debug.pin) +
        " lvl=" + String(debug.sampledHigh ? 1 : 0) +
        " valid=" + String(debug.signalValid ? 1 : 0) +
        " rawUs=" + String(debug.rawPulseWidthUs) +
        " pulseUs=" + String(debug.clampedPulseWidthUs) +
        " riseAgeUs=" + String(debug.ageSinceLastRiseUs) +
        " pulseAgeUs=" + String(debug.ageSinceLastPulseUs) +
        " edges=" + String(debug.risingEdgeCount) + "/" + String(debug.fallingEdgeCount));
}
}  // namespace

namespace debug_print {

void noteRcUpdateCadence(uint32_t nowUs) {
    if (lastRcUpdateUs != 0U) {
        const uint32_t dtUs = nowUs - lastRcUpdateUs;
        if (dtUs < rcUpdateIntervalMinUs) {
            rcUpdateIntervalMinUs = dtUs;
        }
        if (dtUs > rcUpdateIntervalMaxUs) {
            rcUpdateIntervalMaxUs = dtUs;
        }
        rcUpdateIntervalSumUs += dtUs;
        ++rcUpdateIntervalCount;
    }
    lastRcUpdateUs = nowUs;
}

void printRcDebugSummary(const RemoteController& rc, uint32_t nowMs, uint32_t elapsedMs) {
    if ((nowMs - lastRcDebugPrintMs) < kRcDebugPeriodMs) {
        return;
    }

    lastRcDebugPrintMs = nowMs;
    Monitor.println(String("[RC DEBUG] invalid drive signal dtMs=") + String(elapsedMs));
    printRcDebugChannel(rc, RCChannel::A);
    printRcDebugChannel(rc, RCChannel::B);
    printRcDebugChannel(rc, RCChannel::C);
    printRcDebugChannel(rc, RCChannel::D);
    printRcDebugChannel(rc, RCChannel::E);
    printRcDebugChannel(rc, RCChannel::F);
    printRcDebugChannel(rc, RCChannel::G);
    printRcDebugChannel(rc, RCChannel::H);
}

void printRcUpdateCadence() {
    const uint32_t minUs = (rcUpdateIntervalCount > 0U) ? rcUpdateIntervalMinUs : 0U;
    const uint32_t maxUs = (rcUpdateIntervalCount > 0U) ? rcUpdateIntervalMaxUs : 0U;
    const uint32_t avgUs = (rcUpdateIntervalCount > 0U) ? (rcUpdateIntervalSumUs / rcUpdateIntervalCount) : 0U;

    Monitor.println(
        String("[DEBUG] RC cadence avg=") + String(avgUs) + "us" +
        " min=" + String(minUs) + "us" +
        " max=" + String(maxUs) + "us" +
        " samples=" + String(rcUpdateIntervalCount));

    rcUpdateIntervalMinUs = 0xFFFFFFFFUL;
    rcUpdateIntervalMaxUs = 0;
    rcUpdateIntervalSumUs = 0;
    rcUpdateIntervalCount = 0;
}

void printLidarDistances(
    const LiDARSensor::QueryResult& lidarAt0Deg,
    const LiDARSensor::QueryResult& lidarAt90Deg,
    const LiDARSensor::QueryResult& lidarAt270Deg) {
    Monitor.println(
        String("LiDAR distances | 0°=") + String(lidarAt0Deg.distance_mm) +
        "mm | 90°=" + String(lidarAt90Deg.distance_mm) +
        "mm | 270°=" + String(lidarAt270Deg.distance_mm) + "mm");
}

void printDetailedLidarDistances(
    const LiDARSensor& lidar,
    const LiDARSensor::QueryResult& lidarAt0Deg,
    const LiDARSensor::QueryResult& lidarAt90Deg,
    const LiDARSensor::QueryResult& lidarAt270Deg) {
    Monitor.println(
        String("[DEBUG] LiDAR scan=") + String(lidar.getLastCommittedScanId()) +
        " points=" + String(lidar.getLastValidPointCount()) +
        " dropped=" + String(lidar.getDroppedScanCount()) +
        " | 0°=" + String(lidarAt0Deg.distance_mm) + "@" + String(lidarAt0Deg.matched_angle_deg_x100 * 0.01f, 2) +
        " | 90°=" + String(lidarAt90Deg.distance_mm) + "@" + String(lidarAt90Deg.matched_angle_deg_x100 * 0.01f, 2) +
        " | 270°=" + String(lidarAt270Deg.distance_mm) + "@" + String(lidarAt270Deg.matched_angle_deg_x100 * 0.01f, 2));
}

void printDriveRcSnapshot(const RemoteController& rc) {
    Monitor.println(
        String("[DEBUG] RC A=") + String(rc.getJoystick(RCChannel::A)) + " (" + String(rc.getPulseWidthUs(RCChannel::A)) + "us)" +
        " B=" + String(rc.getJoystick(RCChannel::B)) + " (" + String(rc.getPulseWidthUs(RCChannel::B)) + "us)" +
        " D=" + String(rc.getJoystick(RCChannel::D)) + " (" + String(rc.getPulseWidthUs(RCChannel::D)) + "us)");
}

void printScaledWheelCommands(float frontLeftCommand, float frontRightCommand, float rearLeftCommand, float rearRightCommand) {
    Monitor.println(
        String("[DEBUG] Wheel commands before scaling: ") +
        String(frontLeftCommand) + ", " +
        String(frontRightCommand) + ", " +
        String(rearLeftCommand) + ", " +
        String(rearRightCommand));
}

void printTargetWheelCommands(const MecanumDriver::WheelCommands& wheelCommands) {
    Monitor.println(
        String("[DEBUG] Target wheel commands: ") +
        String(wheelCommands.frontLeft) + ", " +
        String(wheelCommands.frontRight) + ", " +
        String(wheelCommands.rearLeft) + ", " +
        String(wheelCommands.rearRight));
}

}  // namespace debug_print