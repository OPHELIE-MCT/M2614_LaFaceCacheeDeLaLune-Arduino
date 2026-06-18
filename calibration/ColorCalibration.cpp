#include "ColorCalibration.h"

#include <Adafruit_AS7341.h>
#include <Arduino_RouterBridge.h>

namespace ColorCalibration {
namespace {

constexpr uint8_t kAs7341Address = 57;
constexpr size_t kColorRawChannelCount = 12;
constexpr size_t kColorFeatureCount = 10;
constexpr uint8_t kColorFeatureIndexes[kColorFeatureCount] = {0, 1, 2, 3, 6, 7, 8, 9, 10, 11};
constexpr unsigned long kColorSampleIntervalMs = 100;
constexpr uint16_t kColorIntegrationAtime = 100;
constexpr uint16_t kColorIntegrationAstep = 100;
constexpr uint16_t kColorLedCurrentMa = 5;

constexpr char kStartCaptureMethod[] = "color_sensor.capture.start";
constexpr char kStopCaptureMethod[] = "color_sensor.capture.stop";
constexpr char kCaptureEnabledMethod[] = "color_sensor.capture.enabled";
constexpr char kSensorReadyMethod[] = "color_sensor.sensor.ready";
constexpr char kSamplesSentMethod[] = "color_sensor.capture.samples_sent";
constexpr char kSampleMethod[] = "color_sensor.sample";

Adafruit_AS7341 colorSensor;

bool colorSensorReady = false;
bool calibrationModeActive = false;
bool captureEnabled = false;
unsigned long lastColorSampleAtMs = 0;
uint32_t colorSamplesSent = 0;

bool initializeColorSensor() {
    if (!colorSensor.begin(kAs7341Address, &Wire1)) {
        return false;
    }

    colorSensor.setATIME(kColorIntegrationAtime);
    colorSensor.setASTEP(kColorIntegrationAstep);
    colorSensor.setGain(AS7341_GAIN_256X);
    colorSensor.setLEDCurrent(kColorLedCurrentMa);
    colorSensor.enableLED(true);
    return true;
}

bool readColorFeatures(uint16_t features[kColorFeatureCount]) {
    uint16_t rawReadings[kColorRawChannelCount];
    if (!colorSensor.readAllChannels(rawReadings)) {
        return false;
    }

    for (size_t index = 0; index < kColorFeatureCount; ++index) {
        features[index] = rawReadings[kColorFeatureIndexes[index]];
    }

    return true;
}

bool startCapture() {
    if (!calibrationModeActive || !colorSensorReady) {
        Monitor.println("Capture start rejected: AS7341 is not ready.");
        return false;
    }

    captureEnabled = true;
    lastColorSampleAtMs = 0;
    Monitor.println("Capture enabled.");
    return true;
}

bool stopCapture() {
    const bool wasEnabled = captureEnabled;
    captureEnabled = false;
    Monitor.println(wasEnabled ? "Capture disabled." : "Capture already disabled.");
    return true;
}

}  // namespace

void registerBridgeMethods() {
    const bool startRegistered = Bridge.provide(kStartCaptureMethod, startCapture);
    const bool stopRegistered = Bridge.provide(kStopCaptureMethod, stopCapture);
    const bool captureRegistered = Bridge.provide(kCaptureEnabledMethod, isCaptureEnabled);
    const bool readyRegistered = Bridge.provide(kSensorReadyMethod, isSensorReady);
    const bool samplesRegistered = Bridge.provide(kSamplesSentMethod, samplesSent);

    Monitor.println(
        String("Bridge methods registered: start=") + (startRegistered ? "ok" : "failed") +
        ", stop=" + (stopRegistered ? "ok" : "failed") +
        ", enabled=" + (captureRegistered ? "ok" : "failed") +
        ", ready=" + (readyRegistered ? "ok" : "failed") +
        ", samples=" + (samplesRegistered ? "ok" : "failed"));
}

bool begin() {
    colorSensorReady = initializeColorSensor();
    calibrationModeActive = colorSensorReady;
    captureEnabled = false;
    lastColorSampleAtMs = 0;
    colorSamplesSent = 0;
    return calibrationModeActive;
}

bool isActive() {
    return calibrationModeActive;
}

bool isSensorReady() {
    return colorSensorReady;
}

bool isCaptureEnabled() {
    return captureEnabled;
}

uint32_t samplesSent() {
    return colorSamplesSent;
}

void update(unsigned long nowMs) {
    if (!captureEnabled || !colorSensorReady) {
        return;
    }

    if (lastColorSampleAtMs != 0 && (nowMs - lastColorSampleAtMs) < kColorSampleIntervalMs) {
        return;
    }

    uint16_t features[kColorFeatureCount];
    if (!readColorFeatures(features)) {
        Monitor.println("AS7341 read failed.");
        return;
    }

    lastColorSampleAtMs = nowMs;
    ++colorSamplesSent;
    Bridge.notify(
        kSampleMethod,
        features[0], features[1], features[2], features[3], features[4],
        features[5], features[6], features[7], features[8], features[9]);
}

}  // namespace ColorCalibration
