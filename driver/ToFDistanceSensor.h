#pragma once

#include <Arduino.h>
#include <DFRobot_VL6180X.h>
#include <Wire.h>

/**
 * @file ToFDistanceSensor.h
 * @brief High-level wrapper for the VL6180X time-of-flight distance sensor.
 */

/**
 * @struct ToFDistanceMeasurement
 * @brief Snapshot returned by the ToF sensor polling API.
 * @author GitHub Copilot
 * @date 2026-06-03
 */
struct ToFDistanceMeasurement {
    float distanceMm;
    bool isValid;
    bool isThresholdReached;
};

/**
 * @class ToFDistanceSensor
 * @brief Loop-driven VL6180X wrapper that caches the latest distance measurement.
 *
 * The wrapper keeps the DFRobot library isolated from the main sketch and
 * exposes one polling method that returns a compact measurement struct suited
 * for both debug output and later stop-threshold logic.
 *
 * @author GitHub Copilot
 * @date 2026-06-03
 */
class ToFDistanceSensor {
   public:
    static constexpr float kDefaultThresholdMm = 40.0f;
    static constexpr uint8_t kInvalidStatusCode = 0xFF;

    /**
     * @brief Construct an idle ToF wrapper with an invalid cached reading.
     * @author GitHub Copilot
     * @date 2026-06-03
     */
    ToFDistanceSensor();

    /**
     * @brief Initialize the VL6180X on the provided I2C bus and store the threshold.
     * @param wire I2C bus instance connected to the sensor.
     * @param thresholdMm Distance threshold in millimeters used for threshold reporting.
     * @return True when the sensor responds and initialization succeeds, otherwise false.
     * @author GitHub Copilot
     * @date 2026-06-03
     */
    bool begin(TwoWire& wire, float thresholdMm = kDefaultThresholdMm);

    /**
     * @brief Poll the sensor once and return the latest measurement snapshot.
     * @details The returned distance is the raw value from the latest poll. Check isValid before trusting it.
     * @return Latest measurement snapshot containing distance, validity, and threshold state.
     * @author GitHub Copilot
     * @date 2026-06-03
     */
    ToFDistanceMeasurement update();

    /**
     * @brief Return the last cached measurement without polling the sensor again.
     * @return Cached measurement snapshot from the latest update().
     * @author GitHub Copilot
     * @date 2026-06-03
     */
    ToFDistanceMeasurement getLastMeasurement() const;

    /**
     * @brief Return the currently configured threshold in millimeters.
     * @return Threshold used to populate isThresholdReached.
     * @author GitHub Copilot
     * @date 2026-06-03
     */
    float getThresholdMm() const;

    /**
     * @brief Return the status code reported by the last VL6180X poll.
     * @return Library status code, or 0xFF when the sensor has not started.
     * @author GitHub Copilot
     * @date 2026-06-03
     */
    uint8_t getLastStatusCode() const;

    /**
     * @brief Check whether the sensor completed a successful begin sequence.
     * @return True when the wrapper is ready to poll hardware, otherwise false.
     * @author GitHub Copilot
     * @date 2026-06-03
     */
    bool isStarted() const;

   private:
    DFRobot_VL6180X sensor_;
    ToFDistanceMeasurement lastMeasurement_;
    float thresholdMm_;
    uint8_t lastStatusCode_;
    bool started_;
};