#pragma once

#include <Arduino.h>

/**
 * @file ColorCalibration.h
 * @brief RouterBridge-driven AS7341 calibration capture service for the Uno Q MCU.
 *
 * This module owns the optional AS7341 initialization, the calibration-only mode
 * latch, and the RouterBridge RPC/notification contract used by the Python SBC
 * application during color-sensor recalibration.
 */

/**
 * @namespace ColorCalibration
 * @brief Public API for the AS7341 calibration capture workflow.
 *
 * The main sketch calls this namespace from setup() and loop() while the module
 * keeps the AS7341 driver, capture state, RouterBridge callbacks, and sample
 * throttling isolated from the robot control state machine.
 *
 * @author GOLETTA David
 * @date 2026-06-15
 */
namespace ColorCalibration {

/**
 * @brief Register the RouterBridge RPC methods exposed to the SBC application.
 *
 * Registered methods:
 * - color_sensor.capture.start
 * - color_sensor.capture.stop
 * - color_sensor.capture.enabled
 * - color_sensor.sensor.ready
 * - color_sensor.capture.samples_sent
 *
 * @author GOLETTA David
 * @date 2026-06-15
 */
void registerBridgeMethods();

/**
 * @brief Initialize the AS7341 on Wire1 and decide whether calibration mode is active.
 * @return True when the AS7341 responds and the sketch should run in calibration-only mode.
 * @author GOLETTA David
 * @date 2026-06-15
 */
bool begin();

/**
 * @brief Check whether calibration-only mode is active.
 * @return True when the AS7341 initialized successfully during begin().
 * @author GOLETTA David
 * @date 2026-06-15
 */
bool isActive();

/**
 * @brief Check whether the AS7341 sensor is available.
 * @return True when the sensor initialized successfully, otherwise false.
 * @author GOLETTA David
 * @date 2026-06-15
 */
bool isSensorReady();

/**
 * @brief Check whether the SBC has enabled sample capture.
 * @return True while the capture session is enabled through RouterBridge.
 * @author GOLETTA David
 * @date 2026-06-15
 */
bool isCaptureEnabled();

/**
 * @brief Return how many samples have been sent since the latest begin().
 * @return Count of successful color_sensor.sample notifications.
 * @author GOLETTA David
 * @date 2026-06-15
 */
uint32_t samplesSent();

/**
 * @brief Publish one color sample when capture is enabled and the sample period elapsed.
 * @param nowMs Current monotonic timestamp from millis().
 *
 * The method is non-blocking apart from the AS7341 read itself. It sends the
 * 10 projected AS7341 feature channels with Bridge.notify("color_sensor.sample", ...).
 *
 * @author GOLETTA David
 * @date 2026-06-15
 */
void update(unsigned long nowMs);

}  // namespace ColorCalibration
