#pragma once

#include <Arduino.h>

#include "LDS_LDROBOT_LD19.h"

/**
 * @class LiDARSensor
 * @brief High-level LD19 wrapper that caches the latest distance per degree.
 *
 * The wrapper owns the low-level packet parser and exposes a loop-driven API:
 * `update()` drains the serial stream and refreshes the cache, while
 * `readAngleAt()` performs a pure lookup against the latest stored values.
 *
 * @author GOLETTA David
 * @date 2026-05-29
 */
class LiDARSensor {
   public:
    static constexpr uint16_t kAngleCount = 360;
    static constexpr uint16_t kNoDistance = 0;

    /**
     * @brief Construct a LiDAR sensor wrapper with an empty cache.
     * @author GOLETTA David
     * @date 2026-05-29
     */
    LiDARSensor();

    /**
     * @brief Attach the LD19 parser to a serial stream and clear cached data.
     * @param serial Stream connected to the LD19 UART output.
     * @author GOLETTA David
     * @date 2026-05-29
     */
    void begin(Stream& serial);

    /**
     * @brief Clear the cached angle map and reset the low-level parser state.
     * @author GOLETTA David
     * @date 2026-05-29
     */
    void reset();

    /**
     * @brief Drain all currently available LD19 points and refresh the cache.
     *
     * @details This method is intentionally non-blocking. It repeatedly asks
     * the low-level parser for points until no full point is immediately
     * available, then returns control to the sketch loop.
     *
     * @author GOLETTA David
     * @date 2026-05-29
     */
    void update();

    /**
     * @brief Read the last known distance for the closest cached angle.
     *
     * @details The search starts at the requested angle and expands
     * symmetrically with wraparound across 0 and 359 degrees. The method
     * returns 0 when no valid sample has been cached yet.
     *
     * @param angleInDegrees Requested heading in whole degrees.
     * @return Cached distance in millimeters, or 0 if no cached sample exists.
     * @author GOLETTA David
     * @date 2026-05-29
     */
    uint16_t readAngleAt(uint8_t angleInDegrees) const;

    /**
     * @brief Check whether the wrapper has been attached to a valid serial stream.
     * @return True when the low-level parser has been started, otherwise false.
     * @author GOLETTA David
     * @date 2026-05-29
     */
    bool isStarted() const;

   private:
    LDS_LDROBOT_LD19 parser_;
    uint16_t cachedDistancesMm_[kAngleCount];
    bool hasDistance_[kAngleCount];
    uint16_t validAngleCount_;

    /**
     * @brief Store one parsed LiDAR point into the degree cache.
     * @param point Parsed LD19 point produced by the low-level parser.
     * @author GOLETTA David
     * @date 2026-05-29
     */
    void cachePoint(const LDS_LDROBOT_LD19::ScanPoint& point);

    /**
     * @brief Convert a centi-degree angle to the nearest whole-degree slot.
     * @param angleDegX100 Angle in hundredths of a degree.
     * @return Cache index in the inclusive range [0, 359].
     * @author GOLETTA David
     * @date 2026-05-29
     */
    static uint16_t toNearestDegreeIndex(uint16_t angleDegX100);

    /**
     * @brief Wrap an angle index into the inclusive range [0, 359].
     * @param angleIndex Signed intermediate angle index.
     * @return Wrapped cache index in the inclusive range [0, 359].
     * @author GOLETTA David
     * @date 2026-05-29
     */
    static uint16_t wrapAngleIndex(int16_t angleIndex);
};