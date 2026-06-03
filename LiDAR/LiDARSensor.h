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
    static constexpr uint8_t kMaxQueryAngleDeltaDeg = 6;
    static constexpr uint16_t kMinUniqueAnglesForCommit = 120;
    static constexpr uint16_t kMinAcceptedPointsForCommit = 180;

    /**
     * @struct QueryResult
     * @brief Structured result returned by high-level angle queries.
     * @author GOLETTA David
     * @date 2026-06-01
     */
    struct QueryResult {
        bool found;
        uint16_t requested_angle_deg;
        uint16_t matched_angle_deg_x100;
        int16_t angular_error_deg_x100;
        int16_t distance_mm;
        uint8_t quality;
        uint32_t scan_id;
        uint16_t valid_point_count;
    };

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
    uint16_t readAngleAt(uint16_t angleInDegrees) const;

    /**
     * @brief Query the last committed scan and return detailed match metadata.
     * @description The query searches valid points around the requested angle
     * within a bounded window to avoid long-range fallback across unrelated
     * sectors when a local hole exists.
     * @param angleInDegrees Requested heading in whole degrees.
     * @return Structured query result containing the matched source angle,
     * distance, quality, and scan metadata.
     * @author GOLETTA David
     * @date 2026-06-01
     */
    QueryResult queryAngleAt(uint16_t angleInDegrees) const;

    /**
     * @brief Check whether the wrapper has been attached to a valid serial stream.
     * @return True when the low-level parser has been started, otherwise false.
     * @author GOLETTA David
     * @date 2026-05-29
     */
    bool isStarted() const;

    /**
     * @brief Return the number of unique angle bins in the last committed scan.
     * @return Unique valid angle count from the last committed scan buffer.
     * @author GOLETTA David
     * @date 2026-06-01
     */
    uint16_t getLastValidPointCount() const;

    /**
     * @brief Return the monotonic identifier of the last committed scan.
     * @return Last committed scan id. Returns 0 before first valid commit.
     * @author GOLETTA David
     * @date 2026-06-01
     */
    uint32_t getLastCommittedScanId() const;

    /**
     * @brief Return how many write scans were rejected by quality gates.
     * @return Number of dropped scans since last reset.
     * @author GOLETTA David
     * @date 2026-06-01
     */
    uint32_t getDroppedScanCount() const;

    /**
     * @brief Check whether at least one valid scan has been committed.
     * @return True when the read buffer contains a committed scan.
     * @author GOLETTA David
     * @date 2026-06-01
     */
    bool hasValidScan() const;

   private:
    /**
     * @struct ScanBuffer
     * @brief Fixed-size storage for one revolution candidate or committed scan.
     * @author GOLETTA David
     * @date 2026-06-01
     */
    struct ScanBuffer {
        uint16_t distance_mm[kAngleCount];
        uint16_t angle_deg_x100[kAngleCount];
        uint8_t quality[kAngleCount];
        bool valid[kAngleCount];
        uint16_t unique_angle_count;
        uint16_t accepted_point_count;
    };

    LDS_LDROBOT_LD19 parser_;
    ScanBuffer scan_buffers_[2];
    uint8_t read_buffer_index_;
    uint8_t write_buffer_index_;
    bool has_committed_scan_;
    uint32_t committed_scan_id_;
    uint32_t dropped_scan_count_;

    /**
     * @brief Store one parsed LiDAR point into the active write buffer.
     * @param point Parsed LD19 point produced by the low-level parser.
     * @author GOLETTA David
     * @date 2026-06-01
     */
    void accumulatePointToWriteBuffer(const LDS_LDROBOT_LD19::ScanPoint& point);

    /**
     * @brief Finalize the current write scan and rotate buffers when valid.
     * @description The write buffer is committed only when quality thresholds
     * are met. Commit is O(1) via index swap. The next write buffer is always
     * reset after finalization.
     * @author GOLETTA David
     * @date 2026-06-01
     */
    void finalizeWriteBuffer();

    /**
     * @brief Evaluate whether the active write buffer is good enough to commit.
     * @return True when commit thresholds are met, otherwise false.
     * @author GOLETTA David
     * @date 2026-06-01
     */
    bool shouldCommitWriteBuffer() const;

    /**
     * @brief Reset one scan buffer to an empty state.
     * @param bufferIndex Buffer index in the inclusive range [0, 1].
     * @author GOLETTA David
     * @date 2026-06-01
     */
    void clearScanBuffer(uint8_t bufferIndex);

    /**
     * @brief Convert a centi-degree angle to the nearest whole-degree slot.
     * @param angleDegX100 Angle in hundredths of a degree.
     * @return Cache index in the inclusive range [0, 359].
     * @author GOLETTA David
     * @date 2026-06-01
     */
    static uint16_t toNearestDegreeIndex(uint16_t angleDegX100);

    /**
     * @brief Wrap an angle index into the inclusive range [0, 359].
     * @param angleIndex Signed intermediate angle index.
     * @return Wrapped cache index in the inclusive range [0, 359].
     * @author GOLETTA David
     * @date 2026-06-01
     */
    static uint16_t wrapAngleIndex(int16_t angleIndex);

    /**
     * @brief Compute shortest signed angular error from requested to matched.
     * @param requestedAngleDeg Requested heading in degrees.
     * @param matchedAngleDegX100 Matched heading in hundredths of a degree.
     * @return Signed shortest angular error in hundredths of a degree.
     * @author GOLETTA David
     * @date 2026-06-01
     */
    static int16_t shortestAngularErrorDegX100(uint16_t requestedAngleDeg, uint16_t matchedAngleDegX100);

    /**
     * @brief Access the committed read buffer.
     * @return Immutable reference to the current read buffer.
     * @author GOLETTA David
     * @date 2026-06-01
     */
    const ScanBuffer& readBuffer() const;

    /**
     * @brief Access the active write buffer.
     * @return Mutable reference to the current write buffer.
     * @author GOLETTA David
     * @date 2026-06-01
     */
    ScanBuffer& writeBuffer();
};