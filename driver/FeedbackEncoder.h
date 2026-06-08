#pragma once

#include <Arduino.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

/**
 * @struct EncoderChannelPins
 * @brief GPIO mapping for one quadrature encoder channel pair.
 *
 * Channel A is carried for complete wiring documentation and future expansion.
 * The current implementation only uses channel B as required by hardware.
 * @author GOLETTA David
 * @date 2026-05-18
 */
struct EncoderChannelPins {
    uint8_t a;
    uint8_t b;
};

/**
 * @struct FeedbackEncoderPins
 * @brief Complete encoder GPIO mapping for all four wheels.
 * @author GOLETTA David
 * @date 2026-05-18
 */
struct FeedbackEncoderPins {
    EncoderChannelPins frontLeft;
    EncoderChannelPins frontRight;
    EncoderChannelPins rearLeft;
    EncoderChannelPins rearRight;
};

/**
 * @struct EncoderSpeedSnapshot
 * @brief Speed snapshot in pulses per 50 ms window for each wheel.
 * @author GOLETTA David
 * @date 2026-05-18
 */
struct EncoderSpeedSnapshot {
    int32_t frontLeft;
    int32_t frontRight;
    int32_t rearLeft;
    int32_t rearRight;
};

/**
 * @struct WheelSpeedTargets
 * @brief Per-wheel speed setpoints for the PID speed controllers.
 *
 * All values are expressed in pulses per 50 ms, matching the unit produced by
 * EncoderSpeedSnapshot, so they can be compared directly.
 * @author GOLETTA David
 * @date 2026-06-08
 */
struct WheelSpeedTargets {
    float frontLeft;
    float frontRight;
    float rearLeft;
    float rearRight;
};

/**
 * @class FeedbackEncoder
 * @brief Reads wheel feedback encoders using channel B interrupts only.
 *
 * Interrupt handlers accumulate edge counters on each channel B transition.
 * A Zephyr thread drains those counters every 50 ms and stores the result in
 * the private `currentSpeed` snapshot.
 * @author GOLETTA David
 * @date 2026-05-18
 */
class FeedbackEncoder {
   public:
    static constexpr int32_t kSamplePeriodMs = 50;

    /**
     * @brief Construct the encoder reader from full wheel encoder pin mapping.
     * @param pins Complete channel A/B pin mapping for all wheels.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    explicit FeedbackEncoder(const FeedbackEncoderPins& pins);

    /**
     * @brief Destroy the encoder reader and stop background processing.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    ~FeedbackEncoder();

    /**
     * @brief Configure pin modes, attach channel B interrupts, and start thread.
     * @details This method is idempotent and returns immediately if already started.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    void begin();

    /**
     * @brief Stop thread processing and detach channel B interrupts.
     * @details This method is idempotent and returns immediately if not running.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    void end();

    /**
     * @brief Read the latest speed snapshot.
     * @return EncoderSpeedSnapshot containing pulses measured during last 50 ms cycle.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    EncoderSpeedSnapshot getCurrentSpeed() const;

   private:
    enum WheelIndex : uint8_t {
        kFrontLeft = 0,
        kFrontRight,
        kRearLeft,
        kRearRight,
        kWheelCount
    };

    static constexpr size_t kThreadStackSize = 1024;
    static constexpr int kThreadPriority = 7;
    static constexpr int32_t kThreadPeriodMs = kSamplePeriodMs;

    FeedbackEncoderPins pins_;
    atomic_t edgeCounters_[kWheelCount];
    bool running_;
    k_tid_t threadId_;
    struct k_thread threadData_;
    k_thread_stack_t threadStack_[kThreadStackSize];

    // Requested name: keeps latest per-wheel pulses measured over 50 ms.
    EncoderSpeedSnapshot currentSpeed;

    static FeedbackEncoder* activeInstance_;

    /**
     * @brief Reset all interrupt counters and speed snapshot values.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    void resetState();

    /**
     * @brief Increment the interrupt counter for one wheel.
     * @param wheel Wheel index corresponding to channel B edge source.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    void onChannelBEdge(WheelIndex wheel);

    /**
     * @brief Drain edge counters into the current speed snapshot.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    void processCounters();

    /**
     * @brief Zephyr thread entry that updates speed every 50 ms.
     * @param p1 FeedbackEncoder instance pointer.
     * @param p2 Unused thread argument.
     * @param p3 Unused thread argument.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static void processingThreadEntry(void* p1, void* p2, void* p3);

    /**
     * @brief Interrupt trampoline for front-left encoder channel B.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static void isrFrontLeft();

    /**
     * @brief Interrupt trampoline for front-right encoder channel B.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static void isrFrontRight();

    /**
     * @brief Interrupt trampoline for rear-left encoder channel B.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static void isrRearLeft();

    /**
     * @brief Interrupt trampoline for rear-right encoder channel B.
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static void isrRearRight();
};
