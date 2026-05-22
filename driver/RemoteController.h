#pragma once

#include <Arduino.h>

/**
 * @file RemoteController.h
 * @brief Object-oriented RC receiver reader with typed channel access.
 */

/**
 * @struct RCReceiverPins
 * @brief GPIO map for channels A to H of the RC receiver.
 * @author GOLETTA David
 * @date 2026-05-18
 */
struct RCReceiverPins {
    uint8_t a;
    uint8_t b;
    uint8_t c;
    uint8_t d;
    uint8_t e;
    uint8_t f;
    uint8_t g;
    uint8_t h;
};

/**
 * @enum RCChannel
 * @brief Logical index for RC channels A..H.
 * @author GOLETTA David
 * @date 2026-05-18
 */
enum class RCChannel : uint8_t {
    A = 0,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    Count
};

/**
 * @class RemoteController
 * @brief Reads 8 RC PWM channels and exposes typed controls.
 *
 * Channel roles are fixed as:
 * - A-D: joystick components, mapped to [-500, +500]
 * - E-F: buttons, read as pressed/released
 * - G-H: digital trim controls, quantized to 3 positions
 *
 * MC-8 pulse specifications used by this implementation:
 * - A-D joystick: 890 us, 1496 us (center), 2100 us
 * - E-F momentary button: 1496 us released, 1995 us pressed
 * - G-H trim switch: 997 us (low), 1496 us (center), 1995 us (high)
 * @author GOLETTA David
 * @date 2026-05-18
 */
class RemoteController {
   public:
    /**
     * @brief Construct the controller from RC receiver pin mapping.
     * @param pins GPIO configuration for channels A..H.
     * @return None.
     * @example
     * // RCReceiverPins pins{9,8,7,6,5,4,3,2};
     * // RemoteController rc(pins);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    explicit RemoteController(const RCReceiverPins& pins);

    /**
     * @brief Configure all RC channel pins as input and reset state.
     * @param none No parameters.
     * @return None.
     * @example
     * // rc.begin();
     * @author GOLETTA David
     * @date 2026-05-18
     */
    void begin();

    /**
     * @brief Poll all channels and refresh pulse width measurements.
     * @param none No parameters.
     * @return None.
     * @example
     * // rc.update();
     * @author GOLETTA David
     * @date 2026-05-18
     */
    void update();

    /**
     * @brief Read the last pulse width of a channel.
     * @param channel Channel identifier A..H.
     * @return Pulse width in microseconds. Returns 0 when signal is invalid.
     * @example
     * // uint16_t aUs = rc.getPulseWidthUs(RCChannel::A);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    uint16_t getPulseWidthUs(RCChannel channel) const;

    /**
     * @brief Read a joystick component mapped to [-500, +500].
     * @param channel One of A, B, C, or D.
     * @return Joystick component in [-500, +500], or 0 for invalid channel/signal.
     * @example
     * // int16_t x = rc.getJoystick(RCChannel::A);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    int16_t getJoystick(RCChannel channel) const;

    /**
     * @brief Read a button state from channel E or F.
     * @param channel One of E or F.
     * @param thresholdUs Press threshold in microseconds. Defaults to 1746.
     * @return True when pulse width is greater than or equal to threshold.
     * @example
     * // bool pressed = rc.getButton(RCChannel::E);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    bool getButton(RCChannel channel, uint16_t thresholdUs = 1746) const;

    /**
     * @brief Read a digital trim control value mapped to [0, 1000].
     *
     * This method quantizes channel G/H to 3 levels based on MC-8 trim pulses:
     * low=0, center=500, high=1000.
     *
     * @param channel One of G or H.
     * @return Quantized trim value (0, 500, or 1000), or 0 for invalid channel/signal.
     * @example
     * // uint16_t value = rc.getTrimValue(RCChannel::G);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    uint16_t getTrimValue(RCChannel channel) const;

    /**
     * @brief Read a digital trim control as a signed step.
     * @param channel One of G or H.
     * @return -1 (low), 0 (center), +1 (high), or 0 if invalid channel/signal.
     * @example
     * // int8_t trim = rc.getTrimStep(RCChannel::H);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    int8_t getTrimStep(RCChannel channel) const;

    /**
     * @brief Check whether the latest sample of a channel is valid.
     * @param channel Channel identifier A..H.
     * @return True if a valid pulse was recently measured, otherwise false.
     * @example
     * // bool ok = rc.isSignalValid(RCChannel::C);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    bool isSignalValid(RCChannel channel) const;

   private:
    static constexpr uint8_t kChannelCount = static_cast<uint8_t>(RCChannel::Count);
    static constexpr uint16_t kPulseMinUs = 890;
    static constexpr uint16_t kPulseMaxUs = 2100;
    static constexpr uint32_t kSignalTimeoutUs = 100000;

    static constexpr uint16_t kJoystickMinUs = 890;
    static constexpr uint16_t kJoystickCenterUs = 1496;
    static constexpr uint16_t kJoystickMaxUs = 2100;

    static constexpr uint16_t kButtonReleasedUs = 1496;
    static constexpr uint16_t kButtonPressedUs = 1995;

    static constexpr uint16_t kTrimLowUs = 997;
    static constexpr uint16_t kTrimCenterUs = 1496;
    static constexpr uint16_t kTrimHighUs = 1995;

    static constexpr int16_t kJoystickAbsMax = 500;
    static constexpr uint16_t kTrimValueMax = 1000;

    struct ChannelState {
        uint8_t pin = 0;
        bool wasHigh = false;
        bool signalValid = false;
        uint32_t startPulseTimestampUs = 0;
        uint16_t pulseWidthUs = 0;
    };

    ChannelState channels_[kChannelCount];

    /**
     * @brief Convert channel enum to index.
     * @param channel Channel identifier A..H.
     * @return Zero-based array index in [0, 7].
     * @example
     * // uint8_t i = toIndex(RCChannel::D);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static uint8_t toIndex(RCChannel channel);

    /**
     * @brief Check whether a channel is a joystick component.
     * @param channel Channel identifier A..H.
     * @return True for A-D, otherwise false.
     * @example
     * // bool isJoy = isJoystickChannel(RCChannel::B);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static bool isJoystickChannel(RCChannel channel);

    /**
     * @brief Check whether a channel is a button.
     * @param channel Channel identifier A..H.
     * @return True for E-F, otherwise false.
     * @example
     * // bool isBtn = isButtonChannel(RCChannel::F);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static bool isButtonChannel(RCChannel channel);

    /**
     * @brief Check whether a channel is a trim channel.
     * @param channel Channel identifier A..H.
     * @return True for G-H, otherwise false.
     * @example
     * // bool isTrim = isTrimChannel(RCChannel::G);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static bool isTrimChannel(RCChannel channel);

    /**
     * @brief Update one channel state from edge timing.
     * @param state Mutable state for one channel.
     * @param nowUs Current timestamp from micros().
     * @return None.
     * @example
     * // updateChannel(channels_[0], micros());
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static void updateChannel(ChannelState& state, uint32_t nowUs);

    /**
     * @brief Clamp a pulse width to receiver limits.
     * @param pulseUs Pulse width in microseconds.
     * @return Clamped pulse width in [1000, 2000].
     * @example
     * // uint16_t p = clampPulse(2100);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static uint16_t clampPulse(uint16_t pulseUs);

    /**
     * @brief Map pulse width to signed joystick range.
     * @param pulseUs Pulse width in microseconds.
     * @return Mapped value in [-500, +500].
     * @example
     * // int16_t joy = mapToJoystick(1500);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static int16_t mapToJoystick(uint16_t pulseUs);

    /**
     * @brief Map pulse width to trim value range.
     * @param pulseUs Pulse width in microseconds.
     * @return Quantized value in {0, 500, 1000}.
     * @example
     * // uint16_t value = mapToTrimValue(1500);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static uint16_t mapToTrimValue(uint16_t pulseUs);

    /**
     * @brief Map pulse width to digital trim step.
     * @param pulseUs Pulse width in microseconds.
     * @return -1 for low, 0 for center, +1 for high.
     * @example
     * // int8_t s = mapToTrimStep(1496);
     * @author GOLETTA David
     * @date 2026-05-18
     */
    static int8_t mapToTrimStep(uint16_t pulseUs);
};
