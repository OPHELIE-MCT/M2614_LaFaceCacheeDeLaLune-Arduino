#include "RemoteController.h"

RemoteController::RemoteController(const RCReceiverPins& pins) {
    channels_[toIndex(RCChannel::A)].pin = pins.a;
    channels_[toIndex(RCChannel::B)].pin = pins.b;
    channels_[toIndex(RCChannel::C)].pin = pins.c;
    channels_[toIndex(RCChannel::D)].pin = pins.d;
    channels_[toIndex(RCChannel::E)].pin = pins.e;
    channels_[toIndex(RCChannel::F)].pin = pins.f;
    channels_[toIndex(RCChannel::G)].pin = pins.g;
    channels_[toIndex(RCChannel::H)].pin = pins.h;
}

void RemoteController::begin() {
    for (uint8_t i = 0; i < kChannelCount; ++i) {
        pinMode(channels_[i].pin, INPUT);
        channels_[i].wasHigh = false;
        channels_[i].signalValid = false;
        channels_[i].startPulseTimestampUs = 0;
        channels_[i].pulseWidthUs = 0;
    }
}

void RemoteController::update() {
    const uint32_t nowUs = micros();
    for (uint8_t i = 0; i < kChannelCount; ++i) {
        updateChannel(channels_[i], nowUs);
    }
}

uint16_t RemoteController::getPulseWidthUs(RCChannel channel) const {
    const ChannelState& state = channels_[toIndex(channel)];
    if (!state.signalValid) {
        return 0;
    }
    return state.pulseWidthUs;
}

int16_t RemoteController::getJoystick(RCChannel channel) const {
    if (!isJoystickChannel(channel)) {
        return 0;
    }

    const ChannelState& state = channels_[toIndex(channel)];
    if (!state.signalValid) {
        return 0;
    }

    return mapToJoystick(state.pulseWidthUs);
}

bool RemoteController::getButton(RCChannel channel, uint16_t thresholdUs) const {
    if (!isButtonChannel(channel)) {
        return false;
    }

    const ChannelState& state = channels_[toIndex(channel)];
    if (!state.signalValid) {
        return false;
    }

    const uint16_t pulse = clampPulse(state.pulseWidthUs);
    const uint16_t appliedThreshold =
        (thresholdUs < kButtonReleasedUs || thresholdUs > kButtonPressedUs)
            ? static_cast<uint16_t>((kButtonReleasedUs + kButtonPressedUs) / 2)
            : thresholdUs;
    return pulse >= appliedThreshold;
}

uint16_t RemoteController::getTrimValue(RCChannel channel) const {
    if (!isTrimChannel(channel)) {
        return 0;
    }

    const ChannelState& state = channels_[toIndex(channel)];
    if (!state.signalValid) {
        return 0;
    }

    return mapToTrimValue(state.pulseWidthUs);
}

int8_t RemoteController::getTrimStep(RCChannel channel) const {
    if (!isTrimChannel(channel)) {
        return 0;
    }

    const ChannelState& state = channels_[toIndex(channel)];
    if (!state.signalValid) {
        return 0;
    }

    return mapToTrimStep(state.pulseWidthUs);
}

bool RemoteController::isSignalValid(RCChannel channel) const {
    return channels_[toIndex(channel)].signalValid;
}

uint8_t RemoteController::toIndex(RCChannel channel) {
    return static_cast<uint8_t>(channel);
}

bool RemoteController::isJoystickChannel(RCChannel channel) {
    return channel == RCChannel::A || channel == RCChannel::B || channel == RCChannel::C ||
           channel == RCChannel::D;
}

bool RemoteController::isButtonChannel(RCChannel channel) {
    return channel == RCChannel::E || channel == RCChannel::F;
}

bool RemoteController::isTrimChannel(RCChannel channel) {
    return channel == RCChannel::G || channel == RCChannel::H;
}

void RemoteController::updateChannel(ChannelState& state, uint32_t nowUs) {
    const bool isHigh = digitalRead(state.pin) == HIGH;

    if (isHigh && !state.wasHigh) {
        state.startPulseTimestampUs = nowUs;
    } else if (!isHigh && state.wasHigh) {
        state.pulseWidthUs = clampPulse(static_cast<uint16_t>(nowUs - state.startPulseTimestampUs));
        state.signalValid = true;
    } else if (!isHigh && (nowUs - state.startPulseTimestampUs) > kSignalTimeoutUs) {
        state.pulseWidthUs = 0;
        state.signalValid = false;
    }

    state.wasHigh = isHigh;
}

uint16_t RemoteController::clampPulse(uint16_t pulseUs) {
    if (pulseUs < kPulseMinUs) {
        return kPulseMinUs;
    }
    if (pulseUs > kPulseMaxUs) {
        return kPulseMaxUs;
    }
    return pulseUs;
}

int16_t RemoteController::mapToJoystick(uint16_t pulseUs) {
    const uint16_t pulseClamped = clampPulse(pulseUs);

    if (pulseClamped <= kJoystickCenterUs) {
        return static_cast<int16_t>(
            map(pulseClamped, kJoystickMinUs, kJoystickCenterUs, -kJoystickAbsMax, 0));
    }

    return static_cast<int16_t>(
        map(pulseClamped, kJoystickCenterUs, kJoystickMaxUs, 0, kJoystickAbsMax));
}

uint16_t RemoteController::mapToTrimValue(uint16_t pulseUs) {
    const int8_t trimStep = mapToTrimStep(pulseUs);
    if (trimStep < 0) {
        return 0;
    }
    if (trimStep > 0) {
        return kTrimValueMax;
    }
    return kTrimValueMax / 2;
}

int8_t RemoteController::mapToTrimStep(uint16_t pulseUs) {
    const uint16_t pulseClamped = clampPulse(pulseUs);
    const uint16_t lowToCenterMid = static_cast<uint16_t>((kTrimLowUs + kTrimCenterUs) / 2);
    const uint16_t centerToHighMid = static_cast<uint16_t>((kTrimCenterUs + kTrimHighUs) / 2);

    if (pulseClamped < lowToCenterMid) {
        return -1;
    }
    if (pulseClamped > centerToHighMid) {
        return 1;
    }
    return 0;
}
