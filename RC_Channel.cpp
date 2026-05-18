#include "RC_Channel.h"

RC_Channel::RC_Channel(uint8_t pin) : _pin(pin) {
    pinMode(_pin, INPUT);
}

void RC_Channel::update() {
    unsigned long currentTime = micros();
    bool isHigh = digitalRead(_pin) == HIGH;

    if (isHigh && !_isHigh) {
        _startPulseTimestamp = currentTime;
    } else if (!isHigh && _isHigh) {
        _pulseWidth = currentTime - _startPulseTimestamp;
        _signalValid = true;
    } else if (!isHigh && currentTime - _startPulseTimestamp > 50000) {
        _pulseWidth = 0;
        _signalValid = false;
    }
    _isHigh = isHigh;
}

uint16_t RC_Channel::getPulseWidth() const {
    return _pulseWidth;
}

int16_t RC_Channel::getMappedPulseWidth() const {
    if (!_signalValid) {
        return 0;
    } else if (_pulseWidth > _max) {
        return _maxMapped;
    } else if (_pulseWidth < _min) {
        return -_maxMapped;
    } else {
        return map(_pulseWidth, _min, _max, -_maxMapped, _maxMapped);
    }
}

void RC_Channel::calibrate(uint16_t min, uint16_t max) {
    _min = min;
    _max = max;
}
