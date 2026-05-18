#include "RC_Receiver.h"

RC_Receiver::RC_Receiver(const uint8_t* pins, uint8_t count) {
    _count = min(count, RC_MAX_CHANNELS);
    for (uint8_t i = 0; i < _count; i++) {
        _channels[i] = new RC_Channel(pins[i]);
    }
}

RC_Receiver::~RC_Receiver() {
    for (uint8_t i = 0; i < _count; i++) {
        delete _channels[i];
        _channels[i] = nullptr;
    }
}

void RC_Receiver::update() {
    for (uint8_t i = 0; i < _count; i++) {
        _channels[i]->update();
    }
}

RC_Channel* RC_Receiver::getChannel(uint8_t index) const {
    if (index >= _count) {
        return nullptr;
    }
    return _channels[index];
}

uint8_t RC_Receiver::getChannelCount() const {
    return _count;
}
