#include "RC_Receiver.h"

// ── ISR shims ─────────────────────────────────────────────────────────────
// One static pointer per possible channel. Populated by beginInterruptDriven().
static RC_Channel* _isrChannels[RC_MAX_CHANNELS] = {};

static void isrRcCh0() {
    if (_isrChannels[0]) _isrChannels[0]->update();
}
static void isrRcCh1() {
    if (_isrChannels[1]) _isrChannels[1]->update();
}
static void isrRcCh2() {
    if (_isrChannels[2]) _isrChannels[2]->update();
}
static void isrRcCh3() {
    if (_isrChannels[3]) _isrChannels[3]->update();
}
static void isrRcCh4() {
    if (_isrChannels[4]) _isrChannels[4]->update();
}
static void isrRcCh5() {
    if (_isrChannels[5]) _isrChannels[5]->update();
}
static void isrRcCh6() {
    if (_isrChannels[6]) _isrChannels[6]->update();
}
static void isrRcCh7() {
    if (_isrChannels[7]) _isrChannels[7]->update();
}

static void (*const _isrTable[RC_MAX_CHANNELS])() = {
    isrRcCh0, isrRcCh1, isrRcCh2, isrRcCh3,
    isrRcCh4, isrRcCh5, isrRcCh6, isrRcCh7};

// ── RC_Receiver implementation ────────────────────────────────────────────
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

void RC_Receiver::beginInterruptDriven() {
    for (uint8_t i = 0; i < _count; i++) {
        _isrChannels[i] = _channels[i];
        attachInterrupt(
            digitalPinToInterrupt(_channels[i]->getPin()),
            _isrTable[i],
            CHANGE);
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
