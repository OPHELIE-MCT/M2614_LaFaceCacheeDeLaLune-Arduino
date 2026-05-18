#ifndef RC_RECEIVER_H
#define RC_RECEIVER_H

#include <Arduino.h>

#include "RC_Channel.h"

static constexpr uint8_t RC_MAX_CHANNELS = 8;

class RC_Receiver {
   public:
    RC_Receiver(const uint8_t* pins, uint8_t count);
    ~RC_Receiver();
    /// Attach a CHANGE interrupt to every channel pin.
    /// Call once in setup(), after calibrate().
    void beginInterruptDriven();
    /// Legacy polled update — harmless no-op when interrupt-driven.
    void update();
    RC_Channel* getChannel(uint8_t index) const;
    uint8_t getChannelCount() const;

   private:
    RC_Channel* _channels[RC_MAX_CHANNELS] = {};
    uint8_t _count = 0;
};

#endif
