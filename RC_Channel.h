#ifndef RC_CHANNEL_H
#define RC_CHANNEL_H

#include <Arduino.h>

class RC_Channel {
   public:
    RC_Channel(uint8_t pin);
    void update();
    uint16_t getPulseWidth() const;
    int16_t getMappedPulseWidth() const;
    void calibrate(uint16_t min, uint16_t max);

   private:
    uint8_t _pin = 0;
    unsigned long _startPulseTimestamp = 0;
    unsigned long _pulseWidth = 0;
    bool _isHigh = false;
    uint16_t _min = 1000;
    uint16_t _max = 2000;
    uint16_t _maxMapped = 1000;
    bool _signalValid = false;
};

#endif
