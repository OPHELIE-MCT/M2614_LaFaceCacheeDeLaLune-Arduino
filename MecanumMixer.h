#ifndef MECANUM_MIXER_H
#define MECANUM_MIXER_H

#include <Arduino.h>

struct WheelSpeeds {
    int16_t fl;
    int16_t fr;
    int16_t bl;
    int16_t br;
};

class MecanumMixer {
   public:
    static WheelSpeeds compute(int16_t vx, int16_t vy, int16_t omega);

   private:
    static int16_t clamp(int32_t value);
};

#endif
