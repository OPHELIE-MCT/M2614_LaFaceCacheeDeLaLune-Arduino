#include "MecanumMixer.h"

WheelSpeeds MecanumMixer::compute(int16_t vx, int16_t vy, int16_t omega) {
    WheelSpeeds speeds;
    speeds.fl = clamp((int32_t)vy + vx + omega);
    speeds.fr = clamp((int32_t)vy - vx - omega);
    speeds.bl = clamp((int32_t)vy - vx + omega);
    speeds.br = clamp((int32_t)vy + vx - omega);
    return speeds;
}

int16_t MecanumMixer::clamp(int32_t value) {
    if (value > 1000) return 1000;
    if (value < -1000) return -1000;
    return (int16_t)value;
}
