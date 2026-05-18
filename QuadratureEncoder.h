#ifndef QUADRATURE_ENCODER_H
#define QUADRATURE_ENCODER_H

#include <Arduino.h>

/// Number of wheel encoders on the robot.
static constexpr size_t ENCODER_COUNT = 4;

/// Encoder index constants.
enum EncoderIndex : uint8_t { ENC_FL = 0,
                              ENC_FR = 1,
                              ENC_BL = 2,
                              ENC_BR = 3 };

namespace QuadratureEncoder {

/// Initialise ISRs for all four quadrature encoders.
/// Must be called after pinSetup().
void begin();

/// Read the accumulated pulse count for one encoder and reset it to zero.
/// Returns a signed value: positive = forward, negative = reverse.
/// This function is ISR-safe (disables interrupts briefly).
int32_t readAndResetDelta(uint8_t index);

/// Read the accumulated pulse count without resetting.
int32_t readDelta(uint8_t index);

/// Reset the accumulated pulse count for one encoder to zero.
void resetDelta(uint8_t index);

}  // namespace QuadratureEncoder

#endif
