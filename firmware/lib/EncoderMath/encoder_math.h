#pragma once

// Pure encoder math for the AS5600 (12-bit magnetic encoder): wrap-around delta
// across the 4095<->0 boundary, and an exponential moving-average filter for
// velocity estimation. Host-tested; the hardware As5600Encoder uses these.

#include <cstdint>

class EncoderMath {
 public:
  static constexpr int kCounts = 4096;  // AS5600 RAW_ANGLE is 12-bit (0..4095)

  // Shortest signed delta from `prev` to `curr`, correctly handling wrap-around.
  // e.g. wrappedDelta(4090, 5) == +11, wrappedDelta(5, 4090) == -11.
  static int wrappedDelta(uint16_t prev, uint16_t curr);
};

// Exponential moving average: value = alpha*sample + (1-alpha)*value.
// First sample initializes the filter so it does not ramp up from zero.
class EmaFilter {
 public:
  explicit EmaFilter(float alpha) : alpha_(alpha) {}

  float update(float sample) {
    if (!initialized_) {
      value_ = sample;
      initialized_ = true;
    } else {
      value_ = alpha_ * sample + (1.0f - alpha_) * value_;
    }
    return value_;
  }

  float value() const { return value_; }

  void reset() {
    initialized_ = false;
    value_ = 0.0f;
  }

 private:
  float alpha_;
  float value_ = 0.0f;
  bool initialized_ = false;
};
