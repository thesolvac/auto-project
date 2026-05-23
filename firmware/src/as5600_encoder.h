#pragma once

// AS5600 12-bit magnetic encoder behind a TCA9548A channel. Reads the RAW_ANGLE
// register (0x0C high byte, 0x0D low byte; datasheet section "Output Stage"),
// accumulates a wrap-aware count, and low-pass-filters the angular velocity.
// Wrap-around and EMA math live in the host-tested EncoderMath/EmaFilter.

#include <cstdint>

#include "encoder_math.h"
#include "tca9548a.h"

class As5600Encoder {
 public:
  // `mux` selects this encoder's `channel` before each read. `ema_alpha` is the
  // velocity low-pass coefficient (CLAUDE.md: 0.2).
  As5600Encoder(Tca9548a& mux, uint8_t channel, float ema_alpha = 0.2f);

  // Read the raw 12-bit angle (0..4095). Returns false on I2C error.
  bool readRaw(uint16_t& raw);

  // Select the channel, read the angle, integrate the wrapped delta, and update
  // the filtered velocity. `dt` is the time since the previous update [s].
  void update(float dt);

  long totalCounts() const { return total_counts_; }
  float velocityCountsPerSec() const { return velocity_filter_.value(); }

 private:
  Tca9548a& mux_;
  uint8_t channel_;
  uint8_t address_ = 0x36;  // AS5600 fixed I2C address
  EmaFilter velocity_filter_;
  uint16_t last_raw_ = 0;
  bool has_last_ = false;
  long total_counts_ = 0;
};
