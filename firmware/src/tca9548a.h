#pragma once

// Driver for the TCA9548A I2C multiplexer (datasheet SCPS207). The two AS5600
// encoders share the fixed I2C address 0x36, so they sit behind separate mux
// channels. Writing a one-hot byte to the mux selects the active channel.

#include <cstdint>

class Tca9548a {
 public:
  explicit Tca9548a(uint8_t address = 0x70);

  // Selects channel 0..7 by writing the control register (one-hot bit mask,
  // datasheet table 6). Assumes Wire.begin() has already been called. Returns
  // true on I2C ACK.
  bool selectChannel(uint8_t channel);

 private:
  uint8_t address_;
};
